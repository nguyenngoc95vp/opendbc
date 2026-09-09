import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, rate_limit, structs, uds
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import (BREAKAWAY_FRAMES, RADAR_ADDR, AdvertisedLead, RadarSessionManager,
                                            RadarSessionState, StandstillHold, create_radar_session_msg)
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaFlags, TorqueInterceptorControllerParams

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

LONG_BUSES = (0, 2)


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    if not CP.flags & MazdaFlags.GEN1:
      raise NotImplementedError(f"unsupported platform: {CP.carFingerprint}")
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.ti_params = TorqueInterceptorControllerParams(CP)
    self.ti_apply_torque_last = 0
    self.ti_rt_torque_last = 0
    self.ti_rt_torque_last_ts = None
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.stop_and_go = StandstillHold()
    self.lead_adv = AdvertisedLead()
    self.long_counter = 0
    self.radar_counter = 0
    self.radar_session = RadarSessionManager()
    self.accel_last = 0.
    self.release_ramp = None
    self.breakaway_frames = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0
    ti_apply_torque = 0

    if hasattr(self.params, 'STEER_MAX_LOOKUP'):
      steer_max = round(float(np.interp(CS.out.vEgoRaw, self.params.STEER_MAX_LOOKUP[0],
                                         self.params.STEER_MAX_LOOKUP[1])))
    else:
      steer_max = self.params.STEER_MAX

    if CC.latActive:
      new_torque = int(round(CC.actuators.torque * steer_max))
      if hasattr(self.params, 'EPS_CEILING_LOOKUP'):
        eps_ceiling = round(float(np.interp(CS.out.vEgoRaw, self.params.EPS_CEILING_LOOKUP[0],
                                            self.params.EPS_CEILING_LOOKUP[1])))
        new_torque = int(np.clip(new_torque, -eps_ceiling, eps_ceiling))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params, steer_max)

    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      # Keep the TI frame alive while taking over. The lateral controller may already have
      # paused (CC.latActive=False), but a healthy TI RUN state still gets a controlled
      # minimum command instead of an immediate zero.
      if CS.ti_lkas_allowed:
        if hasattr(self.ti_params, 'STEER_MAX_LOOKUP'):
          ti_steer_max = round(float(np.interp(CS.out.vEgoRaw, self.ti_params.STEER_MAX_LOOKUP[0],
                                               self.ti_params.STEER_MAX_LOOKUP[1])))
        else:
          ti_steer_max = self.ti_params.STEER_MAX

        if CC.latActive and not CS.out.steeringPressed:
          ti_new_torque = int(round(CC.actuators.torque * ti_steer_max))
          if CS.out.vEgoRaw < self.ti_params.STANDSTILL_ZERO_SPEED:
            ti_new_torque = 0
          ti_apply_torque = apply_driver_steer_torque_limits(ti_new_torque, self.ti_apply_torque_last,
                                                             CS.out.steeringTorque, self.ti_params, ti_steer_max)
        elif CS.out.steeringPressed:
          ti_min_torque = 15
          prev = self.ti_apply_torque_last
          if prev > ti_min_torque:
            ti_apply_torque = max(prev - self.ti_params.STEER_DELTA_DOWN, ti_min_torque)
          elif prev < -ti_min_torque:
            ti_apply_torque = min(prev + self.ti_params.STEER_DELTA_DOWN, -ti_min_torque)
          else:
            ti_apply_torque = int(np.clip(prev, -ti_min_torque, ti_min_torque))
        else:
          prev = self.ti_apply_torque_last
          if prev > 0:
            ti_apply_torque = max(prev - self.ti_params.STEER_DELTA_DOWN, 0)
          elif prev < 0:
            ti_apply_torque = min(prev + self.ti_params.STEER_DELTA_DOWN, 0)

        if self.ti_rt_torque_last_ts is None:
          self.ti_rt_torque_last_ts = now_nanos
        highest_torque = max(self.ti_rt_torque_last, 0) + self.ti_params.STEER_MAX_RT_DELTA
        lowest_torque = min(self.ti_rt_torque_last, 0) - self.ti_params.STEER_MAX_RT_DELTA
        ti_apply_torque = max(min(ti_apply_torque, highest_torque), lowest_torque)
        if now_nanos - self.ti_rt_torque_last_ts > self.ti_params.STEER_RT_INTERVAL_NS:
          self.ti_rt_torque_last = ti_apply_torque
          self.ti_rt_torque_last_ts = now_nanos
      else:
        prev = self.ti_apply_torque_last
        if prev > 0:
          ti_apply_torque = max(prev - self.ti_params.STEER_DELTA_DOWN, 0)
        elif prev < 0:
          ti_apply_torque = min(prev + self.ti_params.STEER_DELTA_DOWN, 0)
        self.ti_rt_torque_last = ti_apply_torque
        self.ti_rt_torque_last_ts = now_nanos
      self.ti_apply_torque_last = ti_apply_torque

    stock_mrcc_owns_cruise = self.CP.openpilotLongitudinalControl and not CS.radar_was_silenced
    if CC.cruiseControl.cancel and not stock_mrcc_owns_cruise:
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if self.resume_requested(CC) and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.update_longitudinal(CC, CC_SP, CS))

    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))
    if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      can_sends.append(mazdacan.create_ti_steering_control(self.packer, ti_apply_torque))

    icbm_suppress = CC.cruiseControl.cancel or CC.cruiseControl.resume or CS.cancel_button == 1
    if not icbm_suppress:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = self.accel_last

    self.frame += 1
    return new_actuators, can_sends

  def resume_requested(self, CC) -> bool:
    return not self.CP.openpilotLongitudinalControl and CC.cruiseControl.resume

  def update_longitudinal(self, CC, CC_SP, CS):
    can_sends = []

    stock_radar_alive = CS.stock_radar_alive
    setup_ok = CS.fsc_settled and not (stock_radar_alive and CS.out.cruiseState.enabled)
    session_state = self.radar_session.update(setup_ok, stock_radar_alive, CC_SP.stockEcuHandBack,
                                              standstill=CS.out.standstill,
                                              session_refused=CS.radar_session_refused)
    radar_master = session_state in (RadarSessionState.SILENCED, RadarSessionState.HANDBACK)

    if self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
      if session_state == RadarSessionState.SILENCING:
        can_sends.append(create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING))
      elif session_state == RadarSessionState.HANDBACK:
        can_sends.append(create_radar_session_msg(uds.SESSION_TYPE.DEFAULT))
      elif session_state == RadarSessionState.SILENCED:
        can_sends.append(make_tester_present_msg(RADAR_ADDR, 0, suppress_response=True))

    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    long_engaged = CC.enabled
    sm = self.stop_and_go
    sm.update(long_engaged, stopping, CS.out.standstill, CC.actuators.accel, CS.brake_hold,
              gas_pressed=CS.out.gasPressed)
    self.lead_adv.update(CC.hudControl.leadVisible, CC_SP.leadOne.dRel,
                         CC_SP.leadOne.vRel, sm.holding)

    if sm.just_released:
      self.release_ramp = CarControllerParams.ACCEL_HOLD_LATCHED if sm.latched_release else \
                          CarControllerParams.ACCEL_RELEASE_BAND
    elif sm.holding or not CC.longActive:
      self.release_ramp = None

    accel = 0.
    if CC.longActive:
      accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      if self.release_ramp is None or not CS.out.standstill:
        self.breakaway_frames = 0
      else:
        self.breakaway_frames += 1
      breakaway = CS.out.standstill and self.breakaway_frames <= BREAKAWAY_FRAMES
      ramp_ceiling = max(accel, CarControllerParams.ACCEL_BREAKAWAY_MAX)
      if self.release_ramp is not None and (self.release_ramp < accel or breakaway):
        accel = self.release_ramp
        if not (sm.latched_release and CS.brake_hold):
          self.release_ramp = min(self.release_ramp + CarControllerParams.ACCEL_RELEASE_RAMP * DT_CTRL,
                                  ramp_ceiling)
      else:
        self.release_ramp = None
        accel = rate_limit(accel, self.accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                           CarControllerParams.ACCEL_WINDUP_LIMIT)
      if sm.car_has_hold:
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif sm.holding:
        accel = min(accel, 0.) if CC.actuators.accel <= 0. else min(self.accel_last, 0.)
      if sm.resume_unlatching:
        if sm.latched_release:
          accel = min(max(accel, CarControllerParams.ACCEL_HOLD_LATCHED),
                      CarControllerParams.ACCEL_RESUME_PULSE_MAX)
        else:
          accel = min(accel, 0.)
    self.accel_last = accel

    if radar_master and self.frame % CarControllerParams.RADAR_STEP == 0:
      for bus in LONG_BUSES:
        can_sends.extend(mazdacan.create_radar_frames(bus, self.radar_counter, self.lead_adv.lead))
      self.radar_counter += 1

    if radar_master and self.frame % CarControllerParams.LONG_STEP == 0:
      acc_available = CS.out.cruiseState.available
      gap = (int(CC.hudControl.leadDistanceBars) or 2) if (long_engaged or acc_available) else 0
      acc_active_2 = sm.acc_active_2 if long_engaged else False
      for bus in LONG_BUSES:
        can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.long_counter, accel,
                                                     long_active=long_engaged, acc_available=acc_available,
                                                     brake_pressed=CS.out.brakePressed,
                                                     stopping=sm.stop_bits, resume_unlatching=sm.resume_unlatching))
        can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, long_engaged, acc_available, gap,
                                                  self.lead_adv.has_lead, self.lead_adv.ctrl_phase,
                                                  acc_active_2))
      self.long_counter += 1

    return can_sends
