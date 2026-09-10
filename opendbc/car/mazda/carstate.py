from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaFlags, TorqueInterceptorState
from opendbc.sunnypilot.car.mazda.carstate_ext import CarStateExt

ButtonType = structs.CarState.ButtonEvent.Type

FSC_SETTLE_FRAMES = int(CarControllerParams.FSC_SETTLE_T / DT_CTRL)
STOCK_RADAR_ALIVE_FRAMES = int(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)
STOCK_RADAR_GUARD_FRAMES = int(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL)
CANCEL_CONTEXT_FRAMES = int(CarControllerParams.CANCEL_CONTEXT_T / DT_CTRL)
CAM_LANEINFO_FRESH_FRAMES = int(CarControllerParams.CAM_LANEINFO_FRESH_T / DT_CTRL)


class CarState(CarStateBase, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.ti_lkas_allowed = False

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.cancel_button = 0
    self.resume_button = 0
    self.main_button = 0

    self.cruise_available = False
    self.cruise_enabled = False
    self.cruise_enabled_blocked = True
    self.brake_pressed_prev = False
    self.stock_radar_silent_frames = 0
    self.radar_was_silenced = False
    self.cancel_context_frames = 0
    self.cam_laneinfo_seen = False
    self.cam_laneinfo_silent_frames = 0
    self.cam_empty_seen = False
    self.radar_session_refused = False
    self.fsc_settled_frames = 0
    self.brake_hold = False

  @property
  def fsc_settled(self) -> bool:
    return self.fsc_settled_frames >= FSC_SETTLE_FRAMES

  @property
  def stock_radar_alive(self) -> bool:
    return self.stock_radar_silent_frames < STOCK_RADAR_ALIVE_FRAMES

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ti_enabled = bool(self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR)

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    self.brake_hold = cp.vl["GEAR"]["BRAKE_HOLD"] == 1

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    if ti_enabled:
      cp_body = can_parsers[Bus.body]
      ti_feedback = cp_body.vl["TI_FEEDBACK"]
      ret.steeringTorque = ti_feedback["TI_TORQUE_SENSOR"]
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 5, 5)
      self.ti_lkas_allowed = cp_body.can_valid and \
        ti_feedback["VERSION_NUMBER"] in (1, 16) and \
        ti_feedback["STATE"] == TorqueInterceptorState.RUN and \
        not any(ti_feedback[s] for s in ("VIOL", "ERROR", "RAMP_DOWN"))
      ret_sp.torqueInterceptorReady = self.ti_lkas_allowed
    else:
      ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
      ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD, 5)

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"], cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1
    if self.CP.minSteerSpeed > 0:
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    if len(cp_cam.vl_all["CAM_LANEINFO"]["LANE_LINES"]) > 0:
      self.cam_laneinfo_seen = True
      self.cam_laneinfo_silent_frames = 0
    else:
      self.cam_laneinfo_silent_frames += 1
    cam_laneinfo_fresh = self.cam_laneinfo_seen and self.cam_laneinfo_silent_frames < CAM_LANEINFO_FRESH_FRAMES

    if not self.cam_empty_seen:
      self.cam_empty_seen = len(cp_cam.vl_all["CAM_EMPTY"]["STATUS"]) > 0
    cam_empty = cp_cam.vl["CAM_EMPTY"]
    ped = cp_cam.vl["CAM_PEDESTRIAN"]
    ret.stockFcw = (self.cam_empty_seen and cam_empty["STATUS"] != 0x7F) or ped["PED_WARNING"] == 1 or ped["BRAKE_WARNING"] == 1

    if self.CP.openpilotLongitudinalControl:
      acc_armed = cp.vl["PEDALS"]["ACC_OFF"] == 1
      acc_active = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      brake_free = not ret.brakePressed and not self.brake_pressed_prev
      if cp.vl["CRZ_BTNS"]["CAN_OFF"] == 1:
        self.cancel_context_frames = CANCEL_CONTEXT_FRAMES
      elif self.cancel_context_frames > 0:
        self.cancel_context_frames -= 1
      if acc_armed or acc_active:
        self.cruise_available = True
      elif brake_free or self.cancel_context_frames > 0:
        self.cruise_available = False
      if acc_armed or acc_active or self.cruise_enabled or brake_free:
        self.cruise_enabled = acc_active

      if len(cp.vl_all["CRZ_INFO"]["CTR1"]) > 0:
        self.stock_radar_silent_frames = 0
      else:
        self.stock_radar_silent_frames += 1

      resp = cp.vl_all["RADAR_UDS_RESPONSE"]
      self.radar_session_refused = any(
        sid == 0x7F and sub == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL and nrc != 0x78
        for sid, sub, nrc in zip(resp["SID"], resp["SUB"], resp["NRC"], strict=True))
      silenced = self.stock_radar_silent_frames >= STOCK_RADAR_GUARD_FRAMES
      ret.accFaulted = self.radar_was_silenced and not silenced
      self.radar_was_silenced |= silenced

      if not self.radar_was_silenced:
        self.cruise_enabled_blocked = True
      elif not self.cruise_enabled:
        self.cruise_enabled_blocked = False

      ret.cruiseState.available = self.cruise_available and self.radar_was_silenced
      ret.cruiseState.enabled = self.cruise_enabled and not self.cruise_enabled_blocked

      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      settled = cam_laneinfo_fresh and not (laneinfo["NO_ERR_BIT"] or laneinfo["ERR_BIT"])
      self.fsc_settled_frames = self.fsc_settled_frames + 1 if settled else 0
    else:
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    self.brake_pressed_prev = ret.brakePressed
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1 and not self.CP.openpilotLongitudinalControl
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS
    ret.invalidLkasSetting = not ti_enabled and cam_laneinfo_fresh and cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    if ti_enabled:
      ret.steerFaultTemporary = not self.ti_lkas_allowed and ret.vEgo > 10
    elif self.CP.minSteerSpeed > 0:
      ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked
    else:
      ret.steerFaultTemporary = False

    self.acc_active_last = ret.cruiseState.enabled
    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = not ti_enabled and cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    prev_cancel_button = self.cancel_button
    prev_resume_button = self.resume_button
    prev_main_button = self.main_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    self.accel_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]
    self.cancel_button = cp.vl["CRZ_BTNS"]["CAN_OFF"]
    self.resume_button = cp.vl["CRZ_BTNS"]["RES"]
    self.main_button = int(cp.vl["CRZ_BTNS"]["MODE_X"] == 1 and cp.vl["CRZ_BTNS"]["MODE_Y"] == 1)

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
      *create_button_events(self.cancel_button, prev_cancel_button, {1: ButtonType.cancel}),
      *create_button_events(self.resume_button, prev_resume_button, {1: ButtonType.resumeCruise}),
      *create_button_events(self.main_button, prev_main_button, {1: ButtonType.mainCruise}),
    ]

    CarStateExt.update(self, ret, ret_sp, can_parsers)
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = []
    if CP.openpilotLongitudinalControl:
      pt_messages.append(("CRZ_INFO", float("nan")))
      pt_messages.append(("RADAR_UDS_RESPONSE", float("nan")))
    cam_messages = [
      ("CAM_LANEINFO", float("nan")),
      ("CAM_TRAFFIC_SIGNS", float("nan")),
      ("CAM_EMPTY", float("nan")),
    ]
    parsers = {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
    if CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      parsers[Bus.body] = CANParser(DBC[CP.carFingerprint][Bus.pt], [("TI_FEEDBACK", 50)], 1)
    return parsers