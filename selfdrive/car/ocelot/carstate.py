from cereal import car
from common.numpy_fast import mean, int_rnd
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.ocelot.values import CAR, DBC, STEER_THRESHOLD, BUTTON_STATES
import cereal.messaging as messaging
from common.travis_checker import travis
from common.op_params import opParams

op_params = opParams()

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['chassis'])
    self.shifter_values = can_define.dv["GEAR_PACKET"]['GEAR']
    self.brakeUnavailable = True
    self.engineRPM = 0
    if not travis:
      self.pm = messaging.PubMaster(['liveTrafficData'])
      self.sm = messaging.SubMaster(['liveMapData'])
    self.buttonStates = BUTTON_STATES.copy()

  def update(self, cp, cp_body):
    ret = car.CarState.new_message()

    #Car specific information
    if self.CP.carFingerprint == CAR.SMART_ROADSTER_COUPE:
        ret.doorOpen = False #any([cp_body.vl["BODYCONTROL"]['RIGHT_DOOR'], cp_body.vl["BODYCONTROL"]['LEFT_DOOR']]) != 0
        ret.seatbeltUnlatched = False
        ret.leftBlinker = bool(cp_body.vl["BODYCONTROL"]['LEFT_SIGNAL'])
        ret.rightBlinker = bool(cp_body.vl["BODYCONTROL"]['RIGHT_SIGNAL'])
        ret.espDisabled = bool(cp_body.vl["ABS"]['ESP_STATUS'])
        ret.wheelSpeeds.fl = cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_FL'] * CV.MPH_TO_MS
        ret.wheelSpeeds.fr = cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_FR'] * CV.MPH_TO_MS
        ret.wheelSpeeds.rl = cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_RL'] * CV.MPH_TO_MS
        ret.wheelSpeeds.rr = cp_body.vl["SMARTROADSTERWHEELSPEEDS"]['WHEELSPEED_RR'] * CV.MPH_TO_MS
        can_gear = int(cp_body.vl["GEAR_PACKET"]['GEAR'])
        ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
        self.engineRPM = cp_body.vl["GEAR_PACKET"]["RPM"]

    #Ibooster data
    ret.brakePressed = bool(cp.vl["BRAKE_STATUS"]['DRIVER_BRAKE_APPLIED'])
    ret.brakeLights = bool(cp.vl["BRAKE_STATUS"]['BRAKE_APPLIED'])
    self.brakeUnavailable = not bool(cp.vl["BRAKE_STATUS"]['BRAKE_OK'])

    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['PED_GAS'] + cp.vl["GAS_SENSOR"]['PED_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15

    #calculate speed from wheel speeds
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.001

    #Toyota SAS
    ret.steeringAngle = -(cp.vl["TOYOTA_STEERING_ANGLE_SENSOR1"]['TOYOTA_STEER_ANGLE'] + cp.vl["TOYOTA_STEERING_ANGLE_SENSOR1"]['TOYOTA_STEER_FRACTION'])
    ret.steeringRate = -cp.vl["TOYOTA_STEERING_ANGLE_SENSOR1"]['TOYOTA_STEER_RATE']


    #Steering information from smart standin ECU
    ret.steeringTorque = cp.vl["STEERING_STATUS"]['STEERING_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEERING_STATUS"]['STEERING_TORQUE_EPS']
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerError = bool(cp.vl["STEERING_STATUS"]['STEERING_OK'] == 0)

    ret.cruiseState.enabled = bool(cp.vl["HIM_CTRLS"]['SET_BTN'])
    ret.cruiseState.available = True
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    #Logic for OP to manage whether it's enabled or not as controls board only sends button inputs


    self.buttonStates["accelCruise"] = bool(cp.vl["HIM_CTRLS"]['SPEEDUP_BTN'])
    self.buttonStates["decelCruise"] = bool(cp.vl["HIM_CTRLS"]['SPEEDDN_BTN'])
    self.buttonStates["cancel"] = bool(cp.vl["HIM_CTRLS"]['CANCEL_BTN'])
    self.buttonStates["setCruise"] = bool(cp.vl["HIM_CTRLS"]['SET_BTN'])

    if not travis:
      self.sm.update(0)
      self.smartspeed = self.sm['liveMapData'].speedLimit

    ret.stockAeb = False
    ret.leftBlindspot = False
    ret.rightBlindspot = False



    return ret



  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("TOYOTA_STEER_ANGLE", "TOYOTA_STEERING_ANGLE_SENSOR1", 0),
      ("BRAKE_APPLIED", "BRAKE_STATUS", 0),
      ("DRIVER_BRAKE_APPLIED", "BRAKE_STATUS", 0),
      ("BRAKE_OK", "BRAKE_STATUS", 0),
      ("BRAKE_PEDAL_POSITION", "BRAKE_STATUS", 0),
      ("TOYOTA_STEER_FRACTION", "TOYOTA_STEERING_ANGLE_SENSOR1", 0),
      ("TOYOTA_STEER_RATE", "TOYOTA_STEERING_ANGLE_SENSOR1", 0),
      ("SET_BTN", "HIM_CTRLS", 0),
      ("CANCEL_BTN", "HIM_CTRLS", 0),
      ("SPEEDUP_BTN", "HIM_CTRLS", 0),
      ("SPEEDDN_BTN", "HIM_CTRLS", 0),
      ("STEERING_TORQUE_DRIVER", "STEERING_STATUS", 0),
      ("STEERING_TORQUE_EPS", "STEERING_STATUS", 0),
      ("STEERING_OK", "STEERING_STATUS", 0),
    ]

    checks = [
      ("TOYOTA_STEERING_ANGLE_SENSOR1", 80),
      ("STEERING_STATUS", 80),
      ("BRAKE_STATUS", 80),
    ]

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("PED_GAS", "GAS_SENSOR", 0))
      signals.append(("PED_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))


    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)

  @staticmethod
  def get_body_can_parser(CP):

    signals = [
    ]

    # use steering message to check if panda is connected to frc
    checks = [
    ]

    if CP.carFingerprint == CAR.SMART_ROADSTER_COUPE:
        signals.append(("RIGHT_DOOR", "BODYCONTROL",0))
        signals.append(("LEFT_DOOR", "BODYCONTROL",0))
        signals.append(("LEFT_SIGNAL", "BODYCONTROL",0))
        signals.append(("RIGHT_SIGNAL", "BODYCONTROL",0))
        signals.append(("ESP_STATUS", "ABS",0))
        signals.append(("WHEELSPEED_FL", "SMARTROADSTERWHEELSPEEDS",0))
        signals.append(("WHEELSPEED_FR", "SMARTROADSTERWHEELSPEEDS",0))
        signals.append(("WHEELSPEED_RL", "SMARTROADSTERWHEELSPEEDS",0))
        signals.append(("WHEELSPEED_RR", "SMARTROADSTERWHEELSPEEDS",0))
        signals.append(("BRAKEPEDAL", "ABS",0))
        signals.append(("GEAR","GEAR_PACKET", 0))
        signals.append(("RPM","GEAR_PACKET",0))

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, 0)
