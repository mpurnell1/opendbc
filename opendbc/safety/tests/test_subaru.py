#!/usr/bin/env python3
import enum
import unittest

import numpy as np

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from functools import partial


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Brake          = 0x220
  ES_Distance       = 0x221
  ES_Status         = 0x222
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323
  ES_UDS_Request    = 0x787
  ES_HighBeamAssist = 0x22A
  ES_STATIC_1       = 0x325
  ES_STATIC_2       = 0x121


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS):
  return [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]


def long_tx_msgs(alt_bus):
  return [[SubaruMsg.ES_Brake,          alt_bus],
          [SubaruMsg.ES_Status,         alt_bus]]


def gen2_long_additional_tx_msgs():
  return [[SubaruMsg.ES_UDS_Request,    SUBARU_CAM_BUS],
          [SubaruMsg.ES_HighBeamAssist, SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_1,       SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_2,       SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS):
  return {SUBARU_CAM_BUS: [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}


class TestSubaruSafetyBase(common.CarSafetyTest):
  FLAGS = 0
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS
  ALT_CAM_BUS = SUBARU_CAM_BUS

  DEG_TO_CAN = 100

  INACTIVE_GAS = 1818

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruLongitudinalSafetyBase(TestSubaruSafetyBase, common.LongitudinalGasBrakeSafetyTest):
  MIN_GAS = 808
  INACTIVE_GAS = 1818
  MAX_POSSIBLE_GAS = 2**13

  # Mirrors SUBARU_MAX_GAS_LOOKUP in opendbc/safety/modes/subaru.h. The gas ceiling is speed
  # dependent, so there is no single MAX_GAS: the generic limit test gets whatever the curve allows
  # at the speed that test runs at, which is a standstill.
  MAX_GAS_BP = [0., 15., 29.]
  MAX_GAS_V = [2618., 3514., 4250.]
  ABSOLUTE_MAX_GAS = 4100

  MIN_BRAKE = 0
  MAX_BRAKE = 600
  MAX_POSSIBLE_BRAKE = 2**16

  MIN_RPM = 0
  MAX_RPM = 3600
  MAX_POSSIBLE_RPM = 2**13

  FWD_BLACKLISTED_ADDRS = {2: [SubaruMsg.ES_LKAS, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance,
                               SubaruMsg.ES_Status, SubaruMsg.ES_DashStatus,
                               SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}

  @classmethod
  def _max_gas_at(cls, speed):
    """What the safety model should allow at `speed` m/s, including its 1 m/s upward fudge."""
    y = np.interp(speed + 1., cls.MAX_GAS_BP, cls.MAX_GAS_V)
    return min(int(y), cls.ABSOLUTE_MAX_GAS)

  def setUp(self):
    super().setUp()
    # The generic gas test sweeps a flat range, so pin it to the ceiling at the speed it runs at.
    self.MAX_GAS = self._max_gas_at(0.)

  def _set_speed(self, speed):
    # vehicle_speed keeps the last 6 samples and the hook reads .max, so flush the whole window
    for _ in range(6):
      self.assertTrue(self._rx(self._speed_msg(speed * CV.MS_TO_KPH)))

  def test_gas_ceiling_follows_speed(self):
    """The ceiling tracks the measured 2.0 m/s^2 envelope rather than a flat count."""
    self.safety.set_controls_allowed(True)
    for speed in (0., 5., 10., 15., 20., 25., 29., 35.):
      with self.subTest(speed=speed):
        self._set_speed(speed)
        limit = self._max_gas_at(speed)
        self.assertTrue(self._tx(self._send_gas_msg(limit)), f"{limit} should be allowed at {speed} m/s")
        self.assertFalse(self._tx(self._send_gas_msg(limit + 1)), f"{limit + 1} should be refused at {speed} m/s")

  def test_gas_ceiling_never_exceeds_absolute(self):
    """However fast the car is going, 4100 is the end of it."""
    self.safety.set_controls_allowed(True)
    self._set_speed(60.)                       # far beyond the last breakpoint
    self.assertTrue(self._tx(self._send_gas_msg(self.ABSOLUTE_MAX_GAS)))
    self.assertFalse(self._tx(self._send_gas_msg(self.ABSOLUTE_MAX_GAS + 1)))

  def test_gas_ceiling_is_tightest_at_a_standstill(self):
    """The curve binds hardest from rest, where a given count buys the most acceleration: 3400
    counts is inside the absolute ceiling but well above what 2.0 m/s^2 needs at 0 m/s."""
    self.safety.set_controls_allowed(True)
    self._set_speed(0.)
    self.assertFalse(self._tx(self._send_gas_msg(3400)))

  def test_rpm_safety_check(self):
    self._generic_limit_safety_check(self._send_rpm_msg, self.MIN_RPM, self.MAX_RPM, 0, self.MAX_POSSIBLE_RPM, 1)

  def _send_brake_msg(self, brake, aeb_status=0):
    values = {"Brake_Pressure": brake, "AEB_Status": aeb_status}
    return self.packer.make_can_msg_safety("ES_Brake", self.ALT_MAIN_BUS, values)

  def _send_gas_msg(self, gas):
    values = {"Cruise_Throttle": gas}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def _send_rpm_msg(self, rpm):
    values = {"Cruise_RPM": rpm}
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, values)

  def _cam_brake_msg(self, aeb_status, brake):
    # its own packer: the camera's counter sequence is independent of openpilot's ES_Brake
    if not hasattr(self, "cam_packer"):
      self.cam_packer = CANPackerSafety("subaru_global_2017_generated")
    values = {"AEB_Status": aeb_status, "Brake_Pressure": brake}
    return self.cam_packer.make_can_msg_safety("ES_Brake", SUBARU_CAM_BUS, values)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = 0
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE_LOOKUP = [0], [1000]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


class TestSubaruGen1LongitudinalSafety(TestSubaruLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.LONG
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS) + long_tx_msgs(SUBARU_MAIN_BUS)

  def _cam_brake_forwarded(self):
    return self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake) == SUBARU_MAIN_BUS

  def _cam_status_forwarded(self):
    return self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Status) == SUBARU_MAIN_BUS

  def _cam_distance_forwarded(self):
    return self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Distance) == SUBARU_MAIN_BUS

  def test_stock_aeb_passthrough(self):
    """The camera's ES_Distance, ES_Brake and ES_Status are forwarded and openpilot's refused, from the
    camera claiming AEB with at least openpilot's brake until its event has ended and it asks no more
    than openpilot."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._send_brake_msg(100)))

    self.assertTrue(self._rx(self._cam_brake_msg(0, 0)))
    self.assertFalse(self._cam_brake_forwarded())
    self.assertFalse(self._cam_status_forwarded())
    self.assertFalse(self._cam_distance_forwarded())
    self.assertTrue(self._tx(self._send_brake_msg(100)))
    self.assertTrue(self._tx(self._send_rpm_msg(1000)))
    self.assertTrue(self._tx(self._send_gas_msg(self.INACTIVE_GAS + 1)))

    # a weaker stock request never replaces a stronger one of ours
    self.assertTrue(self._rx(self._cam_brake_msg(8, 50)))
    self.assertFalse(self._cam_brake_forwarded())
    self.assertTrue(self._tx(self._send_brake_msg(100)))

    # nor does a status without pressure, even against no brake of ours
    self.assertTrue(self._tx(self._send_brake_msg(0)))
    self.assertTrue(self._rx(self._cam_brake_msg(8, 0)))
    self.assertFalse(self._cam_brake_forwarded())
    self.assertTrue(self._tx(self._send_brake_msg(100)))

    for aeb_status in (8, 4, 12):
      with self.subTest(aeb_status=aeb_status):
        self.assertTrue(self._rx(self._cam_brake_msg(aeb_status, 300)))
        self.assertTrue(self._cam_brake_forwarded())
        self.assertTrue(self._cam_status_forwarded())
        self.assertTrue(self._cam_distance_forwarded())
        self.assertFalse(self._tx(self._send_brake_msg(100)))
        self.assertFalse(self._tx(self._send_rpm_msg(0)))
        self.assertFalse(self._tx(self._send_gas_msg(self.INACTIVE_GAS)))
        # latched: the camera easing off mid-event does not hand the brake back
        self.assertTrue(self._rx(self._cam_brake_msg(aeb_status, 10)))
        self.assertTrue(self._cam_brake_forwarded())
        self.assertFalse(self._tx(self._send_brake_msg(100)))
        # nor does the event ending while the camera still asks for more than we do
        self.assertTrue(self._rx(self._cam_brake_msg(0, 150)))
        self.assertTrue(self._cam_brake_forwarded())
        self.assertFalse(self._tx(self._send_brake_msg(100)))
        self.assertTrue(self._rx(self._cam_brake_msg(0, 100)))
        self.assertFalse(self._cam_brake_forwarded())
        self.assertFalse(self._cam_status_forwarded())
        self.assertFalse(self._cam_distance_forwarded())
        self.assertTrue(self._tx(self._send_brake_msg(100)))
        self.assertTrue(self._tx(self._send_rpm_msg(1000)))
        self.assertTrue(self._tx(self._send_gas_msg(self.INACTIVE_GAS + 1)))

  def _cam_warning_msg(self, lkas_alert=0, lkas_alert_msg=0):
    values = {"LKAS_Alert": lkas_alert, "LKAS_Alert_Msg": lkas_alert_msg}
    return self.packer.make_can_msg_safety("ES_LKAS_State", SUBARU_CAM_BUS, values)

  def test_collision_warning_forwards_the_precharge(self):
    """The camera brakes during its warning stage with AEB_Status still 0, and the brake module acts
    on it, so the warning opens the latch on the same terms as the status."""
    self.safety.set_controls_allowed(True)
    for alert, alert_msg in ((1, 0), (2, 0), (5, 0), (0, 6)):
      with self.subTest(alert=alert, alert_msg=alert_msg):
        self.assertTrue(self._tx(self._send_brake_msg(0)))
        self.assertTrue(self._rx(self._cam_warning_msg()))
        self.assertTrue(self._rx(self._cam_brake_msg(0, 100)))
        self.assertFalse(self._cam_brake_forwarded())

        self.assertTrue(self._rx(self._cam_warning_msg(alert, alert_msg)))
        self.assertTrue(self._rx(self._cam_brake_msg(0, 100)))
        self.assertTrue(self._cam_brake_forwarded())
        self.assertFalse(self._tx(self._send_brake_msg(100)))

        # the warning ending releases it only once the camera asks no more than openpilot
        self.assertTrue(self._rx(self._cam_warning_msg()))
        self.assertTrue(self._rx(self._cam_brake_msg(0, 100)))
        self.assertFalse(self._cam_brake_forwarded())
        self.assertTrue(self._tx(self._send_brake_msg(0)))

  def test_ordinary_camera_braking_is_not_forwarded(self):
    # the camera's own ACC braking carries no warning, whatever it asks for
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._cam_warning_msg()))
    for brake in (100, 346, 579):
      self.assertTrue(self._rx(self._cam_brake_msg(0, brake)))
      self.assertFalse(self._cam_brake_forwarded(), brake)

  def test_no_aeb_claim(self):
    # only the camera claims AEB
    self.safety.set_controls_allowed(True)
    for aeb_status in (4, 8, 12):
      self.assertFalse(self._tx(self._send_brake_msg(100, aeb_status)))
    self.assertTrue(self._tx(self._send_brake_msg(100, 0)))

  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Status,
                                               SubaruMsg.ES_Distance)}


class TestSubaruGen1LongitudinalCameraEchoSafety(TestSubaruGen1LongitudinalSafety):
  """Gen1 long feeding the camera the response its own command expects: openpilot's copies of
  Brake_Status and Throttle replace the car's on the camera bus, free to say anything about ES braking
  and cruise throttle, honest about the driver's pedals, except the standstill gas tap."""
  FLAGS = SubaruSafetyFlags.LONG | SubaruSafetyFlags.CAMERA_ECHO
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS) + long_tx_msgs(SUBARU_MAIN_BUS) + \
            [[SubaruMsg.Brake_Status, SUBARU_CAM_BUS], [SubaruMsg.Throttle, SUBARU_CAM_BUS]]
  FWD_BLACKLISTED_ADDRS = {2: TestSubaruLongitudinalSafetyBase.FWD_BLACKLISTED_ADDRS[2], 0: [SubaruMsg.Brake_Status, SubaruMsg.Throttle]}
  RELAY_MALFUNCTION_ADDRS = {**TestSubaruGen1LongitudinalSafety.RELAY_MALFUNCTION_ADDRS,
                             SUBARU_CAM_BUS: (SubaruMsg.Brake_Status, SubaruMsg.Throttle)}

  def _cam_brake_status_msg(self, es_brake, brake):
    values = {"ES_Brake": es_brake, "Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", SUBARU_CAM_BUS, values)

  def _cam_throttle_msg(self, pedal, cruise):
    values = {"Throttle_Pedal": pedal, "Throttle_Cruise": cruise}
    return self.packer.make_can_msg_safety("Throttle", SUBARU_CAM_BUS, values)

  def test_brake_status_copy_keeps_the_pedal_honest(self):
    for pedal in (0, 1):
      for _ in range(10):
        self._rx(self._user_brake_msg(pedal))
      for es_brake in (0, 1):
        self.assertTrue(self._tx(self._cam_brake_status_msg(es_brake, pedal)))
        self.assertFalse(self._tx(self._cam_brake_status_msg(es_brake, 1 - pedal)))

  def test_throttle_copy_keeps_the_pedal_honest(self):
    for pedal in (0, 30, 5):
      for _ in range(10):
        self._rx(self._user_gas_msg(pedal))
      for cruise in (0, 45, 87):
        self.assertTrue(self._tx(self._cam_throttle_msg(pedal, cruise)))
        self.assertFalse(self._tx(self._cam_throttle_msg(pedal + 1, cruise)))

  def test_copies_may_lag_the_car_by_100ms(self):
    # a pedal moving fast leaves the copy a frame or two behind; a refused copy is one the camera
    # never gets, and it faults on the gap
    ramp = [0, 4, 11, 17, 19, 22, 30, 41, 55, 60]
    for pedal in ramp:
      self._rx(self._user_gas_msg(pedal))
    for pedal in ramp:
      self.assertTrue(self._tx(self._cam_throttle_msg(pedal, 45)), pedal)
    self.assertFalse(self._tx(self._cam_throttle_msg(61, 45)))
    # and no further back than that
    for pedal in range(70, 80):
      self._rx(self._user_gas_msg(pedal))
    for pedal in ramp:
      self.assertFalse(self._tx(self._cam_throttle_msg(pedal, 45)), pedal)

  def test_throttle_copy_gas_tap_only_below_walking_pace_while_engaged(self):
    self._rx(self._user_gas_msg(0))
    for controls_allowed in (False, True):
      for v_ms in (0.0, 1.0, 5.0):
        self.safety.set_controls_allowed(controls_allowed)
        self._reset_speed_measurement(v_ms * CV.MS_TO_KPH)
        self.assertEqual(controls_allowed and v_ms < 2., self._tx(self._cam_throttle_msg(5, 0)), (controls_allowed, v_ms))


class TestSubaruGen2LongitudinalSafety(TestSubaruLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS) + long_tx_msgs(SUBARU_ALT_BUS) + gen2_long_additional_tx_msgs()
  FWD_BLACKLISTED_ADDRS = {2: [SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                               SubaruMsg.ES_Infotainment]}
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment),
                             SUBARU_ALT_BUS: (SubaruMsg.ES_Brake, SubaruMsg.ES_Status, SubaruMsg.ES_Distance)}

  def _rdbi_msg(self, did: int):
    return b'\x03\x22' + did.to_bytes(2) + b'\x00\x00\x00\x00'

  def _es_uds_msg(self, msg: bytes):
    return libsafety_py.make_CANPacket(SubaruMsg.ES_UDS_Request, 2, msg)

  def test_es_uds_message(self):
    tester_present = b'\x02\x3E\x80\x00\x00\x00\x00\x00'
    not_tester_present = b"\x03\xAA\xAA\x00\x00\x00\x00\x00"

    button_did = 0x1130

    # Tester present is allowed for gen2 long to keep eyesight disabled
    self.assertTrue(self._tx(self._es_uds_msg(tester_present)))

    # Non-Tester present is not allowed
    self.assertFalse(self._tx(self._es_uds_msg(not_tester_present)))

    # Only button_did is allowed to be read via UDS
    for did in range(0xFFFF):
      should_tx = (did == button_did)
      self.assertEqual(self._tx(self._es_uds_msg(self._rdbi_msg(did))), should_tx)

    # any other msg is not allowed
    for sid in range(0xFF):
      msg = b'\x03' + sid.to_bytes(1) + b'\x00' * 6
      self.assertFalse(self._tx(self._es_uds_msg(msg)))


if __name__ == "__main__":
  unittest.main()
