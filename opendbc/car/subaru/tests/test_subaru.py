import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.subaru.carstate import HIGH_ANGLE_CUT_DEG, HIGH_ANGLE_GATE_SPEED_KPH, HIGH_ANGLE_HANDS_ON_TORQUE, HIGH_ANGLE_RESTORE_DEG
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car import structs
from opendbc.car.subaru.values import DBC, LONG_TUNE, CarControllerParams, SubaruFlags, long_tune
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP
import numpy as np


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruHighAngleGuard(unittest.TestCase):
  def _interface(self, car):
    CarInterface = interfaces[car]
    fingerprints = dict.fromkeys(range(7), {})
    CP = CarInterface.get_params(car, fingerprints, [], alpha_long=False, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprints, [], alpha_long=False, is_release_sp=False, docs=False)
    ci, packer = CarInterface(CP, CP_SP), CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self._step(ci, packer, 0)  # the parser drops the first frame after construction
    return ci, packer

  def _step(self, ci, packer, angle, warning=0, torque=0, speed=0, eps_angle=None):
    eps_angle = angle if eps_angle is None else eps_angle
    frames = [
      packer.make_can_msg("Brake_Pressure_L_R", 0, {"Steering_Angle": angle}),
      packer.make_can_msg("Brake_Pedal", 0, {"Speed": speed}),
      packer.make_can_msg("Steering_Torque", 0, {"Steering_Angle": eps_angle, "Steer_Warning": warning, "Steer_Torque_Sensor": torque}),
    ]
    cs, _ = ci.update([(0, [CanData(*f) for f in frames])])
    return cs.steerFaultTemporary

  def test_follows_the_eps_generation(self):
    for car in ("SUBARU_FORESTER", "SUBARU_OUTBACK", "SUBARU_LEGACY", "SUBARU_IMPREZA_2020"):
      ci, packer = self._interface(car)
      assert ci.CP.flags & SubaruFlags.STEER_RATE_LIMITED, car
      assert self._step(ci, packer, HIGH_ANGLE_CUT_DEG + 30), car
    ci, packer = self._interface("SUBARU_IMPREZA")
    assert not self._step(ci, packer, HIGH_ANGLE_CUT_DEG + 30)

  def test_latch_with_hysteresis(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    # Steering_Angle is quantized to 0.0217 deg, keep test angles a degree clear of the thresholds
    cut, restore = HIGH_ANGLE_CUT_DEG, HIGH_ANGLE_RESTORE_DEG
    for angle in (0, 50, cut - 1):
      assert not self._step(ci, packer, angle), angle
    assert self._step(ci, packer, cut + 1)
    # inside the band the latch holds, in either direction
    for angle in (cut - 1, restore + 1, cut + 5, restore + 1):
      assert self._step(ci, packer, angle), angle
    assert not self._step(ci, packer, restore - 1)
    assert not self._step(ci, packer, cut - 1)
    # sign does not matter
    assert self._step(ci, packer, -(cut + 1))
    assert not self._step(ci, packer, -(restore - 1))

  def test_one_transition_per_excursion(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    profile = list(range(0, 130)) + list(range(130, -1, -1))
    flags = [self._step(ci, packer, a) for a in profile]
    edges = sum(1 for a, b in zip(flags, flags[1:]) if a != b)
    assert edges == 2, edges
    assert flags[profile.index(HIGH_ANGLE_CUT_DEG + 1)]
    assert not flags[-1]

  def test_hands_on_defers_cut(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    cut, restore, hands_on = HIGH_ANGLE_CUT_DEG, HIGH_ANGLE_RESTORE_DEG, HIGH_ANGLE_HANDS_ON_TORQUE + 10
    for angle in (cut + 1, cut + 40, cut + 80, -(cut + 80)):
      assert not self._step(ci, packer, angle, torque=hands_on), angle
    assert not self._step(ci, packer, cut + 80, torque=-hands_on)
    # steered through by hand the whole way: no cut, whatever the grip does once back under the band
    assert not self._step(ci, packer, restore - 1, torque=0)
    # grip relaxes at the apex: cut, held through a re-grip until the wheel unwinds
    assert not self._step(ci, packer, cut + 80, torque=hands_on)
    assert self._step(ci, packer, cut + 80, torque=HIGH_ANGLE_HANDS_ON_TORQUE - 10)
    assert self._step(ci, packer, cut + 80, torque=hands_on)
    assert self._step(ci, packer, restore + 1, torque=hands_on)
    assert not self._step(ci, packer, restore - 1, torque=hands_on)

  def test_reads_the_vdc_angle(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    cut = HIGH_ANGLE_CUT_DEG
    assert not self._step(ci, packer, cut - 5, eps_angle=cut + 20)
    assert self._step(ci, packer, cut + 5, eps_angle=cut - 20)

  def test_only_below_the_gate_speed(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    cut, fast = HIGH_ANGLE_CUT_DEG, HIGH_ANGLE_GATE_SPEED_KPH + 5
    for angle in (cut + 1, cut + 80):
      assert not self._step(ci, packer, angle, speed=fast), angle
    # a latch taken at parking speed releases as soon as the module's gate disarms
    assert self._step(ci, packer, cut + 40, speed=HIGH_ANGLE_GATE_SPEED_KPH - 5)
    assert not self._step(ci, packer, cut + 40, speed=fast)

  def test_real_warning_still_reported(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    assert self._step(ci, packer, 0, warning=1)
    assert not self._step(ci, packer, 0, warning=0)


class TestSubaruStockAeb(unittest.TestCase):
  def _interface(self):
    car = "SUBARU_FORESTER"
    CarInterface = interfaces[car]
    fingerprints = dict.fromkeys(range(7), {})
    CP = CarInterface.get_params(car, fingerprints, [], alpha_long=True, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprints, [], alpha_long=True, is_release_sp=False, docs=False)
    return CarInterface(CP, CP_SP), CANPacker(DBC[CP.carFingerprint][Bus.pt])

  def _drive(self, ci, packer, aeb_status, pressure, pcb_off):
    frames = [CanData(*packer.make_can_msg("ES_Brake", 2, {"AEB_Status": aeb_status, "Brake_Pressure": pressure})),
              CanData(*packer.make_can_msg("ES_DashStatus", 2, {"PCB_Off": pcb_off}))]
    cs, _ = ci.update([(0, frames)])
    ci.CS.out.vEgo = 25.0
    CC = structs.CarControl(enabled=True, longActive=True).as_reader()
    thr = brake_active = dash_pcb_off = None
    for _ in range(10):
      _, sends = ci.apply(CC, structs.CarControlSP(), 0)
      for addr, dat, _ in sends:
        if addr == 0x221:
          thr = int.from_bytes(dat[2:4], "little") & 0x1FFF
          brake_active = (dat[4] >> 4) & 1
        if addr == 0x321:
          dash_pcb_off = (dat[1] >> 4) & 1
    return cs.stockAeb, thr, brake_active, dash_pcb_off

  def test_stock_aeb_stops_the_drive_and_keeps_the_dash_honest(self):
    """The controller's latch follows the panda's: the camera claiming AEB with pressure takes the
    drive away until its event has ended and it asks no more brake than openpilot does."""
    ci, packer = self._interface()
    self._drive(ci, packer, 0, 0, 0)  # the parser drops the first frame after construction
    hold = int(round(np.interp(25.0, long_tune("SUBARU_FORESTER")["THROTTLE_HOLD_BP"], long_tune("SUBARU_FORESTER")["THROTTLE_HOLD_V"])))
    assert self._drive(ci, packer, 0, 0, 0) == (False, hold, 0, 0)
    assert self._drive(ci, packer, 8, 0, 0) == (False, hold, 0, 0)
    for aeb_status in (8, 4, 12):
      assert self._drive(ci, packer, aeb_status, 300, 0) == (True, 808, 1, 0), aeb_status
      assert self._drive(ci, packer, 0, 150, 0) == (False, 808, 1, 0), aeb_status
      assert self._drive(ci, packer, 0, 0, 0) == (False, hold, 0, 0), aeb_status
    assert self._drive(ci, packer, 8, 300, 1) == (True, 808, 1, 1)
    assert self._drive(ci, packer, 0, 0, 0) == (False, hold, 0, 0)


class TestSubaruStopAndGoUnderLong(unittest.TestCase):
  def _sends(self, alpha_long):
    car = "SUBARU_FORESTER"
    CarInterface = interfaces[car]
    fingerprints = dict.fromkeys(range(7), {})
    CP = CarInterface.get_params(car, fingerprints, [], alpha_long=alpha_long, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprints, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
    CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO.value
    ci = CarInterface(CP, CP_SP)
    ci.update([(0, [])])
    _, sends = ci.apply(structs.CarControl(enabled=True).as_reader(), structs.CarControlSP(), 0)
    return {(addr, bus) for addr, _, bus in sends}

  def test_pedal_spoofs_only_under_stock_long(self):
    assert (0x40, 2) in self._sends(alpha_long=False)
    es = {0x122, 0x321, 0x322, 0x222, 0x220, 0x221}
    assert self._sends(alpha_long=True) == {(addr, 0) for addr in es}


class TestSubaruCameraCopies(unittest.TestCase):
  def _interface(self):
    car = "SUBARU_FORESTER"
    CarInterface = interfaces[car]
    fingerprints = dict.fromkeys(range(7), {})
    CP = CarInterface.get_params(car, fingerprints, [], alpha_long=True, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprints, [], alpha_long=True, is_release_sp=False, docs=False)
    CP_SP.flags |= SubaruFlagsSP.CAMERA_ECHO.value
    return CarInterface(CP, CP_SP), CANPacker(DBC[CP.carFingerprint][Bus.pt])

  def _copies(self, ci, frames, addr, n, accel=0.0, v_ego=10.0):
    CC = structs.CarControl(enabled=True, longActive=True)
    CC.actuators.accel = accel
    CC = CC.as_reader()
    ci.update([(0, frames)])  # the parser drops the first frame after construction
    ci.apply(CC, structs.CarControlSP(), 0)
    out = []
    for _ in range(n):
      ci.update([(0, frames)])
      ci.CS.out.vEgo = v_ego
      ci.CS.out.standstill = v_ego == 0.0
      _, sends = ci.apply(CC, structs.CarControlSP(), 0)
      out += [dat for a, dat, bus in sends if (a, bus) == (addr, 2)]
    return out

  def test_brake_status_copy_says_what_the_camera_commanded_and_keeps_the_pedal(self):
    ci, packer = self._interface()
    for cam_active, pedal in ((1, 0), (0, 1), (1, 1)):
      frames = [CanData(*packer.make_can_msg("ES_Brake", 2, {"Cruise_Brake_Active": cam_active, "Brake_Pressure": 100 * cam_active})),
                CanData(*packer.make_can_msg("Brake_Status", 0, {"ES_Brake": 1 - cam_active, "Brake": pedal}))]
      copies = self._copies(ci, frames, 0x13c, 4)
      assert len(copies) == 2
      for d in copies:
        assert (d[7] >> 2) & 1 == cam_active, (cam_active, pedal)  # ES_Brake, bit 58
        assert (d[7] >> 6) & 1 == pedal, (cam_active, pedal)       # Brake, bit 62

  def test_throttle_copy_maps_the_camera_command_and_keeps_the_pedal(self):
    ci, packer = self._interface()
    for cam_throttle, expected, pedal in ((808, 0, 0), (2600, 28, 0), (3200, 56, 40), (4100, 87, 0)):
      frames = [CanData(*packer.make_can_msg("ES_Distance", 2, {"Cruise_Throttle": cam_throttle})),
                CanData(*packer.make_can_msg("Throttle", 0, {"Throttle_Pedal": pedal, "Throttle_Cruise": 99, "Throttle_Combo": 77, "Engine_RPM": 1500}))]
      copies = self._copies(ci, frames, 0x40, 3)
      assert len(copies) == 3
      for d in copies:
        assert d[4] == pedal, (cam_throttle, pedal)
        assert d[5] == expected, (cam_throttle, pedal)
        assert d[6] == (expected if pedal == 0 else 77), (cam_throttle, pedal)
        assert int.from_bytes(d[2:4], "little") & 0x1FFF == 1500

  def test_pulling_away_under_the_camera_hold_taps_the_gas_once(self):
    ci, packer = self._interface()
    frames = [CanData(*packer.make_can_msg("ES_Brake", 2, {"Cruise_Brake_Active": 1, "Brake_Pressure": 297})),
              CanData(*packer.make_can_msg("Throttle", 0, {"Throttle_Pedal": 0}))]
    pedals = [d[4] for d in self._copies(ci, frames, 0x40, 40, accel=0.8, v_ego=0.0)]
    assert pedals == [5] * 15 + [0] * 25
    # and not while the camera is not holding, or the car is moving
    frames_moving = frames
    assert all(p == 0 for p in [d[4] for d in self._copies(ci, frames_moving, 0x40, 5, accel=0.8, v_ego=3.0)])


class TestSubaruDisengageBeep(unittest.TestCase):
  def _alert(self, alpha_long):
    car = "SUBARU_FORESTER"
    CarInterface = interfaces[car]
    fingerprints = dict.fromkeys(range(7), {})
    CP = CarInterface.get_params(car, fingerprints, [], alpha_long=alpha_long, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprints, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
    ci, packer = CarInterface(CP, CP_SP), CANPacker(DBC[CP.carFingerprint][Bus.pt])
    frames = [CanData(*packer.make_can_msg("ES_LKAS_State", 2, {"LKAS_Alert": 26}))]
    ci.update([(0, frames)])
    out = []
    for _ in range(20):
      ci.update([(0, frames)])
      _, sends = ci.apply(structs.CarControl(enabled=False).as_reader(), structs.CarControlSP(), 0)
      out += [dat[4] & 0x1F for addr, dat, bus in sends if (addr, bus) == (0x322, 0)]
    return out

  def test_acc_disengaged_beep_is_filtered_only_under_openpilot_long(self):
    assert set(self._alert(alpha_long=False)) == {26}
    assert set(self._alert(alpha_long=True)) == {0}


class TestSubaruLongHold(unittest.TestCase):
  def _interface(self, car, alpha_long):
    CarInterface = interfaces[car]
    fingerprints = dict.fromkeys(range(7), {})
    CP = CarInterface.get_params(car, fingerprints, [], alpha_long=alpha_long, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, car, fingerprints, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
    return CarInterface(CP, CP_SP)

  FORESTER_MEASURED = {"THROTTLE_HOLD_BP", "THROTTLE_HOLD_V", "RPM_HOLD_BP", "RPM_HOLD_V",
                       "THROTTLE_GAIN_BP", "THROTTLE_GAIN_V", "THROTTLE_DECEL_GAIN",
                       "RPM_GAIN_UP_BP", "RPM_GAIN_UP_V", "RPM_GAIN_DOWN", "THR_DECEL_V"}

  def test_forester_has_its_own_measured_tables(self):
    forester, crosstrek = long_tune("SUBARU_FORESTER"), long_tune("SUBARU_IMPREZA_2020")
    for key in self.FORESTER_MEASURED:
      assert forester[key] != crosstrek[key], key
    for key in set(crosstrek) - self.FORESTER_MEASURED:
      assert forester[key] == crosstrek[key], key
    assert long_tune("SUBARU_IMPREZA") == crosstrek
    assert long_tune("SUBARU_ASCENT") == crosstrek

  def test_pid_accel_limits_take_controlsd_arguments(self):
    # controlsd calls get_pid_accel_limits(CP, CP_SP, v_ego, v_cruise); a commaai-shaped signature
    # crashes it on every frame and the car loses every ES message
    for car in LONG_TUNE:
      ci = self._interface(car, True)
      assert ci.get_pid_accel_limits(ci.CP, ci.CP_SP, 10.0, 20.0) == (CarControllerParams.ACCEL_MIN, long_tune(car)["ACCEL_MAX"]), car

  def test_zero_accel_commands_the_hold_point(self):
    v_ego = 25.0
    for car in LONG_TUNE:
      ci = self._interface(car, True)
      assert ci.CP.openpilotLongitudinalControl, car
      ci.update([(0, [])])
      ci.CS.out.vEgo = v_ego
      CC = structs.CarControl(enabled=True, longActive=True).as_reader()
      CC_SP = structs.CarControlSP()
      thr = rpm = None
      for _ in range(20):
        _, sends = ci.apply(CC, CC_SP, 0)
        for addr, dat, _ in sends:
          if addr == 0x221:
            thr = int.from_bytes(dat[2:4], "little") & 0x1FFF
          if addr == 0x222:
            rpm = int.from_bytes(dat[2:4], "little") & 0x1FFF
      tune = long_tune(car)
      assert thr == int(round(np.interp(v_ego, tune["THROTTLE_HOLD_BP"], tune["THROTTLE_HOLD_V"]))), (car, thr)
      assert rpm == int(round(np.interp(v_ego, tune["RPM_HOLD_BP"], tune["RPM_HOLD_V"]))), (car, rpm)
