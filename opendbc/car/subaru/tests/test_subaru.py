import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.subaru.carstate import HIGH_ANGLE_CUT_DEG, HIGH_ANGLE_GATE_SPEED_KPH, HIGH_ANGLE_HANDS_ON_TORQUE, HIGH_ANGLE_RESTORE_DEG
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import DBC, SubaruFlags


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
