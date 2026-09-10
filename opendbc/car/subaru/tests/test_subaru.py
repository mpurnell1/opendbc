import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.subaru.carstate import HIGH_ANGLE_CUT_DEG, HIGH_ANGLE_HANDS_ON_TORQUE, HIGH_ANGLE_RESTORE_DEG
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

  def _step(self, ci, packer, angle, warning=0, torque=0):
    values = {"Steering_Angle": angle, "Steer_Warning": warning, "Steer_Torque_Sensor": torque}
    frame = CanData(*packer.make_can_msg("Steering_Torque", 0, values))
    cs, _ = ci.update([(0, [frame])])
    return cs.steerFaultTemporary

  def test_forester_has_flag(self):
    ci, _ = self._interface("SUBARU_FORESTER")
    assert ci.CP.flags & SubaruFlags.HIGH_ANGLE_FAULT

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

  def test_real_warning_still_reported(self):
    ci, packer = self._interface("SUBARU_FORESTER")
    assert self._step(ci, packer, 0, warning=1)
    assert not self._step(ci, packer, 0, warning=0)
