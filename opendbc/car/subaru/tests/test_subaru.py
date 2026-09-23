import unittest

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import DBC, SubaruFlags, long_tune


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


def _forester(alpha_long=True):
  car = "SUBARU_FORESTER"
  CarInterface = interfaces[car]
  fingerprints = dict.fromkeys(range(7), {})
  CP = CarInterface.get_params(car, fingerprints, [], alpha_long=alpha_long, is_release=False, docs=False)
  return CarInterface(CP), CANPacker(DBC[CP.carFingerprint][Bus.pt])


class TestSubaruStockAeb(unittest.TestCase):
  def _drive(self, ci, packer, aeb_status, pressure, pcb_off, lkas_alert=0):
    frames = [CanData(*packer.make_can_msg("ES_Brake", 2, {"AEB_Status": aeb_status, "Brake_Pressure": pressure})),
              CanData(*packer.make_can_msg("ES_LKAS_State", 2, {"LKAS_Alert": lkas_alert})),
              CanData(*packer.make_can_msg("ES_DashStatus", 2, {"PCB_Off": pcb_off}))]
    cs = ci.update([(0, frames)])
    ci.CS.out.vEgo = 25.0
    CC = structs.CarControl(enabled=True, longActive=True).as_reader()
    thr = brake_active = dash_pcb_off = None
    for _ in range(10):
      _, sends = ci.apply(CC, 0)
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
    ci, packer = _forester()
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

  def test_the_collision_warning_takes_the_drive_away_too(self):
    # the camera brakes during its warning stage with AEB_Status still 0, and the panda forwards it
    ci, packer = _forester()
    self._drive(ci, packer, 0, 0, 0)
    hold = int(round(np.interp(25.0, long_tune("SUBARU_FORESTER")["THROTTLE_HOLD_BP"], long_tune("SUBARU_FORESTER")["THROTTLE_HOLD_V"])))
    for alert in (1, 2):
      assert self._drive(ci, packer, 0, 100, 0, lkas_alert=alert)[1:] == (808, 1, 0), alert
      assert self._drive(ci, packer, 0, 0, 0) == (False, hold, 0, 0), alert
    # the camera's own ACC braking is not one
    assert self._drive(ci, packer, 0, 346, 0)[1:] == (hold, 0, 0)


class TestSubaruCameraCopies(unittest.TestCase):
  def test_the_echo_comes_with_gen1_openpilot_longitudinal(self):
    assert _forester(alpha_long=True)[0].CP.flags & SubaruFlags.CAMERA_ECHO
    assert not _forester(alpha_long=False)[0].CP.flags & SubaruFlags.CAMERA_ECHO

  def _copies(self, ci, frames, addr, n, accel=0.0, v_ego=10.0, brake_last=None, throttle_last=None):
    CC = structs.CarControl(enabled=True, longActive=True)
    CC.actuators.accel = accel
    CC = CC.as_reader()
    ci.update([(0, frames)])  # the parser drops the first frame after construction
    ci.apply(CC, 0)
    out = []
    for _ in range(n):
      ci.update([(0, frames)])
      ci.CS.out.vEgo = v_ego
      ci.CS.out.standstill = v_ego == 0.0
      if brake_last is not None:
        ci.CC.brake_last = brake_last
      if throttle_last is not None:
        ci.CC.throttle_last = throttle_last
      _, sends = ci.apply(CC, 0)
      out += [dat for a, dat, bus in sends if (a, bus) == (addr, 2)]
    return out

  def test_brake_status_copy_says_what_the_camera_commanded_and_keeps_the_pedal(self):
    ci, packer = _forester()
    for cam_active, pedal in ((1, 0), (0, 1), (1, 1)):
      frames = [CanData(*packer.make_can_msg("ES_Brake", 2, {"Cruise_Brake_Active": cam_active, "Brake_Pressure": 100 * cam_active})),
                CanData(*packer.make_can_msg("Brake_Status", 0, {"ES_Brake": 1 - cam_active, "Brake": pedal}))]
      copies = self._copies(ci, frames, 0x13c, 4)
      assert len(copies) == 2
      for d in copies:
        assert (d[7] >> 2) & 1 == cam_active, (cam_active, pedal)  # ES_Brake, bit 58
        assert (d[7] >> 6) & 1 == pedal, (cam_active, pedal)       # Brake, bit 62

  def test_throttle_copy_maps_the_camera_command_and_keeps_the_pedal(self):
    ci, packer = _forester()
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

  def _tap(self, ci, packer, brake_last, throttle_last, v_ego=0.0, cam_brake=1, driver_pedal=0):
    frames = [CanData(*packer.make_can_msg("ES_Brake", 2, {"Cruise_Brake_Active": cam_brake, "Brake_Pressure": 297 * cam_brake})),
              CanData(*packer.make_can_msg("Throttle", 0, {"Throttle_Pedal": driver_pedal}))]
    ci.update([(0, frames)])
    ci.update([(0, frames)])
    ci.CS.out.vEgo = v_ego
    ci.CS.out.standstill = v_ego == 0.0
    ci.CC.brake_last, ci.CC.throttle_last = brake_last, throttle_last
    CC = structs.CarControl(enabled=True, longActive=True).as_reader()
    out = []
    for frame in range(4):
      out += [(dat[4], dat[6]) for addr, dat, bus in ci.CC.create_camera_copies(ci.CC.packer, CC, ci.CS, frame)
              if (addr, bus) == (0x40, 2)]
    return out

  def test_the_gas_tap_waits_until_openpilot_is_really_leaving(self):
    ci, packer = _forester()
    # openpilot still on its own brake, or not yet asking for throttle: nothing to back a tap up
    assert all(p == (0, 0) for p in self._tap(ci, packer, brake_last=130, throttle_last=2000))
    assert all(p == (0, 0) for p in self._tap(ci, packer, brake_last=0, throttle_last=1818))
    # brake released and real throttle commanded: pedal and combined throttle go out together
    assert all(p == (5, 5) for p in self._tap(ci, packer, brake_last=0, throttle_last=2000))
    # not once the car is moving, and not when the camera is not holding
    assert all(p == (0, 0) for p in self._tap(ci, packer, brake_last=0, throttle_last=2000, v_ego=1.0))
    assert all(p == (0, 0) for p in self._tap(ci, packer, brake_last=0, throttle_last=2000, cam_brake=0))

  def test_the_drivers_own_pedal_always_wins(self):
    ci, packer = _forester()
    assert all(p[0] == 17 for p in self._tap(ci, packer, brake_last=0, throttle_last=2000, driver_pedal=17))


class TestSubaruDisengageBeep(unittest.TestCase):
  def _alert(self, alpha_long):
    ci, packer = _forester(alpha_long)
    frames = [CanData(*packer.make_can_msg("ES_LKAS_State", 2, {"LKAS_Alert": 26}))]
    ci.update([(0, frames)])
    out = []
    for _ in range(20):
      ci.update([(0, frames)])
      _, sends = ci.apply(structs.CarControl(enabled=False).as_reader(), 0)
      out += [dat[4] & 0x1F for addr, dat, bus in sends if (addr, bus) == (0x322, 0)]
    return out

  def test_acc_disengaged_beep_is_filtered_only_under_openpilot_long(self):
    assert set(self._alert(alpha_long=False)) == {26}
    assert set(self._alert(alpha_long=True)) == {0}
