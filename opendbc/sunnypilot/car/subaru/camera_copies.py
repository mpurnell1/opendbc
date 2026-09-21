"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarStateBase

from opendbc.sunnypilot.car.subaru import subarucan_ext
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP

# The camera's ACC faults about half a second after the car stops following its command. Each copy
# below replaces a car message on the camera bus (the safety model blocks the real one) so the camera
# sees a car that never disagrees with it, or never engages it.
CRUISE_BUTTONS_STEP = 2  # 50 Hz
BRAKE_STATUS_STEP = 2    # 50 Hz

# Probe: one RESUME tap this long after stock ACC engages. A set speed that rises by one says the
# camera takes presses over CAN, which is what a Subaru ICBM port needs.
PROBE_DELAY_FRAMES = int(5.0 / DT_CTRL)
PROBE_PRESS_FRAMES = int(0.12 / DT_CTRL)


class CameraCopiesController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.hide_buttons = bool(CP_SP.flags & SubaruFlagsSP.HIDE_CRUISE_BUTTONS)
    self.button_probe = bool(CP_SP.flags & SubaruFlagsSP.CRUISE_BUTTON_PROBE)
    self.brake_echo = bool(CP_SP.flags & SubaruFlagsSP.CAMERA_BRAKE_ECHO)

    self.engaged_frame = None
    self.probe_done = False

  def _probe_press(self, CS: CarStateBase, frame: int) -> bool:
    if not CS.out.cruiseState.enabled:
      self.engaged_frame = None
      self.probe_done = False
      return False
    if self.engaged_frame is None:
      self.engaged_frame = frame
    since = frame - self.engaged_frame
    if since >= PROBE_DELAY_FRAMES + PROBE_PRESS_FRAMES:
      self.probe_done = True
    return not self.probe_done and since >= PROBE_DELAY_FRAMES

  def create_camera_copies(self, packer, CS: CarStateBase, frame: int) -> list[CanData]:
    can_sends = []

    if (self.hide_buttons or self.button_probe) and frame % CRUISE_BUTTONS_STEP == 0:
      msg = CS.cruise_buttons_msg
      probe = self.button_probe and self._probe_press(CS, frame)
      set_pressed = self.button_probe and msg["Set"]
      resume_pressed = self.button_probe and (msg["Resume"] or probe)
      can_sends.append(subarucan_ext.create_cruise_buttons(packer, frame // CRUISE_BUTTONS_STEP, msg, set_pressed, resume_pressed))

    if self.brake_echo and frame % BRAKE_STATUS_STEP == 0:
      can_sends.append(subarucan_ext.create_brake_status(packer, frame // BRAKE_STATUS_STEP, CS.brake_status_msg,
                                                         CS.es_brake_msg["Cruise_Brake_Active"]))

    return can_sends
