"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarStateBase

from opendbc.sunnypilot.car.subaru import subarucan_ext
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP

# The camera's ACC stays engaged under openpilot longitudinal (it reads the cruise switch by a path the
# harness never sees) and faults about half a second after the car's response stops matching its
# command: the brake module's ES_Brake echo, and the ECM's cruise throttle. Each copy below replaces
# that car message on the camera bus (the safety model blocks the real one) so the camera sees the
# response its own command would have produced. The driver's pedals in the copies stay real.
BRAKE_STATUS_STEP = 2  # 50 Hz
THROTTLE_STEP = 1      # 100 Hz

# The ECM's Throttle_Cruise against the camera's Cruise_Throttle, medians of 703k steady stock
# frames on the Forester
THROTTLE_CRUISE_BP = [808, 1818, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3450, 3900]
THROTTLE_CRUISE_V = [0, 2, 15, 20, 23, 28, 35, 45, 56, 63, 87]

# The camera holds the car at a standstill on its own brake until the driver taps the gas or RESUME;
# openpilot pulling away under that hold is the one disagreement the copies cannot hide, so the copy
# carries the stock stop-and-go gas tap first and the camera releases the hold itself
GAS_TAP_FRAMES = 15


class CameraCopiesController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.camera_echo = bool(CP_SP.flags & SubaruFlagsSP.CAMERA_ECHO)
    self.gas_tap_frames = 0
    self.tapped = False

  def create_camera_copies(self, packer, CC: structs.CarControl, CS: CarStateBase, frame: int) -> list[CanData]:
    can_sends = []
    if not self.camera_echo:
      return can_sends

    cam_brake_active = CS.es_brake_msg["Cruise_Brake_Active"]
    if frame % BRAKE_STATUS_STEP == 0:
      can_sends.append(subarucan_ext.create_brake_status(packer, frame // BRAKE_STATUS_STEP, CS.brake_status_msg, cam_brake_active))

    camera_hold = CC.longActive and CS.out.standstill and cam_brake_active
    if not camera_hold:
      self.tapped = False
      self.gas_tap_frames = 0
    elif CC.actuators.accel > 0 and not self.tapped:
      self.tapped = True
      self.gas_tap_frames = GAS_TAP_FRAMES
    gas_tap = self.gas_tap_frames > 0
    if gas_tap:
      self.gas_tap_frames -= 1

    throttle_cruise = int(round(np.interp(CS.es_distance_msg["Cruise_Throttle"], THROTTLE_CRUISE_BP, THROTTLE_CRUISE_V)))
    can_sends.append(subarucan_ext.create_throttle_echo(packer, frame, CS.throttle_msg, throttle_cruise, gas_tap))

    return can_sends
