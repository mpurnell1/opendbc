"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class SubaruSafetyFlagsSP:
  STOP_AND_GO = 1
  HIDE_CRUISE_BUTTONS = 2


class SubaruFlagsSP(IntFlag):
  STOP_AND_GO = 1
  STOP_AND_GO_MANUAL_PARKING_BRAKE = 2
  HIDE_CRUISE_BUTTONS = 4
