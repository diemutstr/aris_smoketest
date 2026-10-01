"""Calibration: what the arms measure about the rig with their own joints.  See
docs/modules/calib.md."""
from aris.calib.plane import (PlaneCalibration, calibrate_plane, calibration_dict,
                              calibration_from_events, fit_plane, write_calibration)

__all__ = ["PlaneCalibration", "calibrate_plane", "calibration_dict", "calibration_from_events",
           "fit_plane", "write_calibration"]
