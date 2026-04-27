"""Pluggable hazard detector backends.

The ``hazard_detector`` ROS node selects one of these at runtime via the
``detector`` parameter. Each backend implements :class:`base.Detector`:
given a BGR OpenCV image it returns a list of :class:`base.Detection`
records describing the color-labeled bounding boxes it saw.

Current backends:
    hsv  -- classic OpenCV HSV thresholding + contour blob.
    yolo -- stub; raises at construction time until real weights are wired up.

To add a new backend, drop a module here, subclass ``Detector``, and
register it in :func:`build_detector`.
"""
from .base import Detection, Detector
from .hsv import HSVDetector
from .yolo import YoloDetector


def build_detector(kind: str, config: dict) -> Detector:
    """Factory used by hazard_detector.py.

    Raises ValueError on an unknown backend so the node fails fast rather
    than silently running with no perception."""
    kind = (kind or '').lower()
    if kind in ('hsv', 'opencv', ''):
        return HSVDetector(config)
    if kind == 'yolo':
        return YoloDetector(config)
    raise ValueError(f'Unknown detector backend: {kind!r}')


__all__ = ['Detection', 'Detector', 'HSVDetector', 'YoloDetector', 'build_detector']
