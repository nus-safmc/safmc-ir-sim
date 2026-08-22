"""Frames, angle conventions, and the bridge to the flight stack's NED frame.

ARENA frame (the canonical simulation frame, and ir-sim's native one)
--------------------------------------------------------------------
    x     metres, East
    y     metres, North
    z     metres, up, zero at the floor
    theta radians, yaw, COUNTER-CLOCKWISE positive from +x

NED map frame (what the real firmware speaks)
---------------------------------------------
    ned_x   metres, North          [esp-everything/main/wifi_task.h:36]
    ned_y   metres, East           [esp-everything/main/wifi_task.h:37]
    ned_z   metres, DOWN
    heading radians, zero at North, CLOCKWISE positive, in [0, 2*pi)

Nothing inside the simulator converts between the two. The conversion exists at exactly one
place -- here -- so that a future ROS 2 / MAVLink node has a single tested adapter to target.
See docs/08-porting-to-ros.md.
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "wrap_pi",
    "wrap_2pi",
    "arena_to_ned",
    "ned_to_arena",
    "arena_yaw_to_ned_heading",
    "ned_heading_to_arena_yaw",
]

_TWO_PI = 2.0 * np.pi


def wrap_pi(angle):
    """Wrap an angle, or array of angles, to (-pi, pi].

    The half-open interval matters: exactly -pi maps to +pi, so wrapping is idempotent and
    two angles that are numerically equal always compare equal after wrapping.
    """
    a = np.asarray(angle, dtype=float)
    wrapped = -(np.mod(-a + np.pi, _TWO_PI) - np.pi)
    # For inputs a hair above pi, `-a + pi` is a tiny negative number and np.mod rounds it to
    # exactly 2*pi, so the expression returns exactly -pi -- outside the declared half-open
    # interval, and breaking the idempotence this function promises. One representable double
    # in 2^64, but the interval is stated exactly, so close it exactly.
    wrapped = np.where(wrapped == -np.pi, np.pi, wrapped)
    return float(wrapped) if np.isscalar(angle) or wrapped.ndim == 0 else wrapped


def wrap_2pi(angle):
    """Wrap an angle, or array of angles, to [0, 2*pi)."""
    a = np.asarray(angle, dtype=float)
    wrapped = np.mod(a, _TWO_PI)
    return float(wrapped) if np.isscalar(angle) or wrapped.ndim == 0 else wrapped


def arena_yaw_to_ned_heading(theta):
    """ARENA yaw (CCW from East) -> NED heading (CW from North), in [0, 2*pi)."""
    return wrap_2pi(np.pi / 2.0 - np.asarray(theta, dtype=float))


def ned_heading_to_arena_yaw(heading):
    """NED heading (CW from North) -> ARENA yaw (CCW from East), in (-pi, pi]."""
    return wrap_pi(np.pi / 2.0 - np.asarray(heading, dtype=float))


def arena_to_ned(x, y, z, theta):
    """ARENA pose -> NED map pose.

    Returns ``(ned_x, ned_y, ned_z, heading)``. Note ned_z is DOWN, so a drone at 0.5 m
    altitude reports ned_z = -0.5 -- which is what PX4's LOCAL_POSITION_NED carries and what
    the firmware's ``mavlink_set_position_ned`` expects.
    """
    return (
        float(y),
        float(x),
        -float(z),
        float(arena_yaw_to_ned_heading(theta)),
    )


def ned_to_arena(ned_x, ned_y, ned_z, heading):
    """NED map pose -> ARENA pose. Returns ``(x, y, z, theta)``."""
    return (
        float(ned_y),
        float(ned_x),
        -float(ned_z),
        float(ned_heading_to_arena_yaw(heading)),
    )

