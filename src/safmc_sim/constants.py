"""Every magic number in one place, each with its provenance.

Three kinds of constant live here and they are kept visually distinct:

  RULE   -- published in a SAFMC 2026 rulebook. Changing one is a factual error.
  HW     -- read out of the flown firmware. Changing one desynchronises us from the drone.
  ASSUME -- chosen without published data. Each has an ID (A-n) tracked in docs/FIDELITY.md.

Nothing else in the package may contain a bare numeric literal for any of these quantities.
"""

from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------------------
# RULE -- SAFMC 2026 Category Swarm Challenge Booklet v2.0
# --------------------------------------------------------------------------------------

FIELD_WIDTH_M = 20.0            # sec 3.2, x extent (East)
FIELD_DEPTH_M = 20.0            # sec 3.2, y extent (North)
START_AREA_DEPTH_M = 6.0        # sec 3.2, full-width strip on the south edge
UNKNOWN_AREA_SIZE_M = 10.0      # sec 3.2, walled room (was 8.0 in 2025 Cat E)

PERIMETER_WALL_HEIGHT_M = 1.5   # sec 3.2, three sides only: west, north, east
INNER_WALL_HEIGHT_M = 2.0       # sec 3.2
PILLAR_DIAMETER_M = 0.30        # sec 3.2
PILLAR_HEIGHT_M = 2.0           # sec 3.2

MIN_GAP_WALL_TO_WALL_M = 2.0    # sec 3.2
MIN_GAP_PILLAR_M = 1.0          # sec 3.2, pillar-to-pillar and pillar-to-wall

CEILING_M = 1.4                 # sec 3.3.1 r.13, "not allowed to fly over walls"
SAFETY_NET_HEIGHT_M = 8.0       # Common Rules sec 3 item 12; not a flight limit

RUN_DURATION_S = 600.0          # sec 3.3.1 r.1, ten minutes, two runs, best counts
MAX_TAKEOFF_WAVES = 2           # sec 3.3.2
TAKEOFF_WAVE_WINDOW_S = 10.0    # sec 3.3.2, last departure within 10 s of the first
FLEET_MIN = 10                  # sec 3.3.1; fewer than 10 forfeits the run (Penalty #2)
FLEET_MAX = 25                  # sec 3.3.1

SCORE_RADIUS_M = 1.0            # sec 3.4, landed drone within 1 m AND line of sight
FIRE_SUPPRESSION_RADIUS_M = 2.5  # sec 3.4, unextinguished fire zeroes victims within this
RELAY_SPACING_M = 1.0           # sec 3.3.7, adjacent landed drones at most 1 m apart
RELAY_MULTIPLIER = 2.0          # sec 3.3.7, multiplies TOTAL mission score

POINTS_VICTIM = 5               # sec 3.4
POINTS_BONUS_VICTIM = 15        # sec 3.4
POINTS_FIRE = 10                # sec 3.4

N_VICTIMS = 4                   # sec 3.3.3 r.3
N_BONUS_VICTIMS = 4             # sec 3.3.3 r.3
N_FIRES = 4                     # sec 3.3.5 r.2

MARKER_FOOTPRINT_M = 0.30       # sec 3.3.3 r.3, 30 x 30 x 100 cm
MARKER_HEIGHT_M = 1.00          # sec 3.3.3 r.3 -- taller than cruise altitude, so it occludes

# --- navigation aids: where the team may put a UWB anchor or a fiducial ------------------
#
# sec 3.3.1 r.14: "Any navigation aids (e.g. ultra-wideband systems, fiducials) must be:
# placed during setup time; placed within the perimeter walls; easily removable; properly
# secured, e.g. will not topple over; cannot be secured to overhead structures; within
# 1 m x 1 m (no height limit)." So an anchor stands on its own tripod, and how high is up to
# the team. sec 6.3 permits ultra-wideband explicitly (5.7-5.9 GHz is banned; UWB is not).
NAV_AID_LIMIT_KNOWN_AREA = 10   # sec 3.3.1 r.15, "A maximum of TEN (10) navigation aids is
                                # allowed in the Known Search Area"
NAV_AID_LIMIT_START_AREA = None  # sec 3.3.1 r.16, "no limit on the number of navigation aids
                                # within the Start Area"
NAV_AID_LIMIT_UNKNOWN_AREA = 0  # sec 3.3.1 r.17, "Navigation aids are NOT allowed in the
                                # Unknown Search Area"
NAV_AID_FOOTPRINT_M = 1.0       # sec 3.3.1 r.14 f, "Within 1m x 1m (no height limit)"

# --------------------------------------------------------------------------------------
# HW -- read from nus-safmc/esp-everything at 99cde05 ("Competition over", 2026-04-02)
# --------------------------------------------------------------------------------------

# --- the ranging sensor: ST VL53L5CX -----------------------------------------------------
#
# Part number matters. The drone flies the **VL53L5CX**: the firmware links ST's
# vl53l5cx driver and calls vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8).
#
# Its near-twin the VL53L7CX is pin- and driver-compatible but has a 90 deg diagonal
# (60 x 60 deg square) FoV against the L5CX's 65 deg diagonal (45 x 45 deg square), and
# 3.5 m of range against 4.0 m. Swapping one for the other silently changes the zone width
# from 5.625 to 7.5 deg and turns a gapless ring into one with 120 deg of overlap.
#
# Note that nus-safmc/gazebo-slam-prototype's description says "8 x VL53L7CX". That is
# inconsistent with the flown firmware and with the mount geometry, and the team has
# confirmed the L5CX is what flies. Treat the other repo's description as stale.

TOF_SENSOR_PART = "VL53L5CX"
TOF_SENSOR_COUNT = 8            # tof_task.h:10
TOF_ZONES_PER_SENSOR = 8        # tof_task.c:109, VL53L5CX_RESOLUTION_8X8, one row used
TOF_SENSOR_FOV_RAD = np.deg2rad(45.0)
"""Square field of view of one sensor, per the datasheet (65 deg diagonal / sqrt(2) ~= 46,
specified by ST as a 45 x 45 deg square). The firmware agrees: TOF_SENSOR_HALF_WIDTH_DEG
is 22.5 (tof_task.h:36)."""

TOF_SENSOR_SPACING_RAD = np.deg2rad(45.0)  # tof_task.c:183
"""Angle between adjacent sensors. Equal to the FoV, which is not a coincidence: 8 x 45 deg
tiles a full circle exactly, which is *why* the airframe carries eight. Any other sensor
part breaks that -- see the note above."""

TOF_ZONE_WIDTH_RAD = TOF_SENSOR_FOV_RAD / TOF_ZONES_PER_SENSOR
"""5.625 deg. **Derived**, never written down twice. The firmware computes the same value
inline as (45.0 / 8.0) at tof_task.c:258; deriving it here means a change to the sensor's FoV
cannot leave the zone width behind."""
TOF_MOUNT_RADIUS_M = 0.040      # urdf robot.urdf, the four CARDINAL sensors (tof_n/e/s/w)
TOF_MOUNT_RADIUS_DIAGONAL_M = 0.034  # urdf: the diagonals sit at (0.02404, 0.02404), i.e.
                                # 34.0 mm -- the PCB is rectangular, so they are closer in
TOF_MIN_VALID_M = 0.050         # tof_task.h:18, TOF_MIN_VALID_MM = 50
TOF_MAX_VALID_M = 3.000         # tof_task.h:17, TOF_MAX_VALID_MM = 3000 (firmware gate)
TOF_SENSOR_MAX_RANGE_M = 4.000  # VL53L5CX datasheet maximum (the L7CX would be 3.5)
TOF_COLLAPSED_BINS = 64         # tof_task.h:99, the only form nav consumes

TOF_STATUS_VALID = 5            # tof_task.c:266, VL53L5CX target_status
TOF_STATUS_NO_RETURN = 255      # tof_task.c:270, treated as INFINITY / free space

DRONE_RADIUS_M = 0.18           # vfh.h:43, the radius the real VFH planner uses
CRUISE_ALT_M = 0.5              # wifi_task.c:29, WIFI_CRUISE_ALT_M
CRUISE_SPEED_MS = 0.45          # nav_task.h:24, NAV_CRUISE_SPEED_MS
NAV_RATE_HZ = 20.0              # nav_task.c:510, the firmware navigation loop
MARKER_RATE_HZ = 2.0            # esp-everything/CLAUDE.md:54, measured AprilTag rate

# --------------------------------------------------------------------------------------
# ASSUME -- no published or measured source. See docs/FIDELITY.md section 3.
# --------------------------------------------------------------------------------------

WALL_THICKNESS_M = 0.10         # A-1  SAFMC publishes no wall thickness
VELOCITY_TAU_S = 0.35           # A-2  no step-response data from the airframe
CLIMB_RATE_MAX_MS = 0.5         # A-3  not present in firmware
MARKER_DETECT_RANGE_M = 3.0     # A-4  MEASURE THIS FIRST -- it dominates every search result
MARKER_DETECT_FOV_RAD = 1.0     # A-5  derived from fx=163.5 over 320 px, not measured
UNKNOWN_AREA_DOORWAYS = 2       # A-7  rulebook diagram is explicitly not to scale
UNKNOWN_AREA_DOORWAY_M = 1.0    # A-7
YAW_RATE_MAX_RADS = 1.5              # A-3-adjacent; no published limit, PX4 default territory

# --- UWB ranging: a sensor the airframe does NOT carry (ADR-0006) ------------------------
#
# The module is unchosen, so nothing here is HW. These are literature values for
# DW1000/DW3000-class hobby modules (Qorvo DWM1001, Bitcraze Loco, Pozyx, Makerfabs MaUWB),
# each standing in for a measurement the team has not made; docs/FIDELITY.md section 3 says
# what would resolve each. A-10 is the one to measure first for THIS arena: whether the
# module reaches 20 m or 60 m decides where the anchors have to go.
UWB_LOS_NOISE_STD_M = 0.05      # A-9   DW1000 timestamp noise 3-4.5 cm; testbeds 2-8 cm
UWB_MAX_RANGE_M = 20.0          # A-10  datasheet 60 m LOS; hobby firmware 12-20 m indoors
UWB_NLOS_BIAS_M = 0.15          # A-11  one apartment wall on a DW1000: +0.49 ns mean
UWB_NLOS_NOISE_STD_M = 0.40     # A-11  ...with a 1.39 ns spread (Kolakowski, arXiv 2403.19706)
UWB_NLOS_DROP_PROBABILITY = 0.10  # A-12  no published rate for hobby modules
UWB_OUTLIER_PROBABILITY = 0.0   # A-13  the heavy positive tail is documented, its rate is not
UWB_OUTLIER_MAX_M = 1.5         # A-13  "up to 1.5 m" through-wall; p99 1.53 m behind a body

# Simulation defaults (not claims about the world)
DEFAULT_TICK_HZ = 20.0          # matches NAV_RATE_HZ
DEFAULT_SEED = 0

UWB_RATE_HZ = 10.0
"""One full sweep of the anchors per 100 ms. That is the ceiling of Decawave's PANS firmware
(a 100 ms TDMA superframe, four anchors per tag per frame) and what a MaUWB board delivers
with ten tag slots of 10 ms. A deployment choice, not a measurement; it must divide the tick
rate, so 20 Hz is the other sensible value on the 20 Hz loop."""

UWB_ANCHOR_HEIGHT_M = 2.0
"""Where the anchors' antennas sit. The rules put no limit on a navigation aid's height
(sec 3.3.1 r.14 f) and forbid hanging one from the ceiling (r.14 e), so this is a tripod. Two
metres is the inner walls' height, which is also the tallest an anchor can be before the
line-of-sight test -- made at the drone's altitude -- starts to over-report obstruction
(F-25). A deployment choice; set it to what the team actually erects."""

START_SPACING_M = 1.25
"""Centre-to-centre spacing of the take-off grid in the Start Area.

Not arbitrary. At 2 x DRONE_RADIUS_M = 0.36 m of body, this leaves ~0.89 m of clear air
between neighbours -- deliberately more than the ~0.8 m threshold a reactive avoidance policy
typically uses. Pack them tighter and every drone sees its neighbour inside its own
avoidance threshold at t=0, so the whole fleet turns on the spot forever and never leaves the
Start Area. That was observed at 0.72 m spacing and is a simulator artefact, not a strategy
failure: the real Start Area is 20 x 6 m and has ample room for 25 drones at this spacing
(15 per row, two rows, 2.5 m of depth)."""

START_WALL_MARGIN_M = 1.5
"""How far the take-off grid is kept from the field boundary.

Same class of artefact as START_SPACING_M and found the same way. At 0.66 m from the southern
boundary, every drone's rear-facing ranger reported ~0.62 m before it had moved, so a policy
with an omnidirectional 0.8 m avoidance threshold turned on the spot for the entire run
without ever leaving the Start Area. Real drones are placed by hand with room around them."""


# --------------------------------------------------------------------------------------
# REFERENCE ONLY -- researched values the simulator does not currently consume.
#
# These are kept deliberately rather than deleted. Each is a real number read out of the
# rulebook or the flown firmware, and each is something a policy author or a future feature
# will want: the thresholds the real navigation stack flies to, the ring's true update rate,
# the VFH bin count, the encodings our sensor does not currently emit. Deleting them would
# throw away the research; leaving them unlabelled would imply the simulator honours them.
# If you start using one, move it up into the block above.
# --------------------------------------------------------------------------------------
PILLAR_BASE_DIAMETER_M = 0.50   # sec 3.2, weighted base, 0.15 m tall
PILLAR_BASE_HEIGHT_M = 0.15     # sec 3.2
DRONE_BBOX_M = 0.30             # sec 6, must fit a 30 cm cube including propellers
TOF_RATE_HZ = 15.0              # tof_task.c:475, 8 sensors x 8 ms round robin. Also the
                                # VL53L5CX's own max rate at 8x8 (it does 60 Hz only at 4x4)
VFH_BINS = 32                   # vfh.h:19
TOF_STATUS_VALID_WEAK = 9       # tof_task.c:266
TOF_UNRELIABLE_SUBSTITUTE_M = 0.40  # tof_task.c, firmware's conservative "assume obstacle"
ARRIVE_RADIUS_M = 0.25          # nav_task.h:26, NAV_ARRIVE_RADIUS_M
YAW_TOLERANCE_RAD = 0.10        # nav_task.c, ROTATING -> FLYING threshold
COLLISION_DANGER_M = 0.40       # nav_task.c:122
COLLISION_CLEAR_M = 0.50        # nav_task.c:123
SETPOINT_STALE_TIMEOUT_S = 0.30  # mavlink_task.c:93, SP_STALE_TIMEOUT_MS
KNOWN_AREA_DEPTH_M = 14.0       # A-6  published in 2025, withdrawn from the 2026 table
