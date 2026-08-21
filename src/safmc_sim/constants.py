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
PILLAR_BASE_DIAMETER_M = 0.50   # sec 3.2, weighted base, 0.15 m tall
PILLAR_BASE_HEIGHT_M = 0.15     # sec 3.2

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
DRONE_BBOX_M = 0.30             # sec 6, must fit a 30 cm cube including propellers

# --------------------------------------------------------------------------------------
# HW -- read from nus-safmc/esp-everything at 99cde05 ("Competition over", 2026-04-02)
# --------------------------------------------------------------------------------------

TOF_SENSOR_COUNT = 8            # tof_task.h:10
TOF_ZONES_PER_SENSOR = 8        # tof_task.c:109, VL53L5CX_RESOLUTION_8X8, one row used
TOF_SENSOR_FOV_RAD = np.deg2rad(45.0)      # tof_task.h:36, half width 22.5 deg
TOF_SENSOR_SPACING_RAD = np.deg2rad(45.0)  # tof_task.c:183, 8 x 45 deg = 360 deg
TOF_ZONE_WIDTH_RAD = np.deg2rad(45.0 / 8.0)  # 5.625 deg per column, tof_task.c:258
TOF_MOUNT_RADIUS_M = 0.040      # safmc-ros/safmc_mapping/urdf/robot.urdf, cardinal sensors
TOF_MIN_VALID_M = 0.050         # tof_task.h:18, TOF_MIN_VALID_MM = 50
TOF_MAX_VALID_M = 3.000         # tof_task.h:17, TOF_MAX_VALID_MM = 3000 (firmware gate)
TOF_SENSOR_MAX_RANGE_M = 4.000  # VL53L5CX physical maximum, above the firmware gate
TOF_RATE_HZ = 15.0              # tof_task.c:475, 8 sensors x 8 ms round robin
TOF_COLLAPSED_BINS = 64         # tof_task.h:99, the only form nav consumes
VFH_BINS = 32                   # vfh.h:19

TOF_STATUS_VALID = 5            # tof_task.c:266, VL53L5CX target_status
TOF_STATUS_VALID_WEAK = 9       # tof_task.c:266
TOF_STATUS_NO_RETURN = 255      # tof_task.c:270, treated as INFINITY / free space
TOF_UNRELIABLE_SUBSTITUTE_M = 0.40  # tof_task.c, firmware's conservative "assume obstacle"

DRONE_RADIUS_M = 0.18           # vfh.h:43, the radius the real VFH planner uses
CRUISE_ALT_M = 0.5              # wifi_task.c:29, WIFI_CRUISE_ALT_M
CRUISE_SPEED_MS = 0.45          # nav_task.h:24, NAV_CRUISE_SPEED_MS
ARRIVE_RADIUS_M = 0.25          # nav_task.h:26, NAV_ARRIVE_RADIUS_M
YAW_TOLERANCE_RAD = 0.10        # nav_task.c, ROTATING -> FLYING threshold
COLLISION_DANGER_M = 0.40       # nav_task.c:122
COLLISION_CLEAR_M = 0.50        # nav_task.c:123
NAV_RATE_HZ = 20.0              # nav_task.c:510, the firmware navigation loop
SETPOINT_STALE_TIMEOUT_S = 0.30  # mavlink_task.c:93, SP_STALE_TIMEOUT_MS
MARKER_RATE_HZ = 2.0            # esp-everything/CLAUDE.md:54, measured AprilTag rate

# --------------------------------------------------------------------------------------
# ASSUME -- no published or measured source. See docs/FIDELITY.md section 3.
# --------------------------------------------------------------------------------------

WALL_THICKNESS_M = 0.10         # A-1  SAFMC publishes no wall thickness
VELOCITY_TAU_S = 0.35           # A-2  no step-response data from the airframe
CLIMB_RATE_MAX_MS = 0.5         # A-3  not present in firmware
MARKER_DETECT_RANGE_M = 3.0     # A-4  MEASURE THIS FIRST -- it dominates every search result
MARKER_DETECT_FOV_RAD = 1.0     # A-5  derived from fx=163.5 over 320 px, not measured
KNOWN_AREA_DEPTH_M = 14.0       # A-6  published in 2025, withdrawn from the 2026 table
UNKNOWN_AREA_DOORWAYS = 2       # A-7  rulebook diagram is explicitly not to scale
UNKNOWN_AREA_DOORWAY_M = 1.0    # A-7
YAW_RATE_MAX = 1.5              # A-3-adjacent; no published limit, PX4 default territory

# Simulation defaults (not claims about the world)
DEFAULT_TICK_HZ = 20.0          # matches NAV_RATE_HZ
DEFAULT_SEED = 0

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
