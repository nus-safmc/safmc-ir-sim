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
PILLAR_BASE_DIAMETER_M = 0.50   # sec 3.2, "includes a weighted circular base of 0.5m diameter
PILLAR_BASE_HEIGHT_M = 0.15     # and 0.15m height". Staging hardware -- a heavy foot so a 2 m
                                # column does not topple -- not an obstacle feature, so the
                                # published >= 1 m pillar gap is measured off the 0.30 m shaft,
                                # which is the only part inside the flight envelope. The base is
                                # modelled as a second, low height band so a drone descending to
                                # land beside a pillar meets it; see world/arena.py Pillar.

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

NAV_AID_MAX_KNOWN_AREA = 10     # sec 3.3.1 r.15, "a maximum of TEN (10) navigation aids is
                                # allowed in the Known Search Area. Teams are allowed to enter
                                # the Known Search Area only during setup time". r.16 puts no
                                # limit on the Start Area; r.17 allows NONE in the Unknown
                                # Search Area and bars teams from entering it at all times.
NAV_AID_FOOTPRINT_M = 1.0       # sec 3.3.1 r.14 f, "Within 1m x 1m (no height limit)" -- so an
                                # anchor stands on its own tripod, and how high is up to the
                                # team (r.14 e forbids hanging one from an overhead structure).

MARKER_FOOTPRINT_M = 0.30       # sec 3.3.3 r.3, 30 x 30 x 100 cm
MARKER_HEIGHT_M = 1.00          # sec 3.3.3 r.3 -- taller than cruise altitude, so it occludes

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
UNKNOWN_AREA_DOORWAYS = 4       # A-7  one per face. The sec 3.2 diagram draws four openings,
                                # one roughly centred on each side of the room, and 3.3.9 r.1
                                # says the swarm enters "via the open doorways shown in the
                                # diagram". Was 2 until the diagram was measured; two doorways
                                # halved the entrances and serialised every entry.
UNKNOWN_AREA_DOORWAY_M = 2.4    # A-7  the four openings measure 2.40-2.83 m on the diagram at
                                # 300 dpi. The diagram is explicitly not to scale, so this is
                                # still an assumption -- but 2.4 m is also exactly one maze cell
                                # (world/maze.py) and clears the published 2 m gap, so the three
                                # independent constraints agree. Was 1.0 m, which was 2.4x too
                                # narrow and had no source at all.
MAZE_CORRIDOR_M = 2.0           # A-9  the published >= 2 m wall-to-wall gap, reused as the floor
                                # on corridor width inside the Unknown Search Area. See
                                # world/maze.py for why this fixes the grid at 4 x 4.
YAW_RATE_MAX_RADS = 1.5              # A-3-adjacent; no published limit, PX4 default territory

# --- UWB ranging: the Qorvo DW3000, a part chosen but not yet carried (ADR-0006) ---------
#
# Part number matters, the same way it does for the ranging sensor above. The team has
# chosen the **DW3000** (Qorvo, formerly Decawave). It is not the DW1000 that most of the
# published UWB literature and every turnkey anchor kit uses, and the differences run the
# wrong way for a first guess:
#
#   * It is NOT faster. Per-frame airtime is essentially identical -- ~170 us against the
#     DW1000's ~180 us at 6.8 Mb/s -- so a ranging exchange costs the same. The DW3000's
#     real gains are power (~half) and 802.15.4z secure ranging.
#   * It is SHORTER-ranged. The DW3000 drops the DW1000's 110 kb/s long-range mode, and
#     Qorvo's own applications staff say that reduces maximum range.
#   * It is over-the-air compatible with the DW1000 only on channel 5 at 6.8 and 850 kb/s
#     (datasheet 1.2). Not pin compatible, not driver compatible: the register map differs.
#     What IS pin compatible is the *module* -- a DWM3000 drops into a DWM1000 footprint.
#
# So a number lifted from a DW1000 paper is a number about a different radio, and every one
# below says which it came from. Where a DW3000 measurement exists it is used; where none
# exists, the DW1000 figure is kept and labelled, which is the honest state of A-11 and A-12.
#
# The datasheet's headline "10 cm" is marketing copy from its first page. The electrical
# specification is Table 14: +/- 6 cm after calibration, +/- 15 cm without it, and a ranging
# standard deviation of 1.5 cm measured at -85 dBm with double-sided two-way ranging in line
# of sight. Those are three different quantities and only the last is our A-9.

UWB_MODULE_PART = "DW3000"
"""Qorvo DW3000. Four variants (DW3110/3120/3210/3220) differing only in package and PDoA
support, all with the same two channels; the module forms are DWM3000 (radio only, ships
UNCALIBRATED) and DWM3001C (radio plus an nRF52833, factory-calibrated and FCC/ETSI/IC
certified). Which one is bought changes F-26, not the model."""

UWB_LOS_NOISE_STD_M = 0.05
"""A-9. Bracketed rather than fitted. The datasheet's 1.5 cm (Table 14, at -85 dBm, DS-TWR,
calibrated, line of sight) is the floor. Two independent DW3000 measurements agree on the
ceiling: a mean absolute line-of-sight error of 5.7 cm in an office (Ember et al., IFIP 2024,
Fig. 12) and a median of 6 cm across nine rooms (Flueratoru et al., WiNTECH'22, Table 2).
Those carry residual bias and multipath this model does not separate out, so 5 cm sits near
the pessimistic end deliberately -- and still comes out about 30% optimistic against them
(F-30). A run that wants to match the measured part should set 0.07."""

UWB_MAX_RANGE_M = 20.0
"""A-10. Not a chip limit -- a configuration one. 20 m is what a stock MaUWB board reached in
an office at 6.8 Mb/s (CNX Software review, 2024-04-16); Qorvo's staff call 40-50 m typical
for line of sight; and a peer-reviewed study reached above 90 m in an indoor hallway by
moving to 850 kb/s with a 4096-symbol preamble and PAC 32 (Han & Jang, Sensors 2025,
25(10):3058), which also measured a 2-3x range gain from the PAC setting alone. The default
is the pessimistic stock-firmware figure; the team can buy back a factor of four in the
firmware before touching the hardware (F-29)."""

UWB_NLOS_BIAS_M = 0.15
"""A-11. The DW3000 evidence is thinner than the DW1000 evidence it replaces, and two
searches of it disagreed about what exists, so both are recorded. Ember et al. (IFIP 2024)
put a DW3000's non-line-of-sight mean absolute error at 46.7 cm in an office, with the 90th
percentile at 129.5 cm -- an aggregate over mixed obstructions, not a bias. Flueratoru et al.
(WiNTECH'22, Table 2) are reported to give medians per obstacle -- a half wall +0.08 m, a
door +0.10 m, a concrete pillar +0.57 m -- which this repository has NOT confirmed against
the paper itself, unlike the DW1000 figures below. This arena is 0.10 m walls and 0.30 m
concrete pillars, so any single number is a compromise; 0.15 m is kept, unchanged from the
DW1000 value, and the pair below reproduces an aggregate error close to Ember's (F-30)."""

UWB_NLOS_NOISE_STD_M = 0.40
"""A-11, and the one number here still inherited from the DW1000, because no DW3000 study
publishes a per-obstacle standard deviation. It is 1.39 ns = 0.42 m, rounded, measured
through one wall or obstacle on a DW1000 in a flat of pre-stressed concrete panels
(Kolakowski & Modelski, TELFOR 2017, Table 1; arXiv 2403.19706). That paper was read in full
here and the numbers are verbatim, which is more than can be said for the DW3000 sources.
The same table gives 1.92 ns and 2.02 ns -- about +0.58 m and 0.61 m -- behind several walls,
which the model does not use (F-24). See F-28."""

UWB_NLOS_DROP_PROBABILITY = 0.10
"""A-12. No published through-wall dropout rate exists for either part -- a search for one
came back empty. The only DW3000 evidence is qualitative: through a wall into an adjacent
room the received level fell to about -90 dBm and the signal became, in the reviewer's words,
extremely difficult to catch (CNX Software, 2024-04-16). One 802.15.4z study is a warning in
the other direction: with packet reception at 100%, *secure* timestamping still failed 10-40%
of the time on the DW3000's own consistency checks, and under Wi-Fi 6E interference more than
half. A tag configured for secure ranging would drop more, not fewer. Measure it."""

UWB_OUTLIER_PROBABILITY = 0.0
"""A-13. Off by default, and the term that would close the largest gap between this model and
the measured part. A DW3000's measured non-line-of-sight 90th percentile is 129.5 cm (Ember
et al., IFIP 2024) against this model's 70 cm with the outlier term off: the real tail is
heavier than a Gaussian, which is what this term is for. Its rate is still unpublished, so
turning it on is a choice a run must make and report (F-30)."""

UWB_OUTLIER_MAX_M = 1.5
"""A-13. Within the ~2.5 m tail the DW3000 CDF reaches (Flueratoru et al., Fig. 3a)."""

# Sources for the UWB block, one per number (docs/README.md asks for a URL per claim):
#   Datasheet   DW3000 Datasheet v1.3 (Decawave/Qorvo, 2020). Table 14 sec 3.9 is the ranging
#               specification; Table 16 sec 4.2 the channels; Table 12 sec 3.7 the link
#               budget; Figure 30 sec 6.4.5 the DS-TWR frame timing.
#   A-9, A-11   E. Ember et al., "Improving the Correction of NLoS-Induced Ranging Errors in
#               UWB Systems through Enhanced Labeling", IFIP 2024. DW3000 on an nRF52,
#               channel 9, BPRF, 64-symbol STS, 125 positions in a 60x40 m office; Fig. 12
#               gives LoS MAE 5.7 cm / 90th 13.7 cm and NLoS MAE 46.7 cm / 90th 129.5 cm.
#               A. Flueratoru, E. S. Lohan, D. Niculescu, "Challenges in Platform-Independent
#               UWB Ranging and Localization Systems", ACM WiNTECH'22, doi
#               10.1145/3556564.3558238, reported to give per-obstacle medians in Table 2 --
#               NOT confirmed against the paper here, unlike the DW1000 source below.
#   A-11 spread M. Kolakowski, J. Modelski, "First path component power based NLOS mitigation
#               in UWB positioning system", TELFOR 2017, Table 1 (arXiv 2403.19706). DW1000.
#   A-10        cnx-software.com MaUWB DW3000 review, 2024-04-16 (the 20 m office figure);
#               Qorvo forum "DWM3000EVB max range" (40-50 m typical); B. Han, J. Jang,
#               "Extending the Coverage of IEEE 802.15.4z HRP UWB Ranging", Sensors 2025,
#               25(10):3058 (above 90 m indoors at 850 kb/s).
#   A-12, F-24  cnx-software.com as above, qualitative only.
#   F-23, F-28  Makerfabs UWB AT Command Manual v1.0.8 sec 3.11 (the TDMA slot arithmetic and
#               the 8-anchor cap); DW3000 datasheet Figure 30; Decawave APS013 v2.3 (one
#               frame per millisecond at 6.8 Mb/s, to hold the regulatory mean power limit).
#   F-26        DW3000 datasheet Table 14 note 1 (+/-15 cm uncalibrated, +/-6 cm calibrated);
#               DWM3000 Data Sheet Rev B sec 2 ("no transmit power or antenna delay
#               calibration"); DWM3001C Data Sheet Rev B sec 1 (factory calibrated).

# Simulation defaults (not claims about the world)
DEFAULT_TICK_HZ = 20.0          # matches NAV_RATE_HZ
DEFAULT_SEED = 0

UWB_RATE_HZ = 10.0
"""One synchronous sweep of every anchor per 100 ms. The period matches Decawave's PANS
superframe, but PANS itself returns four ranges per tag per frame; a sweep of all N anchors
at 10 Hz is what MaUWB-class firmware delivers (eight anchors per 10 ms tag slot, ten slots).
A deployment choice, not a measurement (F-23); it must divide the tick rate, so 20 Hz is the
other sensible value on the 20 Hz loop."""

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
DRONE_BBOX_M = 0.30             # sec 6, must fit a 30 cm cube including propellers
UWB_CHANNEL_5_HZ = 6_489_600_000.0
UWB_CHANNEL_9_HZ = 7_987_200_000.0
UWB_CHANNEL_BANDWIDTH_HZ = 499_200_000.0
"""The only two channels the DW3000 has (datasheet Table 16, sec 4.2), and the reason the
part is legal here. Rule 6.3 bans wireless transmission in 5.7-5.9 GHz on pain of immediate
disqualification and permits ultra-wideband in the same breath; channel 5 occupies
6240.0-6739.2 MHz and channel 9 occupies 7737.6-8236.8 MHz, so the nearer band edge sits
340 MHz clear of 5900 MHz. The datasheet's own channel-5 spectrum plot (Figure 4) reads
about -71 dBm/MHz or lower across 5.7-5.9 GHz, some 30 dB under the in-band plateau. Note
the DW3000 does NOT offer channel 7, whose 1081.6 MHz bandwidth about the same centre would
have reached down to 5948.8 MHz. The simulator models no radio band; these are here so the
compliance argument is written down where the numbers live."""

UWB_SLOT_S = 0.010
"""One tag's TDMA slot at 6.8 Mb/s, the floor a shipping firmware allows (Makerfabs AT
manual v1.0.8 sec 3.11; 850 kb/s needs 15 ms). Slots are per TAG, and every anchor is swept
inside one -- so the sweep rate falls with FLEET size, not anchor count. See
``uwb.sweep_rate_hz`` and F-28."""

UWB_MAX_ANCHORS_FIRMWARE = 8
"""Anchors one shipping AT firmware supports (Makerfabs, sec 1.2: 8 anchors, 64 tags). The
rules allow ten aids in the Known Search Area plus any number in the Start Area, so a legal
layout can exceed what an off-the-shelf tag will actually range to. Not enforced."""

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
KNOWN_AREA_DEPTH_M = 14.0       # DERIVED, not assumed: FIELD_DEPTH_M - START_AREA_DEPTH_M.
                                # This carried assumption id A-6 and the note "published in 2025,
                                # withdrawn from the 2026 table". Both halves were wrong. The
                                # Play Field Element tables in booklet v1 and v2 are
                                # character-identical and NEITHER has a Known Search Area row, so
                                # nothing was ever published and nothing was withdrawn. The
                                # figure is arithmetically forced by two published numbers, so
                                # A-6 has been retired rather than left as a standing unknown.
