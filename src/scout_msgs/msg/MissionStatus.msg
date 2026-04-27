# High-level status broadcast by the mission_manager so dashboards / the UR7 side
# can follow along without having to reason about Nav2 internals.

std_msgs/Header header

# ---- phase ----
uint8 PHASE_IDLE=0
uint8 PHASE_MAPPING=1          # explore_lite + hazard_detector running
uint8 PHASE_PLANNING=2         # mapping done, deciding hazard visit order
uint8 PHASE_EN_ROUTE=3         # navigating toward a hazard
uint8 PHASE_AT_HAZARD=4        # arrived, requesting package from arm
uint8 PHASE_RETURNING=5        # heading back to dock / arm pickup
uint8 PHASE_COMPLETE=6
uint8 PHASE_ABORTED=7

uint8 phase

# ---- current hazard context (only meaningful in EN_ROUTE / AT_HAZARD) ----
int32 current_hazard_id        # -1 if none
string current_hazard_category # "" if none

# ---- counters ----
int32 hazards_total
int32 hazards_visited

string message                 # free-form human-readable status line
