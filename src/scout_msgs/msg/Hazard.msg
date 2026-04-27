# A detected / confirmed hazard in the map.
#
# Used twice in the pipeline:
#   * hazard_detector -> /hazards/raw        (one per detection frame, may duplicate)
#   * hazard_tracker  -> /hazards/confirmed  (deduplicated, stable IDs, latched)

std_msgs/Header header        # stamp + frame_id (always "map" for confirmed)

int32 id                      # stable numeric ID (tracker-assigned; -1 on raw stream)
string color                  # "red" | "yellow" | "blue" | ...  (from detector config)
string category               # semantic label, e.g. "fire" | "chemical" | "medical"

geometry_msgs/Point position  # map-frame (x, y, z) of the cube center estimate

float32 confidence            # detector confidence in [0.0, 1.0]
int32 observation_count       # how many raw detections the tracker has merged into this
