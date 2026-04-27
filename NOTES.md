# Scout project — learning notes

Running log of substantive concept questions and their answers, for
future-me reference. Condensed from chat discussions; not exhaustive.

Entries in chronological order (oldest first).

---

## Q1 — `run()` method vs `rclpy.spin(node)` in `main()`

**Context:** lab examples all use `rclpy.spin(node)` in `main()`. Our
`scout_teleop` / `manual_mapper` / `mission_manager` instead call a
custom `node.run()`. Why the difference?

**Answer:** two valid patterns for driving a ROS 2 node.

### Pattern A — callback-driven (`rclpy.spin`)

```python
rclpy.init()
node = MyNode()
rclpy.spin(node)        # framework runs the loop
node.destroy_node()
rclpy.shutdown()
```

`rclpy.spin()` says "sit here forever and invoke my callbacks when
events happen." Events = message on a subscribed topic, timer firing,
incoming service request, action goal. You never write a loop; the
framework runs it and calls your callbacks.

Use when: the node is driven by ROS events (subs, timers, services).

### Pattern B — imperative (`node.run()`)

```python
rclpy.init()
node = MyNode()
try:
    node.run()          # YOU drive the loop
finally:
    node.destroy_node()
    rclpy.shutdown()
```

Your `run()` is a `while rclpy.ok():` loop that does whatever you want
in whatever order. rclpy is still initialized so publishers / TF / etc.
work, but nothing is spinning you.

Use when: the node is driven by a blocking external source (keyboard,
joystick, file, socket) or by imperative sequencing (state machine
stepping through phases).

### Rule of thumb

| Driven by | Pattern |
|---|---|
| Topic messages, timers, callbacks only | A (`rclpy.spin`) |
| Blocking external input (keyboard etc.) | B (own loop) |
| Imperative sequencing / FSM | B (own loop) |
| Mix of the above | B, with `rclpy.spin_once(node, timeout_sec=0)` inside the loop to pump callbacks, OR use a `MultiThreadedExecutor` on another thread |

### Which nodes in this project use which

| Node | Pattern | Why |
|---|---|---|
| `scout_teleop` | B | keyboard loop |
| `manual_mapper` | B | keyboard loop + map save |
| `mission_manager` | B | FSM sequencing is clearer imperatively |
| `hazard_detector` | A | image subscriber |
| `hazard_tracker` | A | subscriber + timer |
| `auto_mapper` | A | subscriber + timer |
| `scan_resampler` | A | subscriber |
| `ur7_stub` | A | pure service |

Neither is more "correct"; pick the one that matches how events arrive.

---

## Q2 — What is `termios` and why does keyboard teleop need it?

**Context:** `scout_teleop._get_key()` uses `termios`, `tty`, and
`select` together. Why all three, and what does the terminal have to do
with it?

**Answer:** `termios` ("terminal I/O settings") is the POSIX module
that controls how a Unix terminal device behaves when a program reads
from it. Teleop needs to change the default terminal mode.

### Canonical (cooked) mode — the default

- Input is **line-buffered**: the OS holds keystrokes until Enter is
  pressed, then delivers the whole line to the program.
- Keystrokes are **echoed** to the screen.
- **Special keys intercepted**: Ctrl-C → SIGINT, Ctrl-Z → suspend,
  Backspace → line edit, Ctrl-D → EOF.

Good for shells, terrible for teleop (you'd have to press Enter after
every 'w').

### Raw mode — what teleop uses

- **No buffering**: each character delivered the instant it's typed.
- **No echo**: typing 'w' doesn't print 'w' on the terminal.
- **No special-key handling**: Ctrl-C arrives as literal byte `\x03`
  instead of firing SIGINT. This is why our loop checks for `\x03`
  explicitly to break.

### API reference

| Call | Purpose |
|---|---|
| `termios.tcgetattr(fd)` | Read current terminal settings; save to restore later. |
| `termios.tcsetattr(fd, when, settings)` | Apply settings. `when=TCSADRAIN` means "wait for output to drain first, then apply." |
| `tty.setraw(fd)` | Convenience: construct raw-mode settings and apply them in one call. |
| `select.select([sys.stdin], [], [], timeout)` | Not termios, but paired: "wait up to `timeout` seconds for stdin to have data." Prevents `_get_key()` from blocking forever. |

### The dance in our code

1. `main()` calls `tcgetattr(sys.stdin)` **once**, stashes result in
   module-level `settings` global. Snapshot of "normal" mode.
2. Each `_get_key()` call:
   - `tty.setraw()` — flip stdin into raw mode.
   - `select.select(..., 0.1)` — wait up to 100 ms for input.
   - Read one character if ready; return `''` if not.
   - `tcsetattr(..., settings)` — restore saved (normal) settings so
     the terminal behaves sanely between reads.
3. `main()`'s `finally` block restores settings one more time as a
   safety net for crashes mid-read.

### Why restore matters

If you exit without restoring, the terminal stays in raw mode after the
process dies. Symptoms: you type and nothing echoes, Enter doesn't
submit, Ctrl-C does nothing. Looks broken; isn't. Blind-type `reset`
+ Enter to fix. Paranoid restore logic avoids this entirely.

### Why `settings` is a module-level global

It has to be captured once at the start of `main()` (before the node
touches the terminal) and read from inside `_get_key()` (a method). A
global is the most compact way to bridge the two without passing it
through the `Node.__init__` signature. Slightly ugly, idiomatic for
this kind of raw-mode script.

---

## Q3 — What is the `tty` module and how does it relate to `termios`?

**Context:** `_get_key()` imports both `tty` and `termios`. Why both?

**Answer:** `tty` is a thin convenience wrapper around `termios`.
It provides only two functions:

| Function | Mode |
|---|---|
| `tty.setraw(fd)` | Raw: no line buffering, no echo, no signal handling (Ctrl-C arrives as byte `\x03`). |
| `tty.setcbreak(fd)` | cbreak: no line buffering, no echo, **but signal handling still works** (Ctrl-C → SIGINT). |

Both internally call `termios.tcgetattr`, twiddle the right flag bits,
then call `termios.tcsetattr`. You could write either one using only
`termios` — nobody does because `tty.setraw(fd)` is a one-liner that
bundles the flag twiddling for you.

### Mental model

- `termios` = low-level POSIX API (read/write terminal attributes;
  matches the C library function names).
- `tty` = convenience presets that compose termios calls for common
  cases (raw, cbreak).

### Why `setraw` and not `setcbreak` for our teleop

cbreak is tempting (Ctrl-C fires SIGINT naturally; no `\x03` check
needed). But we don't want SIGINT interrupting ROS mid-publish or
mid-`rclpy.spin_once()`. We want deliberate shutdown through our own
loop's exit path, which then lets `main()`'s `finally` block restore
the terminal and call `rclpy.shutdown()` in the right order. With
`setraw`, Ctrl-C is just another byte — our loop decides when to
break.

---

## Q4 — Where is the SLAM logic? `manual_mapper` has no mapping code.

**Context:** Reading `manual_mapper.py`, the only logic is "read keys,
publish Twist, save map on 'p'." There's no scan matching, no graph
optimization, no occupancy grid building. So where does the map come
from?

**Answer:** SLAM is a separate node — `slam_toolbox`. Our nodes don't
do mapping at all; they only drive the robot so the lidar can see new
areas.

### Architecture

```
Gazebo (or real lidar)
   |
   |-- /scan         (laser data)
   |-- /tf           (odom -> base_link from wheel encoders)
   v
slam_toolbox  ---->  /map   (occupancy grid, RViz displays this)
   ^                 /tf    (map -> odom correction)
   |
   +-- needs the robot to move to cover new areas
       |
       +-- driver: manual_mapper  (human WASD)
                OR auto_mapper + explore_lite  (frontier exploration)
```

`slam_toolbox` is launched by `system.launch.py` (lines 168-185 in the
`slam_sim` / `slam_real` nodes). It's a third-party ROS 2 package by
Steve Macenski, installed in `~/turtle_test/install`. We never touch
the SLAM math.

Our scout-side mapping nodes (`manual_mapper`, `auto_mapper`) are
**driver-layer**: they decide where the robot goes. SLAM happens in a
different process, in parallel, regardless of who's driving.

### What slam_toolbox is doing under the hood

It's **pose-graph SLAM with scan matching**:

1. **Scan matching** — align each new lidar scan against recent scans
   (correlative scan matching). Refines the pose estimate beyond what
   wheel-odometry alone gives.
2. **Build the graph** — each keyframe (scan + pose) becomes a *node*;
   relative transforms between scans become *edges* with uncertainty.
3. **Loop closure** — when the robot revisits a place, slam_toolbox
   detects the match, adds an edge between the current node and the
   old one.
4. **Graph optimization** — solve the graph with a least-squares solver
   (CERES) to find the pose configuration that best satisfies all edge
   constraints simultaneously. This is what fixes drift; without it,
   long runs would slowly skew the map.
5. **Render the occupancy grid** — ray-trace each scan into a grid,
   mark cells occupied / free / unknown, publish on `/map`.

### Other SLAM approaches (context)

- **Particle-filter SLAM** (gmapping) — older, cheaper, less accurate,
  worse loop closure. Was the TurtleBot3 default before slam_toolbox.
- **Visual SLAM** (ORB-SLAM, RTAB-Map) — uses cameras instead of lidar.
  Different math, similar graph backbone for the optimization step.
- **Pure odometry "mapping"** — integrate wheel encoders only. Not
  SLAM. Drifts horribly. Useless beyond a few meters.

slam_toolbox is the modern default for indoor lidar robots: handles
loop closure well, runs in real time on modest hardware, actively
maintained.

### Why we don't write our own SLAM

(a) slam_toolbox is genuinely state-of-the-art; reimplementing would be
worse. (b) Writing graph SLAM from scratch is a semester-long project
in itself. (c) The course is about *using* SLAM as a building block,
not implementing it. Same reason we don't write Nav2 from scratch.

---

## Q5 — How does `manual_mapper` save the map? (map_saver_cli)

**Context:** `manual_mapper.save_map()` shells out to
``ros2 run nav2_map_server map_saver_cli -f <path>``. What is that
and why shell out instead of calling a service?

**Answer:** `map_saver_cli` is a small standalone CLI tool from the
`nav2_map_server` package. On invocation it:

1. Starts up and subscribes to `/map`.
2. Receives one message immediately (because `/map` is published with
   TRANSIENT_LOCAL QoS — late subscribers still get the latest map).
3. Writes two files:
   - `<path>.pgm` — the occupancy grid as a PGM image (black = wall,
     white = free, gray = unknown).
   - `<path>.yaml` — metadata (resolution, origin, thresholds). This
     is the file Nav2's `map_server` loads later.
4. Exits.

`manual_mapper` itself knows nothing about maps; it just spawns a
subprocess that does. `subprocess.run(..., check=True)` **blocks**
until the CLI exits — usually a few seconds.

### Why shell out instead of calling `/slam_toolbox/save_map`?

slam_toolbox offers its own service for saving, but we use the CLI
approach because:

- **Portable** — works with any `/map` publisher, not just slam_toolbox.
  If we swap the SLAM backend later, this code still works.
- **Format-correct** — the `.yaml` + `.pgm` pair is exactly what Nav2's
  `map_server` expects to load back in.
- **Simpler** — one `subprocess.run` vs setting up a service client
  for a one-shot operation.

Trade-off: slightly slower (process spawn overhead) and harder to
error-handle granularly. For a "user pressed p, save the map" action
that runs twice a session, these costs are invisible.

### Gotcha: stdin buffering during save

Because `subprocess.run` blocks, any keys pressed during the save are
buffered in stdin and get read on the next loop iterations after
`save_map()` returns. Fix if needed:

```python
elif key == 'p':
    self.save_map()
    termios.tcflush(sys.stdin, termios.TCIFLUSH)  # discard buffered keys
```

---

## Q6 — What is a Nav2 lifecycle manager and why do we need one?

**Context:** Launching the keepout filter required a
`nav2_lifecycle_manager` node in addition to the two filter nodes
themselves. Why can't the filter nodes just start on their own like
a normal ROS 2 node?

**Answer:** Nav2's core nodes are **managed / lifecycle nodes**, a
ROS 2 concept that gives a node a formal state machine instead of
just "running or not."

### The lifecycle state machine

A managed node has four primary states:

```
   unconfigured ── configure ──> inactive
                                   │
                                 activate
                                   │
                                   v
                                 active ── deactivate ──> inactive
                                   │
                                 shutdown
                                   │
                                   v
                               finalized
```

- `unconfigured`: started but hasn't loaded config/parameters yet.
- `inactive`: configured (params loaded, publishers/subs created), but
  not publishing / not processing. Safe to reconfigure.
- `active`: publishing and processing normally.
- `finalized`: shutting down cleanly.

Transitions are triggered via ROS 2 services (`/configure`, `/activate`,
`/deactivate`, `/cleanup`, `/shutdown`) that every managed node exposes.

### Why this exists

Nav2 is a distributed system: many nodes (`planner_server`,
`controller_server`, `map_server`, filter servers, etc.) must come up
in the right order with coordinated state:

- Parameter files have to be loaded before topics are advertised.
- Subscribers have to exist before their partner publishers start
  pumping data.
- One component failing shouldn't leave others in a half-started state.

Without the lifecycle, you'd get race conditions at startup
("map_server published /map before planner_server subscribed") and
cascading failures at shutdown.

The lifecycle gives Nav2 an ordered "configure everything → activate
everything → system is ready" sequence, with the ability to tear down
and restart subsets cleanly.

### Role of the `lifecycle_manager` node

A `nav2_lifecycle_manager` instance is a supervisor: you give it a
list of managed node names, and it walks each one through the state
machine in order. It handles:

- **Startup ordering**: `configure` all nodes first, then `activate`
  them in list order. If any step fails, log and bail.
- **Health checks**: the default bond-based health check sees if a
  managed node dies and tears down the others to avoid partial state.
- **Shutdown ordering**: deactivate → cleanup → shutdown, reverse order.

Without a lifecycle manager in your launch, your managed nodes would
start up and sit in `unconfigured` forever — nobody is calling their
`/configure` service. That's why we needed `lifecycle_manager_filters`:
`filter_mask_server` and `costmap_filter_info_server` are both managed
nodes, and something has to flip them from unconfigured → active.

### The `autostart: True` parameter

In our launch we set `autostart: True` on the lifecycle manager. That
means "on startup, immediately walk all managed nodes through
configure + activate, without waiting for an external call to
`/bringup_all`." Without it, you'd have to manually call a service
after launch to bring the filters online. `autostart: True` is the
standard pattern for "I always want these active."

### Which Nav2 nodes in our stack are lifecycle nodes

Pretty much all of them:

- `map_server`, `amcl`, `planner_server`, `controller_server`,
  `bt_navigator`, `behavior_server`, `smoother_server`,
  `waypoint_follower`, `velocity_smoother`.
- `filter_mask_server`, `costmap_filter_info_server` (our additions).

The main Nav2 bringup launch already includes its own
`lifecycle_manager_navigation` for the core set. We added a separate
`lifecycle_manager_filters` for our filter nodes to keep their
lifecycle independent of the main Nav2 stack.

### What non-lifecycle nodes look like by comparison

Our scout nodes (`scout_teleop`, `manual_mapper`, `hazard_detector`,
etc.) are plain `rclpy.node.Node` instances. They start up, run, get
Ctrl-C'd, exit. No state machine, no manager. That's fine because
they're self-contained — no inter-node state ordering to coordinate.

---

## Q7 — Why did we bump the spawn pose to (-2.5, -2.5)?

**Context:** The default spawn was `(-2.6, -3.0)`. After adding the
keepout filter, we moved it to `(-2.5, -2.5)`. Why?

**Answer:** the original y-coordinate `-3.0` was **exactly on the
keepout boundary line**. With a 1m keepout border and arena origin at
`(-4, -4)`, the allowed region is `(-3, -3)` to `(+3, +3)`. Spawning
with y=-3.0 puts the robot center on the boundary itself.

### Why that's a problem

1. **Floating-point + TF jitter.** The robot center might report as
   y=-3.001 (inside allowed) or y=-2.999 (also inside) on alternating
   ticks. Either way, part of the robot's **footprint** (radius 0.22m
   in scout_params) extends into the keepout region.
2. **Nav2 interprets that as a collision.** The keepout filter marks
   cells as lethal. If the robot footprint overlaps lethal cells,
   Nav2 thinks the robot has crashed into an obstacle and refuses to
   plan, entering recovery behaviors or failing goals.

### The fix

Move spawn to `(-2.5, -2.5)` — 0.5m from the nearest keepout edge in
both axes. That's comfortably more than `robot_radius (0.22) +
inflation_radius (0.20) ≈ 0.42m`, so the whole robot footprint + its
safety inflation lies inside the allowed region.

### Generalizable rule

When using a keepout filter, always spawn and plan goals with at least
`robot_radius + inflation_radius + safety_margin` from any keepout
edge. In our config that's ~0.5m. When we move to the real lab and
generate a new keepout mask, apply the same rule to the chosen
starting location.

---

## Q8 — Topic vs Service vs Action

**Context:** Our nodes use all three. Knowing which is which is
essential for reading code and making design choices.

### The three patterns

| | **Topic** | **Service** | **Action** |
|---|---|---|---|
| Pattern | pub/sub | request/response | request -> feedback* -> result |
| Direction | one-way | two-way, one-shot | two-way, long-running |
| Timing | continuous stream | ~instant (< 1s) | takes time (seconds to minutes) |
| Can cancel? | no | no | **yes** (client can preempt) |
| Feedback during? | no | no | **yes** (periodic updates) |
| Uses QoS profiles? | yes | limited | limited |

### When to pick each

- **Topic:** streaming data, status broadcasts, sensor feeds. Multiple
  subscribers are cheap. No guaranteed delivery (depends on QoS).
- **Service:** short RPC-style calls. "Please do X, tell me when done."
  Bad fit for anything that takes more than ~1 second.
- **Action:** goal-based tasks that take time and may fail or be
  cancelled. Think "navigate there", "follow this path", "pick up that
  object." The client gets periodic feedback and a final result.

### Examples in this project

| Thing | Type | Why |
|---|---|---|
| `/scan` (lidar) | Topic | continuous stream |
| `/map` | Topic (latched) | occupancy grid, published on update |
| `/cmd_vel` | Topic | continuous velocity commands |
| `/hazards/raw` / `/hazards/confirmed` | Topic | detection stream |
| `/mission_status` | Topic | status broadcast |
| `/request_package` (UR7 stub) | Service | RPC: "give me a package for category X" |
| `/navigate_to_pose` (Nav2) | Action | navigating takes time, need feedback + cancel |
| slam_toolbox `/save_map` | Service | one-shot "save map now" RPC |

### Code patterns (rclpy)

Topic:
```python
self.create_publisher(Twist, 'cmd_vel', 10)
self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
```

Service (client side):
```python
self.client = self.create_client(RequestPackage, '/request_package')
future = self.client.call_async(request)
# await future somehow
```

Action (client side):
```python
self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
send_future = self.nav_client.send_goal_async(goal)
# send_future.add_done_callback -> _on_goal_response
# then handle.get_result_async() -> _on_goal_result
```

Actions are the most complex because they're intrinsically async and
involve two futures (accept + result). That's why the auto_mapper
plumbing for `_send_nav_goal` / `_on_goal_response` / `_on_goal_result`
is non-trivial even though it "just sends a goal."

---

## Q9 — `map` vs `odom` frame relationship

**Context:** polygon coords live in `map` frame. What's the actual
TF hierarchy, and how does SLAM use it?

### Hierarchy (parent -> child, top-down)

```
map        (parent, published by slam_toolbox)
 -> odom        (child, published by odometry source)
     -> base_footprint
         -> base_link
             -> camera_link, laser_link, ...
```

Every TF frame has **exactly one parent**. `base_link` is already
parented to `odom` by the odometry publisher (gazebo_ros in sim,
turtlebot3_node on hardware). So SLAM has to insert `map` *above*
`odom`, not below.

### Who publishes what

- `odom -> base_link`: wheel encoders integrated. Smooth,
  high-frequency, drifts over time.
- `map -> odom`: slam_toolbox. Starts at identity. Updated when SLAM
  scan-matches or does loop closure. Jumps occasionally.

### Composition

`map -> base_link` = `(map -> odom)` then `(odom -> base_link)`.
When you `lookup_transform(map, base_link)`, TF walks both hops and
composes them. You get the SLAM-corrected pose directly, never have
to think about the odom hop.

### Mnemonic

> `map` is truth. `odom` is the lie. SLAM fixes the transform between
> them.

### Practical notes

- At startup, `map -> odom` is identity, so `map` coords ~= `odom`
  coords. They diverge over time as drift accumulates.
- Big loop closures can cause `map -> base_link` to **jump** by
  10-30 cm instantly. Nav2 handles this. RViz may look jittery.
- Always prefer `map` for anything that needs to be consistent over
  time (goals, hazard locations, bounds polygon).

---
