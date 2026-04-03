# Change Log: main.py (V1: cav_sumo → V2: cav_sumo_github)

---

## 1. Per-Vehicle CAV Controller

**Problem in V1:** A single `self.ego = CAV()` instance is shared across all urban CAVs, so per-vehicle controller flags (e.g. stop-line state, sign state) leak across the entire fleet.

**Why this causes a bug:** The CAV controller maintains internal state across timesteps — things like "am I in deceleration mode?", "what was my speed last frame?", "have I already committed to stopping at this red?". With a single shared object, calling `getCommand()` for vehicle A writes A's state onto that shared object. When `getCommand()` is then called for vehicle B in the same step, B reads A's leftover state as if it were its own history. Crucially, simply clearing the state between vehicles is not a fix — each vehicle *needs* its history to persist across timesteps (e.g. vehicle A entering deceleration mode at t=10.0s must still be in that mode at t=10.1s). The only correct solution is separate state per vehicle.

**V1:**
```python
if self.args.penetration > 0 or True:
    # Automatically pick the virtual ego CAV controller
    if self.is_traffic_light_scenario():
        self.ego = CAV()   # one shared instance for ALL CAVs
    else:
        self.ego = PCC()
```

**V2:** Each CAV gets its own Python `CAV` object. All instances share one native C API handle to avoid repeated initialization overhead. A new `get_controller_for_vehicle()` method manages creation and caching.

```python
# __init__
if self.args.penetration > 0 or True:
    if self.is_traffic_light_scenario():
        self.shared_cav_api = None   # shared C API handle
        # self.cav_controllers = {} per-vehicle CAV objects (set in __init__)
    else:
        self.ego = PCC()
```

```python
def get_controller_for_vehicle(self, ego_id: str):
    if not self.is_traffic_light_scenario():
        return self.ego

    controller = self.cav_controllers.get(ego_id)
    if controller is not None:
        return controller

    if self.shared_cav_api is None:
        controller = CAV()
        self.shared_cav_api = controller.api   # save handle for reuse
    else:
        controller = CAV(api=self.shared_cav_api)  # reuse existing handle

    self.cav_controllers[ego_id] = controller
    return controller
```

---

## 2. Two-Stage Control Loop

**Problem in V1:** The single-pass loop computes and immediately applies commands (including V2V comms updates) within the same iteration. A comms update for vehicle A can be visible to vehicle B's sensor read in the same step, breaking synchronous simulation semantics.

**Why this causes a bug:** The simulation runs at 0.1 s steps. Vehicle B should make its control decision based on what vehicle A *broadcast at the previous step* — that is the physically correct model of wireless V2V communication latency. In V1, because A's comms are published mid-loop, B reads A's freshly computed trajectory from the *current* step, effectively giving B a look-ahead into A's future that would never exist in a real system. This produces artificially tight or unrealistic following behavior at high penetration rates.

**V1** (single pass — compute, apply, and publish comms all at once):
```python
type_gen = (ego_id for ego_id, vtype in self.spawned_vehs.items() if vtype == 'cav')
for ego_id in type_gen:
    # ... read sensors ...
    desired_acceleration, time_traj, pos_traj, vel_traj, acc_traj = self.ego.getCommand(
        ..., NextTLS=NextTLS, SPATmsg=SPATmsg, TLInfo=self.TLInfo)

    self.comms.update_veh_comms(ego_id, time_traj, pos_traj, vel_traj)  # <-- published mid-loop

    # ... set acceleration, debug ...
    traci.vehicle.setAcceleration(ego_id, desired_acceleration, self.acc_duration)
```

**V2** (two stages — snapshot all, then apply all):
```python
# Stage 1: snapshot inputs + compute commands for all CAVs
control_plan = {}
comm_updates = {}
cav_ids = sorted(ego_id for ego_id, vtype in self.spawned_vehs.items()
                 if vtype == 'cav' and ego_id in vehicle_id_set)

for ego_id in cav_ids:
    # ... read sensors ...
    controller = self.get_controller_for_vehicle(ego_id)
    desired_acceleration, time_traj, pos_traj, vel_traj, acc_traj = controller.getCommand(
        ..., NextTLS=next_tls, SPATmsg=step_spat, TLInfo=self.TLInfo)

    control_plan[ego_id] = {
        "desired_acceleration": desired_acceleration,
        "next_tls": next_tls,
        "controller": controller,
    }
    comm_updates[ego_id] = (time_traj, pos_traj, vel_traj)

# Stage 2: apply commands and publish comms
for ego_id in cav_ids:
    plan = control_plan.get(ego_id)
    traci.vehicle.setAcceleration(ego_id, plan["desired_acceleration"], self.acc_duration)
    self.debug(ego_id=ego_id, controller=plan["controller"])
    # ... stuck rescue check ...

# Publish V2V comms only after all commands applied
for ego_id, (time_traj, pos_traj, vel_traj) in comm_updates.items():
    self.comms.update_veh_comms(ego_id, time_traj, pos_traj, vel_traj)
```

---

## 3. Route Lane Guidance (Traffic-Light Scenarios)

**Problem in V1:** No mechanism to guide CAVs into the correct lane before a signalized intersection. CAVs could reach the stop line in a turn-only pocket with no valid forward connection, causing a permanent deadlock (observed at the Clinton intersection — edge `435834768#0`, only 9.95 m long, where lane 0 is right-turn-only).

**Why this causes a bug:** SUMO's default lane-change logic sometimes assigns a CAV to a short turn-pocket lane too late for a lane change to physically complete. Once the vehicle is within a few meters of the stop line, there is no room to execute the change — SUMO will not force a teleport because TraCI speed control is active, and the controller keeps issuing acceleration commands that hold the vehicle in place. The result is a permanent deadlock: the CAV cannot move forward (its route has no valid connection from this lane), and the vehicles queued behind it are also blocked, freezing the entire simulation.

**V2** adds three new methods:

**`get_route_aligned_lane_target()`** — within 400 m of a TLS, checks SUMO's `getBestLanes()` data and returns a target lane if the current lane does not permit route continuation:
```python
def get_route_aligned_lane_target(self, ego_id, next_tls, approach_distance=400.0):
    # Returns None if no guidance needed, or a dict with target lane info
    best_lanes = traci.vehicle.getBestLanes(ego_id)
    valid_lanes = [lane for lane in best_lanes if lane[4]]  # lane[4] = allows continuation
    target_info = min(valid_lanes, key=lambda lane: (abs(lane[3]), ...))
    ...
    return {"target_lane_id": ..., "lane_delta": ..., "remaining_distance": ..., ...}
```

**`align_vehicle_to_route_lane()`** — throttled to once per second, nudges the CAV one lane step at a time using `changeLaneRelative`:
```python
def align_vehicle_to_route_lane(self, ego_id, next_tls):
    lane_target = self.get_route_aligned_lane_target(ego_id, next_tls)
    if lane_target is None:
        return
    lane_step = 1 if lane_target["lane_delta"] > 0 else -1
    traci.vehicle.changeLaneRelative(ego_id, lane_step, 2.0)
    self.last_route_lane_guidance[ego_id] = self.sim_time
```

**`rescue_wrong_lane_at_stopline()`** — when the CAV is already within 15 m of the stop line in a dead-end lane, uses `moveTo` to teleport it laterally to the nearest route-valid lane before the controller runs:
```python
def rescue_wrong_lane_at_stopline(self, ego_id):
    # Only acts when within 15 m of stop line AND lane has no forward connection
    if (lane_length - lane_pos) > 15.0:
        return False
    if current_info[4]:      # already on a valid lane
        return False
    target_lane = min(valid_on_edge, key=lambda l: abs(l[3]))
    traci.vehicle.moveTo(ego_id, target_lane_id, safe_pos)
    return True
```

Called in the apply stage, before `setAcceleration`:
```python
if self.is_traffic_light_scenario() and not self.args.replay:
    self.rescue_wrong_lane_at_stopline(ego_id)   # in Stage 1 (before getCommand)
    ...
    self.align_vehicle_to_route_lane(ego_id, plan["next_tls"])  # in Stage 2
```

---

## 4. Stuck-at-Stop-Line Rescue (Time-to-Teleport + Active Detection)

### 4a. `--time-to-teleport` default

**V1:** The SUMO `--time-to-teleport` flag was never set, so SUMO used its own default (which may be too long or disabled entirely for urban scenarios).

**Why this causes a bug:** SUMO has a built-in recovery mechanism — any vehicle stuck for longer than `time-to-teleport` seconds is automatically teleported away. However, TraCI's `setAcceleration()` command overrides SUMO's internal speed model, causing SUMO to treat the vehicle as being actively controlled rather than stuck. As a result, SUMO's stuck-timer never increments, and the teleport never fires regardless of how long the vehicle sits motionless.

**V2:** A smart default is applied at init:
```python
# V1: flag not present in SUMO_CMD at all

# V2:
if self.args.time_to_teleport is None:
    # 60 s is short enough to trigger within a single green phase (~40-60 s),
    # teleporting any CAV whose ACC controller fails to resume after a red stop.
    self.time_to_teleport = 60.0 if self.args.scenario in TRAFFIC_LIGHT_SCENARIOS else -1.0
else:
    self.time_to_teleport = float(self.args.time_to_teleport)

# Added to SUMO_CMD:
'--time-to-teleport', f'{self.time_to_teleport:g}',
```

### 4b. Active stuck detection

**V1:** No detection — a CAV stuck at a red stop under TraCI speed control would remain frozen indefinitely (SUMO's teleport cannot fire because TraCI overrides the speed).

**V2:** After applying the acceleration command each step, checks if the CAV has been stationary at the stop line for >10 s. If so, releases TraCI control and suspends the vehicle for 150 s so SUMO's own model can recover it:
```python
actual_speed = traci.vehicle.getSpeed(ego_id)
at_stop_line = (rescue_lane_len - rescue_lane_pos) < 1.5

if at_stop_line and actual_speed < 0.05:
    self.cav_stuck_ticks[ego_id] = self.cav_stuck_ticks.get(ego_id, 0) + 1
else:
    self.cav_stuck_ticks.pop(ego_id, None)

# 100 ticks × 0.1 s/tick = 10 s → suspend
if self.cav_stuck_ticks.get(ego_id, 0) >= 100:
    traci.vehicle.setSpeed(ego_id, -1)            # release TraCI speed override
    traci.vehicle.setSpeedMode(ego_id, 31)        # restore all SUMO safety checks
    self.has_been_set_vehs.discard(ego_id)
    self.cav_stuck_ticks.pop(ego_id, None)
    self.cav_suspended_until[ego_id] = self.sim_time + 150.0  # SUMO drives for 150 s
```

---

## 5. Runtime Summary Logging

**V1:** Prints only the current simulation time every 10 s, giving no insight into performance or fleet composition.

**Why this matters in practice:** In batch runs (e.g. sweeping 20 penetration rates), a simulation that deadlocks will keep printing `t: 300.0`, `t: 310.0`... indefinitely and is indistinguishable from a healthy run. The new summary line includes real-time factor (RTF) and `minExpected`, so a stuck simulation is immediately visible — RTF drops toward zero and `minExpected` stops changing.

```python
# V1
if abs(fmod(self.sim_time, 10)) < 1e-6:
    print('t: {:0.1f}'.format(self.sim_time))
```

**V2:** Replaced with `print_runtime_summary()`, printing a single structured line with real-time factor, vehicle counts by type, and throughput:

```python
# V2
if self.sim_time + 1e-9 >= self.next_summary_time:
    self.print_runtime_summary()
    self.next_summary_time += self.summary_interval  # default 10 s
```

```python
def print_runtime_summary(self):
    wall_elapsed = counter() - self.wall_start_time
    rtf = self.sim_time / max(wall_elapsed, 1e-9)
    print(
        f"[SIM] t={self.sim_time:6.1f}s | wall={wall_elapsed:6.1f}s | RTF={rtf:5.3f} | "
        f"minExpected={min_expected:4d} | inNet={in_net:4d} | cav={cav_in_net:4d} | "
        f"hdv={hdv_in_net:4d} | loaded={loaded:3d} | departed={departed:3d} | arrived={arrived:3d}"
    )
```

Example output:
```
[SIM] t= 120.0s | wall=  45.2s | RTF=2.655 | minExpected= 142 | inNet=  87 | cav=  14 | hdv=  73 | loaded=  5 | departed=  3 | arrived=  2
```

Also, `is_running()` now supports an optional `--max_sim_time` hard cutoff:

```python
# V1
return traci.simulation.getMinExpectedNumber() > lower_bound

# V2
max_t = getattr(self.args, 'max_sim_time', 0) or 0
if max_t > 0 and self.sim_time >= max_t:
    print(f"[SIM] max_sim_time={max_t:g}s reached at t={self.sim_time:.1f}s — terminating.")
    return False
return traci.simulation.getMinExpectedNumber() > lower_bound
```
