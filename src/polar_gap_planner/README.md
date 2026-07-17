# polar_gap_planner

360-degree polar gap global planner for CMU autonomous exploration stack.
It searches the widest continuous obstacle-free sector from `/terrain_map` (or
`/registered_scan`), publishes sub-goals to `/way_point`, and keeps the existing
`local_planner` + `pathFollower` unchanged.

## Build (ROS Noetic)

```bash
cd ~/autonomous_exploration_development_environment
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run with existing simulation

Terminal 1: start your Gazebo system as usual, e.g.

```bash
roslaunch vehicle_simulator system_garage.launch
```

Terminal 2: start polar gap + local planner (if not already running)

```bash
roslaunch polar_gap_planner polar_gap_with_local_planner.launch
```

Or only the global planner node if `local_planner` is already launched:

```bash
roslaunch polar_gap_planner polar_gap_planner.launch
```

## Publish final goal

Use RViz **2D Nav Goal** after remapping, or command line:

```bash
rostopic pub /final_goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position: {x: 10.0, y: 5.0, z: 0.75}
  orientation: {w: 1.0}" -1
```

Alternative topic:

```bash
rostopic pub /final_goal_point geometry_msgs/PointStamped "header:
  frame_id: 'map'
point: {x: 10.0, y: 5.0, z: 0.75}" -1
```

**Important:** Do not use the built-in RViz Waypoint tool for final goals while
this node is running, because it also publishes `/way_point`. Use `/final_goal`
instead.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/terrain_map` | `sensor_msgs/PointCloud2` | subscribe (default) |
| `/registered_scan` | `sensor_msgs/PointCloud2` | subscribe (fallback) |
| `/state_estimation` | `nav_msgs/Odometry` | subscribe |
| `/final_goal` | `geometry_msgs/PoseStamped` | subscribe |
| `/final_goal_point` | `geometry_msgs/PointStamped` | subscribe |
| `/way_point` | `geometry_msgs/PointStamped` | publish |
| `/speed` | `std_msgs/Float32` | publish |
| `/planner_state` | `std_msgs/String` | publish (`ESCAPE`/`NAVIGATE`/`DONE`/`STUCK`) |
| `/polar_gap_markers` | `visualization_msgs/MarkerArray` | publish |

## RViz visualization

Add `MarkerArray` on `/polar_gap_markers`:

- Green rays: free directions
- Red rays: blocked directions
- Blue sector: selected gap

## State machine

1. **ESCAPE** – goal direction blocked; follow widest 360° gap
2. **NAVIGATE** – path toward goal is clear; sub-goal along goal bearing
3. **DONE** – within `goal_tolerance` of final goal
4. **STUCK** – no gap wider than `min_gap_width_deg`

## Integration into system launch

Add before or after `local_planner` in your `system_xxx.launch`:

```xml
<include file="$(find polar_gap_planner)/launch/polar_gap_planner.launch"/>
```

And tune local planner:

```xml
<param name="dirThre" type="double" value="180.0"/>
<param name="dirWeight" type="double" value="0.005"/>
<param name="autonomySpeed" type="double" value="1.0"/>
```

## Parameters

See `config/default.yaml`. Key params:

- `min_free_distance`: minimum ray length to treat a direction as free (m)
- `min_gap_width_deg`: minimum acceptable gap width (deg)
- `navigate_free_dist`: switch to NAVIGATE when goal direction is clear this far
- `escape_free_dist`: switch back to ESCAPE when clearance drops below this
- `use_terrain_map`: `true` for `/terrain_map`, `false` for `/registered_scan`

## Test checklist (U-trap)

1. Launch `roslaunch vehicle_simulator system_campus.launch`
2. In RViz use **Waypoint** tool (W) to click **final goal** (purple sphere on `/final_goal_point`)
3. Check `/planner_state` is `ESCAPE` inside U-trap
4. Green **SubGoal** (`/way_point`) should appear toward the U opening, not through the wall
5. `/polar_gap_markers` blue sector should point to the widest gap
6. After exiting, state becomes `NAVIGATE` and vehicle heads toward purple goal
