# ROS1 Assignments Workspace

A ROS1 catkin workspace containing multiple homework/demo packages used in class. Each package is self-contained with launch files and minimal configuration.

Tested with **ROS Noetic** on Ubuntu 20.04.

## Contents

- **`tm`** — Turtlesim + RViz marker demo. Publishes a cube marker that tracks the turtle’s pose.
- **`p1_h2`** — Turtlesim swarm with simple collision-avoidance/wandering behavior.
- **`hw3`** — Path generation and obstacle visualization in RViz using parameters for start/goal and obstacle lists.
- **`hw4`** — Simple “altitude” control visualization driven by a simulated potentiometer signal.
- **`lecxacro`** — Minimal Xacro robot with two prismatic joints, visualized in RViz with joint controls.

---

## Quick Start

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
cd <your_workspace>
mkdir -p src && cd src
# put these packages under ./src (or unzip here)
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

> If you use zsh: `source devel/setup.zsh`

---

## Package Matrix

| Package    | Purpose                                                                 | Launch file                          | Key topics / services                                                                 |
|------------|-------------------------------------------------------------------------|--------------------------------------|----------------------------------------------------------------------------------------|
| `tm`       | Turtlesim + RViz marker that follows `/turtle1/pose`.                   | `tm/launch/launch.launch`            | Sub: `/turtle1/pose` (turtlesim), Pub: `/visualization_marker` (Marker)               |
| `p1_h2`    | Multi-turtle swarm; random motion with simple collision avoidance.      | `p1_h2/launch/swarm.launch`          | Srv: `/spawn`, `/kill` (turtlesim), Sub: `/<name>/pose`, Pub: `/<name>/cmd_vel`       |
| `hw3`      | Generate a smooth path and visualize start/goal/obstacles in RViz.      | `hw3/launch/follow_path.launch`      | Pub: `obstacles` (MarkerArray), `robot_path` (Path), `robot_pose` (Marker)            |
| `hw4`      | Altitude visualization with a sine-wave “potentiometer” input.          | `hw4/launch/launch.launch`           | Pub: `/pot` (Float32), `/altitude` (Float32MultiArray), `/create_sphere` (Marker)     |
| `lecxacro` | Xacro robot (base + 2 prismatic links) with joint GUI + RViz.           | `lecxacro/launch/robot.launch`       | Uses `robot_state_publisher`, `joint_state_publisher_gui`, RViz                        |

---

## How to Run

### `tm` — Turtlesim + Marker
```bash
roslaunch tm launch.launch
```
- This starts `turtlesim_node`, keyboard teleop, a marker node, and RViz.
- Move the turtle with the terminal focus on the teleop window; a cube marker in RViz tracks its pose.

### `p1_h2` — Swarm Avoid Collision
```bash
roslaunch p1_h2 swarm.launch
```
Default parameters (see `p1_h2/config/params.yaml`):
- `names`: `['T1','T2']`
- `initial_x`: `[1, 1]`
- `initial_y`: `[1, 9]`
- `initial_th`: not in the YAML; node uses `~initial_th` with default `[0.0, 0.0, ...]`

The node reads **private** params (e.g., `~initial_x`). To set them explicitly under the `swarm` node namespace:
```bash
rosparam set /swarm/names "['T1','T2','T3']"
rosparam set /swarm/initial_x "[2,5,8]"
rosparam set /swarm/initial_y "[2,5,8]"
rosparam set /swarm/initial_th "[0,0,0]"
roslaunch p1_h2 swarm.launch
```

### `hw3` — Path & Obstacles Visualization
```bash
roslaunch hw3 follow_path.launch
```
Parameters (`hw3/config/params.yaml`):
- `obstacles_x`, `obstacles_y`: integer lists of obstacle positions
- `starting_pose`: `[x, y]`
- `goal_pose`: `[x, y]`

Override on the command line if needed:
```bash
roslaunch hw3 follow_path.launch starting_pose:="[3, -7]" goal_pose:="[-5, 10]"
```

You should see:
- `obstacles` (MarkerArray) for all obstacles (+ markers for start/goal)
- `robot_path` (nav_msgs/Path) published latched for RViz display
- `robot_pose` (Marker) stepping through the generated/smoothed path

### `hw4` — Altitude Control + Potentiometer Simulator
```bash
roslaunch hw4 launch.launch
```
- `pot_sim.py` publishes `/pot` as a sine wave between `0..1023`.
- `Control.py` maps `/pot` to a Z value and publishes:
  - `/altitude` (Float32MultiArray): `[z_value, tolerance]`
  - `/create_sphere` (Marker): sphere at current altitude

Parameters (`hw4/config/params.yaml`):
- `zmin`, `zmax`: clamp of altitude range
- `tolerance`: visualization/logic tolerance

Optional runtime override for the pot cycle time:
```bash
rosparam set /pot_sim/cycle_time 20.0
roslaunch hw4 launch.launch
```

### `lecxacro` — Xacro Robot + RViz + Joint GUI
```bash
roslaunch lecxacro robot.launch
```
- Loads `urdf/robot.xacro` (which includes `urdf/variables.xacro`)
- Starts `robot_state_publisher`, `joint_state_publisher_gui` and RViz
- Move sliders for joints `j1` and `j2` to see prismatic motion in RViz

Adjust robot dimensions in `variables.xacro`:
- `base_length`, `base_radius`, `link1_length`, `link1_radius`, `link2_length`, `link2_radius`

---

## Verification Snippets

Run these in separate terminals after launching.

**`tm`**
```bash
rostopic echo -n1 /visualization_marker
```

**`p1_h2`**
```bash
rosservice list | grep -E '/(spawn|kill)$'
rostopic list | grep cmd_vel
```

**`hw3`**
```bash
rostopic echo -n1 /robot_path
rostopic echo -n1 /obstacles
```

**`hw4`**
```bash
rostopic echo -n1 /pot
rostopic echo -n1 /altitude
rostopic echo -n1 /create_sphere
```

**`lecxacro`**
```bash
rostopic list | grep joint_states
```

---

## Parameters Reference

- **`tm`**
  - none required; uses turtlesim defaults; marker is centered at `(5.5, 5.5)` and offsets turtle pose.

- **`p1_h2`** (read as **private** to node `swarm`)
  - `~names`: list of turtle names
  - `~initial_x`, `~initial_y`: lists of initial positions
  - `~initial_th`: list of initial headings (defaults to zeros if omitted)

- **`hw3`** (global)
  - `obstacles_x`, `obstacles_y`: obstacle coordinates
  - `starting_pose`, `goal_pose`: start/goal points

- **`hw4`** (global)
  - `zmin`, `zmax`, `tolerance`: altitude mapping/visualization
  - `~cycle_time` (private to `pot_sim`): period of sine wave in seconds

---

## Workspace Layout (abridged)

```
src/
  CMakeLists.txt
  tm/
    launch/launch.launch
    rviz/marker.rviz
    src/marker.py
    src/trace_turtle.py
  p1_h2/
    config/params.yaml
    launch/swarm.launch
    src/swarm_avoid_collision.py
  hw3/
    config/params.yaml
    config/rviz.rviz
    launch/follow_path.launch
    src/get_path.py
    src/obstacles.py
  hw4/
    config/params.yaml
    config/rviz.rviz
    launch/launch.launch
    src/Control.py
    src/pot_sim.py
  lecxacro/
    launch/robot.launch
    rviz/robot.rviz
    urdf/robot.xacro
    urdf/variables.xacro
```

---

## Notes & Good Practices

- Use `rosdep install --from-paths src --ignore-src -r -y` before building to fetch dependencies like `turtlesim`, `rviz`, `xacro`, `robot_state_publisher`, and message types.
- Source your workspace before running anything: `source devel/setup.bash` (or add it to your shell profile).
- Prefer `roslaunch` files provided here; they load parameters and start RViz/configured nodes together.
- Keep private parameters (`~param`) under the node namespace when setting them manually with `rosparam`.


