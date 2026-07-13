# cv_robot

Differential drive robot navigation stack built in ROS2 Jazzy + Gazebo Harmonic, using an overhead camera and ArUco markers for localization instead of ground-truth odometry.


## Files

- Coupled PID controller for waypoint navigation (`diff_PID.py`)
- Sequential waypoint publisher (`seq_diff_target.py`)
- Single-target publisher for testing (`diff_target.py`)
- Evaluation node (`eval_diff.py`)
- ArUco marker-based localization via overhead camera
- (`robot_arm.urdf`, `robot_arm1.urdf`)
- Gazebo world + overhead camera SDF setup

## Adding the ArUco marker (Gazebo Harmonic)

This one took some trial and error, so documenting it properly. The obvious approach — generate a marker PNG, drop it in a texture folder, point the URDF material at it, does **not** work in Gazebo Harmonic. It just renders as a plain white square. Harmonic doesn't handle simple material/texture tags the way older Gazebo versions did.

- The working method is to define the marker as a hand-written `.obj` mesh with a paired `.mtl` file that references the PNG.
- An `.obj` file explicitly lists texture coordinates (`vt` lines) per vertex and pairs them to the geometry in its face definitions
- All three files live together in `materials/meshes/` - the `.obj`, the `.mtl`, and the marker PNG , because `.mtl` texture paths resolve relative to the mesh file's own folder

The `.mtl` is just a couple of lines pointing at the texture:

```
newmtl aruco_material
map_Kd aruco_marker_5x5_250_ID1.png
```

And the URDF references the `.obj` mesh like any other visual (no `<gazebo><material>` block needed — the `.mtl` does that job):

```xml
<link name="aruco_marker_link">
  <visual>
    <geometry>
      <mesh filename="package://cv_robot/materials/meshes/aruco_marker.obj" scale="1 1 1"/>
    </geometry>
  </visual>
</link>
```

Marker config used throughout:

- Dictionary: `DICT_5X5_250`
- Marker ID: 1
- Physical size: 1.0m × 1.0m
- Source image: `aruco_marker_5x5_250_ID1.png`

## How the navigation works

- A static overhead camera (spawned looking straight down) detects an ArUco marker mounted on top of the robot
- Because the camera is fixed and watching a marker on the robot, marker pose gives direct global position,no fusion with wheel odometry needed
- Marker pose gets transformed into world-frame x/y/theta via a quaternion offset rotation (the camera's optical frame isn't aligned with the world frame, hence the `-0.7071, 0.7071, 0, 0` offset applied in `odom_callback`)
- PID controller computes linear + angular velocity commands from distance error and heading error
- Linear velocity gets scaled by `cos(angle_error)` so the robot doesn't drive full speed while still turning to face the target — this was a fix for the robot cutting corners badly during heading correction
- Waypoints get published in sequence; each target switch resets nothing right now (known issue, see below)

## Known issues

- PID integral/derivative state doesn't reset on target switch — can cause a rough start on the next leg if the previous leg ended with residual error buildup
- Timer period is currently hardcoded to roughly match average camera framerate (~28Hz) as a temporary fix
- Camera framerate itself jitters a fair amount (measured 7–36Hz spread via `ros2 topic hz`), so localization updates aren't perfectly steady
- Noisy data and further smoothing can be done
## Requirements

- ROS2 Jazzy
- Gazebo Harmonic
- `ros2_aruco` package
- Python: `scipy`, `numpy`, `matplotlib`

## Running it

```bash
ros2 launch cv_robot cv_robot.launch.py
```

Then publish targets via the sequencer node, or manually to `/target` as a `Float64MultiArray` of `[x, y]`.

## Repo structure

```
cv_robot/
├── scripts/          # PID controller, target publishers, eval node
├── urdf/              # robot + camera descriptions
├── worlds/            # Gazebo world
├── materials/meshes/  # STL meshes for arm links + markers
└── launch/
```
