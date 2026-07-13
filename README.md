# coupled-pid branch
## Nodes

**pid_controller** — reads pose from Gazebo, publishes cmd_vel. Uses cos(angle_error) coupling on linear velocity so the robot slows down when misaligned before driving forward.
also has both angluar and linear PID.

**seq_diff_target** — publishes waypoints sequentially. Sends the next target only after the robot arrives within 0.15m of the current one.

**eval_diff** — metrics node. Tracks path length, rotation, and time per leg. Saves a 2x2 matplotlib plot on exit showing distance/angle/position vs time and trajectory.

## Waypoints
(5,5) → (5,-5) → (-5,-5) → (-5,5) → (0,0) → (-7,12)

## Run order
1. Launch Gazebo with robot
2. pid_controller
3. seq_diff_target
4. eval_diff
