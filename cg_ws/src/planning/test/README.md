# Test

Unit tests are run using `colcon test`. A config file can help visualize planner state using RVIZ.

Notes:
- All commands must be run inside the container
- All build steps should be run inside a `zsh` shell (e.g. container default shell)
- All execution/launch steps should be run inside a `bash` shell (e.g. `tmux`).

## Unit Tests
1. Build the package and dependencies.
```
colcon build --symlink-install --packages-up-to planning
```

2. Run the tests.
```
colcon test --packages-select planning --event-handlers console_cohesion+
```

## Visualization
1. Build the package and dependencies.
```
colcon build --symlink-install --packages-up-to planning
```

2. Open a VNC Viewer to port 5900.
- Ex. on laptop: `localhost:5900` 

3. Launch RVIZ with the config file.
```
rviz2 -d /root/CRATER_GRADER/cg_ws/src/cg_visualization/config/planning_debug.rviz
```

4. Launch the mapping subsystem.
```
ros2 launch mapping mapping_launch.py
```

5. Launch the planning subsystem.
```
ros2 launch planning planning_launch.py
```

## Resources
- Colcon test "how-to": https://colcon.readthedocs.io/en/released/user/how-to.html
