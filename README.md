# Autonomosu Pylon Racing Competition Official Scoring Metrics
This package contains RViz2-based pylon course and lap time scoring for fixed-wing UAS racing.

## Run
```bash
colcon build --symlink-install --packages-select pylon_2026_world
. install/setup.bash
ros2 run pylon_2026_world course.launch.py
