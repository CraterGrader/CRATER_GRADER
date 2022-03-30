# -------- Build on existing docker images ----------------
FROM p84514faf325/cratergrader:test-v1.0.0 as cg_base
# ---------------------------------------------------------

# -------- Install additional packages --------------------
RUN apt-get update && apt-get install -y \
  ros-$ROS_DISTRO-rviz2
# ---------------------------------------------------------
