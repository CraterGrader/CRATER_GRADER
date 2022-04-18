# -------- Build on existing docker images ----------------
FROM p84514faf325/cratergrader:main as cg_base
# ---------------------------------------------------------

# -------- Install additional packages --------------------
RUN apt-get update && apt-get install -y \
  # ros-galactic-tf2-tools \
  # ros-galactic-tf-transformations \
  # Health monitoring gui
  ros-$ROS_DISTRO-rosbridge-suite

RUN conda install netifaces && pip install pymongo
# ---------------------------------------------------------
