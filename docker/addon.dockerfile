# -------- Build on existing docker images ----------------
FROM p84514faf325/cratergrader:main as cg_base
# ---------------------------------------------------------

# -------- Install additional packages --------------------
# RUN apt-get update && apt-get install -y \
  RUN apt-get update && apt-get install ros-galactic-tf2-tools ros-galactic-tf-transformations
# ---------------------------------------------------------
