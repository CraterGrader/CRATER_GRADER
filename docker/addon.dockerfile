# -------- Build on existing docker images ----------------
FROM p84514faf325/cratergrader:main as cg_base
# ---------------------------------------------------------

# -------- Install additional packages --------------------
# Upgrade cmake; see https://askubuntu.com/questions/829310/how-to-upgrade-cmake-in-ubuntu
RUN wget https://github.com/Kitware/CMake/releases/download/v3.24.2/cmake-3.24.2-linux-x86_64.sh -P /opt/ \
  && chmod +x /opt/cmake-3.24.2-linux-x86_64.sh \
  && cd /opt/ \
  ## TODO
  # && bash /opt/cmake-3.24.2-linux-x86_64.sh \ 
  ##
  && sudo ln -s /opt/cmake-3.24.2-linux-x86_64/bin/* /usr/local/bin

# Install OR-Tools; see https://github.com/google/or-tools/blob/stable/cmake/README.md
RUN git clone https://github.com/google/or-tools.git \
  && cd or-tools && mkdir build \
  && cmake -S. -Bbuild -DBUILD_DEPS:BOOL=ON \
  && cmake --build build \
  && cd build/ && make install
# ---------------------------------------------------------
