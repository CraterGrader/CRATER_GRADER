# CRATER_GRADER
Complete source code for robot designed to autonomously grade lunar craters.


## Table of Contents
  1. [Docker Description and Setup](#docker-description-and-setup)
      - [Notice for iCloud users (MacOS):](#notice-for-icloud-users-macos)
  2. [High-Level Notes:](#high-level-notes)
  3. [Setup Instructions](#setup-instructions)
      - [GUI Visualization](#gui-visualization)
  4. [Other Notes/Tips](#other-notestips)

## Docker Description and Setup
The goal of docker is to streamline software development by providing a consistent environment/packages that is host machine agnostic. The following steps describe how to get up and running the the docker resources in this repository. For more extensive background on docker please consult the docker documentation and google for other questions you might have. 

#### Notice for iCloud users (MacOS):
> - If using iCloud, clone this repository OUTSIDE the files tracked by iCloud. iCloud and git are both great forms of version control software on their own and when coupled can create issues such as duplicate files (e.g. see [stack overflow post](https://stackoverflow.com/questions/59308049/icloud-drive-desktop-sync-vs-git-deleted-files-reappear-and-duplicates-with-n)) that can in some cases hinder software development. Both iCloud and git can be used together but must be separate. A good workflow for this is to clone the git repository to a location outside iCloud and either access the repository there directly OR use a symlink to store a pointer to the repository from anywhere in the file system (i.e. including from within an iCloud directory). Read more about [how to create symlinks on mac](https://www.switchingtomac.com/tutorials/osx/how-to-create-symlinks-on-your-mac/), and the following command for creating the symlink:
> ```
> ln -s /path/to/cloned/repository /path/to/store/symlink/directory
> ```
> A folder with the same name as the cloned repository will appear in the directory given in the symlink filepath. The folder in the symlink directory can then be accessed from anywhere in the file system to interact with the repository, now stored outside iCloud.

## High-Level Notes:
The `cg-dev` docker container is the primary container for development. This container:
- Is based on a [ROS2 Galactic image](https://hub.docker.com/_/ros), which runs on Ubuntu 20.04 (Focal Fossa).
- Can have new system packages (e.g. `apt-get ...`) added to `cg_dev.dockerfile`, ideally at the bottom of the file to keep re-build times shorter.
- Can have new python packages added to `environment.yml` (pip-specific packages should go at the very bottom, otherwise order doesn't matter).
- Uses volumes to sync file changes between the container and the host machine; any changes made will appear in both places, whether or not the container is active.
- Perform all git operations outside of the container (i.e. on local host OS, in separate terminal outside the container). While in-container git use is possible, compatibility can vary significantly depending on OS and architecture. For example, well-supported environments are Unix-based machines with `x86_64` architecture, but Windows machines can have problems copying git credentials to the container and `ARM` architectures can have problems with `openssh-client` configuration.
- Starts each shell in a conda environment (specifically the `cg` conda environment, with packages as specified in the `environment.yml` file). You can double check the current and available conda environments by running `conda env list` from within the active container.
- Uses a `zsh` shell for convenient command line information (e.g. Github repo status), specifically with the [powerlevel10k theme](https://github.com/romkatv/powerlevel10k). The theme is customized for the container as specified by the `docker/.p10k.zsh` file
- Is initialized with vim editing customizations, located in the `docker/.vim/` directory and `docker/.vimrc` file.
- Has [tmux](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) for multiple terminal windows (note that the visualization sometimes seems buggy, to be resolved going forward).
- This setup uses VNC to visualize GUI windows generated from the Docker container (e.g. Rviz, PlotJuggler, RQt, etc.). Note that the VNC client must be connected before GUI windows can be generated. The VNC client uses [fluxbox](https://wiki.debian.org/FluxBox) for rudimentary window management. To do so, follow the instructions below for [GUI Visualization](#gui-visualization)
  
## Setup Instructions
First, make sure Docker and Docker Compose are installed.
> 1. [Docker Desktop for Mac/Windows](https://docs.docker.com/desktop/) OR [Docker Engine for Linux](https://docs.docker.com/engine/install/#server)
> 2. [Docker Compose](https://docs.docker.com/compose/install/)
> - Once complete, you can verify your installation version by running the following command:
>  ```
>  docker-compose --version
>  ```
> Then, use the following commands to create/activate the environment (after Docker is installed). Make sure to start somewhere in this repository (exact location doesn't matter).

1. Windows/MacOS only: if not already running, start the Docker daemon by opening the Docker Desktop application (the Docker daemon should already be accessible for Linux-based systems).

2. The first time you use the image, you need to build the image. There are currently two images you can build, differing only on hardware access given to the resulting container. Specify the desired service in the following commands by replacing `cg-dev` with the desired service name. Both services use the same VNC port so can be viewed the same way, but cannot both be running at the same time (because the ports will conflict).
> - PERSONAL LAPTOP: `cg-dev` enables device-only access meaning the only devices that will be accessible by the container must be specified in the service or loaded with a separate script. This is the safest option and should be used for team laptop development.
> - XAVIER: `cg-dev-hw` is the full hardware access service meaning the container can access ALL hardware devices and drivers on the machine. This container uses `privileged=true` and mounts the full `/dev/` directory to the container. Full hardware access is less secure and inadvertent commmands could potentially cause significant harm to the host device, so this service should only be used for the robot hardware i.e. the Xavier computer.

Note that building only needs to be done if you want to update the image. This step will likely take the longest to run; typically 5-30 minutes. Subsequent builds will likely be much shorter because of docker's cache system.
  ```
  docker-compose build cg-dev
  ```

3. Docker does not have an automatic garbage collection, so every time we build an image more disk space may continue to be taken up by dangling images. To remove any dangling images, run the following command. For more information, see [What are Docker \<none\>:\<none\> images?](https://projectatomic.io/blog/2015/07/what-are-docker-none-none-images/)
  > Note that in some cases no dangling images are generated so you will see an output of `"docker rmi" requires at least 1 argument.`; this is expected.
  ```
  docker rmi $(docker images -f "dangling=true" -q)
  ```

4. Bring the image up in the background. It will be running, but we won't attach to it yet. If you'd like, you can check the result of this step by running `docker-compose ps` before and/or after the command to see the container status. You can also view the container status with the docker desktop app.
  ```
  docker-compose up -d cg-dev
  ```
  
5. In order to connect with all devices in the docker container without providing excessive privleges to the container, the udev rules for the devices must be symlinked to a custom directory. This can occur using the following script (Note: this script requires root access, eg. must be run with `sudo`)
  ```
  source set-udev-rules.sh
  ```

6. Attach to a shell in the image. You will now enter the container.
  ```
  docker-compose exec cg-dev zsh
  ```
> - To exit the shell when you're done doing in the container, just type `exit` on the command prompt. The docker image will stay active in the background until you do step 6 (you can simply re-attach when you want, by running step 4 again after exiting)

7. You generally don't need to shut down the docker container, but if you won't be using it for a while and/or to save resources use while not using it you can use the following command. Note that this command does not need a specified service; the command will bring all active services down. To re-start the container up again, simply begin with step 4 (i.e. no need to re-build unless dockerfile/etc. changes were made).
  ```
  docker-compose down
  ```

### GUI Visualization
  1) Enter the container (e.g. `docker-compose exec cg-dev zsh`)
  2) Open your VNC client application (e.g. download [VNC Viewer from RealVNC](https://www.realvnc.com/en/connect/download/viewer/))
  3) Connect to the VNC server address (search in the VNC client application for `localhost:5901`)
  4) When prompted, enter the VNC server password (for now, `cratergrader`)
  5) Start gui appliation in the Docker container, either in VNC window or original container terminal; best practice is to use the original terminal you entered the container from (e.g. `ros2 run rviz2 rviz2`, `ros2 run plotjuggler plotjuggler`, etc.)


### Running the CraterGrader worksystem

To run the `teleop` node, with joystick controller and drive output:

- `ros2 launch teleop joystick_launch.py`

To launch the UltraWideband beacons node:

- `ros2 launch uwb_beacon_rtls.launch.py`

To launch the IMU node:

- `ros2 launch imu_launch.py`

To launch the realsense:

- `ros2 launch realsense realsense_launch.py`

To launch robot_localization 

- `ros2 launch localization ekf_localization.launch.py`


## Other Notes/Tips
- To completely remove _all docker related files from your system_ (e.g. to clear up resources), use the following command. Only non-active containers will be removed (i.e. containers that are "down"; any containers that are "up" will not be affected). Note that this command will clear the Docker build cache, so you will need to re-build any removed images afterwards. See the Docker documentation for more information about [pruning to reclaim space](https://docs.docker.com/config/pruning/), and/or [managing file system storage for Mac](https://docs.docker.com/desktop/mac/space/).
  ```
  docker system prune -a --volumes
  ```
- You can check the status of containers using `docker-compose ps`, or with the desktop app.
- The `docker-compose.yml` file defines how the dockerfile(s) get called.
- If desired, you can add an additional arguement for the image name (e.g. `cg-dev`) with steps 2, 4, and 6 to apply `docker-compose` commands to only that service. Service must exist in `docker-compose.yml`. Step 5 still requires specification of an image name.
