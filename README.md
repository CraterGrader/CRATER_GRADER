# CRATER_GRADER

### Mac OS setup notes
- If using iCloud, clone this repository OUTSIDE the files tracked by iCloud. iCloud and git are both great forms of version control software on their own and when coupled can create issues such as duplicate files (e.g. see [stacked overflow post](https://stackoverflow.com/questions/59308049/icloud-drive-desktop-sync-vs-git-deleted-files-reappear-and-duplicates-with-n)) that can in some cases hinder software development. Both iCloud and git can be used together but must be separate. A good workflow for this is to clone the git repository to a location outside iCloud and either access the repository there directly OR use a symlink to store a pointer to the repository from anywhere in the file system (i.e. including from within an iCloud directory). Read more about [how to create symlinks on mac](https://www.switchingtomac.com/tutorials/osx/how-to-create-symlinks-on-your-mac/), and the following command for creating the symlink:
```
ln -s /path/to/cloned/repository /path/to/store/symlink/directory
```
A folder with the same name as the cloned repository will appear in the directory given in the symlink filepath. The folder in the symlink directory can then be accessed from anywhere in the file system to interact with the repository, now stored outside iCloud.

## Docker Description and Setup
The goal of docker is to streamline software development by providing a consistent environment/packages that is host machine agnostic. The following steps describe how to get up and running the the docker resources in this repository. For more extensive background on docker please consult the docker documentation and google for other questions you might have. 

### High-level Notes:
The `cg-dev` docker container is the primary container for development. This container:
- Is based on a [ROS2 Galactic image](https://hub.docker.com/_/ros), which runs on Ubuntu 20.04 (Focal Fossa).
- Can have new system packages (e.g. `apt-get ...`) added to `cg_dev.dockerfile`, ideally at the bottom of the file to keep re-build times shorter.
- Can have new python packages added to `environment.yml` (pip-specific packages should go at the very bottom, otherwise order doesn't matter).
- Uses volumes to sync file changes between the container and the host machine; any changes made will appear in both places, whether or not the container is active.
- Enables in-container access to remote Github repository by copying host machine ssh keys to the local container (only a local image) to allow commit/push/pull/etc. If you are using a different configuration (i.e. not an ssh key(s)), please access the remote Github via a separate terminal window outside the docker container.
- Starts each shell in a conda environment (specifically the `cg` conda environment, with packages as specified in the `environment.yml` file). You can double check the current and available conda environments by running `conda env list` from within the active container.
- Uses a `zsh` shell for convenient command line information (e.g. Github repo status), specifically with the [powerlevel10k theme](https://github.com/romkatv/powerlevel10k). The theme is customized for the container as specified by the `docker/.p10k.zsh` file
- Is initialized with vim editing customizations, located in the `docker/.vim/` directory and `docker/.vimrc` file.
- Has [tmux](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) for multiple terminal windows (note that the visualization sometimes seems buggy, to be resolved going forward).
- FOR GUI VISUALIZATION (e.g. Rviz, PlotJuggler, RQt, etc.)
  - Start gui appliation in the Docker container (e.g. `ros2 run rviz2 rviz2`, `ros2 run plotjuggler plotjuggler`, etc.)
  - Go to the following link in a browser: http://localhost:8080/vnc_auto.html
  
### Instructions
First, make sure Docker and Docker Compose are installed.
1. [Docker Desktop for Mac/Windows](https://docs.docker.com/desktop/) OR [Docker Engine for Linux](https://docs.docker.com/engine/install/#server)
2. [Docker Compose](https://docs.docker.com/compose/install/)
- Once complete, you can verify your installation version by running the following command:
  ```
  docker-compose --version
  ```

Then, use the following commands to create/activate the environment (after Docker is installed). Make sure to start somewhere in this repository (exact location doesn't matter). Note that `cg-dev` is one example of a docker-compose service to be run; in general the `docker-compose.yml` file can contain more than one service, and if so you may want to specify a different service for the following commands.

1. Start the Docker application, if it's not already running (it should already be running for Linux-based systems).

2. The first time you use the image, you need to build the image. Note that building only needs to be done if you want to update the image. This step will likely take the longest to run; at time of writing (12/24/2021) it takes ~5min on a Mid-2015 MacBook Pro. Subsequent builds will likely be much shorter because of docker's cache system.
```
docker-compose build
```

3. Docker does not have an automatic garbage collection, so every time we build an image more disk space will continue to be taken up by dangling images. To remove the dangling images, run the following command.
```
docker rmi $(docker images -f "dangling=true" -q)
```

4. Bring the image up in the background. It will be running, but we won't attach to it yet. If you'd like, you can check the result of this step by running `docker-compose ps` before and/or after the command to see the container status. You can also view the container status with the docker desktop app.
```
docker-compose up -d
```
5. Attach to a shell in the image. You will now be in the container.
```
docker-compose exec cg-dev zsh
```
- To exit the shell when you're done doing in the container, just type `exit` on the command prompt. The docker image will stay active in the background until you do step 6 (you can simply re-attach when you want, by running step 4 again after exiting)

6. You generally don't need to shut down the docker container, but if you won't be using it for a while and/or to save resources use while not using it you can use the following command. To re-start the container up again, simply begin with step 4 (i.e. no need to re-build unless dockerfile/etc. changes were made).
```
docker-compose down
```

### Other Notes/Tips
- To completely remove all docker related files from your system (e.g. to clear up resources), use `docker system prune -a --volumes`. Note that this will also clear the cache, so you will need to re-build the image afterwards. See the Docker documentation for more information about [pruning to reclaim space](https://docs.docker.com/config/pruning/), and [managing file system storage for Mac](https://docs.docker.com/desktop/mac/space/).
- You can check the status of containers using `docker-compose ps`, or with the desktop app.
- The `docker-compose.yml` file defines how the dockerfile(s) get called.
- If desired, you can omit the image name (e.g. `cg-dev`) with steps 1-2 and 4 to apply `docker-compose` commands to all services in `docker-compose.yml`. Step 3 still requires specification of an image name.
- The `entrypoint.sh` runs on startup, and shouldn't need to be edited except for rare instances.
