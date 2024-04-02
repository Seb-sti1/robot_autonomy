# Robot autonomy

This is my repo for the [Robot Autonomy course of DTU](https://kurser.dtu.dk/course/34761).

For now:

- I added a docker image to be able to run ROS using docker but without using VNC _(it's too laggy)_
- My _terrible_ algorithm trying to do something similar to ICP.

## Usage

**This will not work on Windows, and might not work either of macOS**.
This was only tested on Ubuntu 22.04.

```sh
# Clone this repo
git clone https://github.com/Seb-sti1/robot_autonomy.git

# Start the docker compose file
docker compose up -d
```

At this point you should have a konsole that popup. If you don't see anything,
try running `xhost +local:` and retry. If you do that, please run `xhost -local:` when you
are not working on the docker: this is for security reason.

_If after this, it still doesn't work please consider opening an issue._

### Good to know

The image use was build locally (maybe one day I will do the GitHub Action) and is accessible
on [Docker Hub](https://hub.docker.com/r/sebsti1/robot-autonomy). The file to build it
is the Dockerfile in the repo.

To allow you to open GUI like `rviz2`, `gazebo` and so on the docker compose adds all the
parameters to forward the X11 session. This requires the `xhost` on the host to authorise
the connexion, and this should work just fine if the ros2 user (from the docker) has the same user id
as the user from your linux (run `id -u` on the host to know what is the id of your
user, the id of the ros2 user is `1000`). There could a thousand reason why the id of
your user wouldn't be the same, so you can use `xhost +local:` to allow any local connexion.
As said earlier this is not the most robust configuration and I highly recommend to
run `xhost -local:` at the end of your coding session.

## Contribution

Any feedback/contribution is welcome.

## License

The software is provided "as is", without warranty of any kind.

See the LICENSE file.