# Robot autonomy

This is my repo for the [Robot Autonomy course of DTU](https://kurser.dtu.dk/course/34761).

## Content

- A docker image to be able to run ROS using docker but without using VNC _(it's too laggy)_
- An implementation of an ICP to compute the Odometry of the robot using the lidar.
  You can start it by using the launch file `ros2 launch robot_autonomy_seb icp_standalone.launch.py`.

## Tests

To make it easier to test the implemented algorithms I _sometimes_ used unit tests. They can be
run using:

```sh
colcon test --packages-select robot_autonomy_seb --event-handlers console_direct+
```

## Usage

**This will not work on Windows, and probably not either of macOS**. This
is because it requires X11 forwarding to be able to display the apps directly on the host _(see Good to know)_.
This was only tested on Ubuntu 22.04.

```sh
# Clone this repo
git clone git@github.com:Seb-sti1/robot_autonomy.git --recursive

# Start the docker compose file
docker compose up -d
```

At this point you should have a konsole that popup. If you don't see anything,
try running `xhost +local:` and retry. If you do that, please run `xhost -local:` when you
are not working on the docker: this is for security reason.

_If after this, it still doesn't work please consider opening an issue._

### Good to know

The image use was build locally (maybe one day I will do the GitHub Action) using the `Dockerfile` in this repo
and is accessible on [Docker Hub](https://hub.docker.com/r/sebsti1/robot-autonomy).

To allow you to open GUI like `rviz2`, `gazebo` and so on the docker compose adds all the
parameters to forward the X11 session. This requires the `xhost` on the host to authorise
the connexion, and this should work just fine if the ros2 user (from the docker) has the same user id
as the user from your linux (run `id -u` on the host to know what is the id of your
user, the id of the ros2 user is `1000`). There could be a thousand reason why the id of
your user wouldn't be the same, so you can use `xhost +local:` to allow any local connexion.
As said earlier this is not the most robust configuration and I highly recommend to
run `xhost -local:` at the end of your coding session.

### Run without X11

If you want to test the package without using my docker image, you just need to make sure that
the `colcon_workspace` folder of this repo is inside the `src` _(or is the `src`)_ of your colcon
workspace of ROS2.

## Contribution

Any feedback & contribution are welcome.

## License

The software is provided "as is", without warranty of any kind.

See the LICENSE file.