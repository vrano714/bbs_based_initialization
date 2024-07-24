# ROS2 Localization Initializer based on 3D-BBS

3D-BBS is a point cloud maching localizer developed by Koki Aoki at [https://github.com/KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs).

This repo contains ROS2 code for replacing "2d pose estimate" in rviz with 3D-BBS.

## Installation

At first, install GPU version of 3D-BBS following the original repo [https://github.com/KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs).

Then clone this repository into your ROS2 workspace and build.

## Run

Write your config in `config/bbs_based_initializer.param.yaml` such as topic names, 3D-BBS params.

Then launch with `ros2 launch bbs_based_initializer bbs_based_initializer.launch.xml`.

The initializer subscribes map topic, lidar topic, imu topic, and trigger topic to run localization.
In the original implementation, map was loaded from local file, but here map is loaded from a topic (and it assumes the map topic has `.transient_local().reliable()` policy).

Sending `Bool` message to the trigger topic will run localization, after run, initializer will publish a `PoseWithCovarianceStamped` message, which is same type as rviz 2d pose estimate.

Example (to send from a command line):

```bash
ros2 topic pub -1 std_msgs/Bool /bbs_localize "{data: true}"
# note that /bbs_localize can be changed by editing config
```

## Notes and acknowledgements

The aim of creating this is to use this kind of location initializer with Autoware.universe instead of using GNSS.
Original intension is that running mobile robot without GNSS with Autoware is bothersome because one has to initialize the location.
If you assign a controller button to publish a message for this initializer, single button push will let the robot find its initial location.

I deeply thank Koki Aoki, original 3D-BBS developer, and the code.
Most of the codes here is lightly-modified version of the original implementation.
