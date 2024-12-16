# MOVE_UR_ASSoonaspossible
Off-shelf solution to save my poor little brain severely damaged by UR ROS2 driver and its doc.

Thanks to `hello_moveit_ur` package from [link](https://github.com/LucaBross/simple_moveit2_universal_robots_movement).

## Installation
Build the docker image

```bash
cd ./docker && docker build -t savicktso/ur_ros2_2024:humble .
```

stay in `docker` directory.

Then run these commands in separate terminals.

```bash
# start driver (you may set use_fake_hardware to false)
docker compose run --rm ur_ros2_base
```

```bash
# start moveit2
docker compose run --rm ur_ros2_moveit
```

```bash
# move the robot in Cartersian path. (move tcp 10cm upper in z axis)
docker compose run --rm ur_ros2_cartesianmove
```

**Notice that I specify robot model as UR10e, make sure to search the codebase and replace them with yours.**

### Use pilz planner 

Commands above are using default OMPL planner, to use the pilz planner's PTP planning, try:

```bash
docker compose run --rm ur_ros2_base
```

```bash
docker compose run --rm ur_ros2_moveit_pilz
```

```bash
# move the robot in Cartersian path. (move tcp 10cm upper in z axis)
docker compose run --rm ur_ros2_cartesianmove_pilz
```
###
Inside the bash container:
```bash
docker compose run --rm ur_ros2_bash
```

After the compilation, you can modify the scale factors and send cartesian space motion commands, for example:
```bash
ros2 launch hello_moveit_ur pilz_moveit_ur_launch.py coordinates_z:=-0.1 vel_scale:=0.5 acc_scale:=0.5
```