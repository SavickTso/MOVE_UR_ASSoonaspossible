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


I wish my life can be this easy.