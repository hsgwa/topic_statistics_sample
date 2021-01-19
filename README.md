# Topic Statistics Sample


## Setup
```
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
$ git clone https://github.com/hsgwa/topic_statistics_sample.git src
$ colcon build --symlink-instal --packages-select topic_statistics_sample
```

## Run

```
$ cd ~/ros2_ws
$ . ./install/local_setup.bash
$ ros2 launch topic_statistics_sample imu.launch.py
$ ros2 launch topic_statistics_sample imu_laser.launch.py
```

## Sample Result

### topic age

| case          |      min |       max |       avg |   std_dev |
|---------------|----------|-----------|-----------|-----------|
| imu only      | 0.000000 |  1.000000 |  0.001005 |  0.031686 |
| imu and laser | 0.000000 | 96.000000 | 15.337022 | 18.064163 |

### topic period

| case          |       min |       max |       avg |   std_dev |
|---------------|-----------|-----------|-----------|-----------|
| imu only      | 18.000000 | 21.000000 | 19.487928 |  0.507841 |
| imu and laser |  0.000000 | 65.000000 | 19.547835 | 23.582385 |
