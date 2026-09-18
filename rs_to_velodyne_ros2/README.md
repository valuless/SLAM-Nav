# rs_to_velodyne_ros2

ROS 2 node that converts `sensor_msgs/PointCloud2` into Velodyne-compatible
`XYZI`, `XYZIR`, or `XYZIRT` layouts without a PCL conversion pass.

## Build

```bash
cd ~/Documents/robot-navigation
colcon build --packages-select rs_to_velodyne_ros2
source install/setup.bash
```

## Confirm The Ring Count

Run the probe while `/rslidar_points` is being published:

```bash
ros2 run rs_to_velodyne_ros2 ring_probe.py
```

The probe reads the field datatype, byte order, row stride, and point stride:

```text
ring datatype=4 is_bigendian=False; min=0 max=95 unique=96 invalid=0
```

`unique` is the number of rings observed in that frame. Confirm that it is
stable across several runs. RSAIRY may operate with 48, 96, or 192 channels.
For standalone use, write the confirmed value to
`config/converter.yaml` as `ring_count`. For full-system use, write it to both
`ring_count` and `scan_line` in
`robot_navigation_bringup/config/lidar_pipeline.yaml`; the system launch checks
that these values match. Keep FAST-LIO `timestamp_unit: 0` because output time
is seconds.

## Run

The launch file loads `config/converter.yaml` by default:

```bash
ros2 launch rs_to_velodyne_ros2 convert.launch.py
```

Use another ROS 2 parameter file with `params_file:=/absolute/path/to/file.yaml`.

| Parameter | Values | Default |
|---|---|---|
| `output_type` | `XYZI`, `XYZIR`, `XYZIRT` | `XYZIRT` |
| `output_frame_id` | empty keeps input frame | empty |
| `ring_source` | `field`, `organized_rows`, `organized_cols` | `field` |
| `ring_count` | physical channel count | `16` in standalone config |
| `ring_map` | empty identity or full permutation | empty |
| `time_source` | `none`, `timestamp`, `time` | `timestamp` |
| `time_mode` | `absolute`, `relative` | `absolute` |
| `time_unit` | `second`, `millisecond`, `microsecond`, `nanosecond` | `second` |
| `time_order_policy` | `validate`, `ignore` | `validate` |

`XYZIRT` requires a non-`none` time source. `ignore` disables ordering
validation; it does not reorder points. RoboSense absolute timestamps require
`ts_first_point: true` in the driver configuration.

## Validation

```bash
ros2 topic hz /velodyne_points
ros2 topic delay /velodyne_points
ros2 topic echo /velodyne_points --once
```

For `XYZIRT` with the Velodyne layout, `point_step` is 32 and fields are
`x`, `y`, `z`, `intensity`, `ring`, and `time`.

## Deployment Order

1. Run `ring_probe.py` and confirm the physical ring count.
2. Build this package with `colcon build --packages-select rs_to_velodyne_ros2`.
3. Set converter `ring_count` and FAST-LIO `scan_line` to that same value in
   the deployment parameter file.
4. Keep FAST-LIO `timestamp_unit: 0`, then start FAST-LIO.
5. Run rosbag and motion tests; verify that deskewing is correct.
6. On shutdown, verify `pool release_to_delete=0` in the converter log.
