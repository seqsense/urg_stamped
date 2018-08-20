urg_stamped
====================

Precisely stamped URG driver for ROS

## background and algorithm

2D-LIDAR URG series provides 1ms resolution timestamp and exclusive clock synchronization mode.
It was hard to compensate clock drift during measurement using them.
Also, the resolution of the timestamp was not enough for a high-speed motion of the sensor.

urg_stamped estimates sub-millisecond timestamp with clock drift compensation.
The estimation is structured by following threefold:
- Observe sensor internal timestamp by using sensor state command that sometimes responds immediately.
- Observe data packet arrival time that is correlated to the true measurement time.
- Fuse accurate but low-resolution timestamp and noisy but high-resolution arrival time by the complementary filter.

## known limitations

- Timestamp estimation is designed for sensors connected by ethernet interface.
  - Tested only for UTM-30LX-EW and UST-20LX at now.
- Some scans are dropped due to the clock synchronization and delay estimation.

## comparizon with urg_node

### configurations

Three UTM-30LX-EW are mounted on a velocity controlled turntable, as shown below, to reconstruct 3-D point cloud from 2-D scans.
The accuracy of the timestamp affects offset and precision of the timestamp affects the distribution of the pointcloud.

[picture here]

### results

stub

[result images here]
