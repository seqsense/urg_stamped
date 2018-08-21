urg_stamped
====================

Precisely and accurately stamped URG driver for ROS

## Backgrounds and Algorithms

2D-LIDAR URG series provides 1ms resolution timestamp and exclusive clock synchronization mode.
It was hard to compensate clock drift during measurement using them.
Also, the resolution of the timestamp was not enough for a high-speed motion of the sensor.

urg_stamped estimates sub-millisecond timestamp with clock drift compensation.
The estimation is structured by following threefold:
- Observe sensor internal timestamp by using sensor state command that sometimes responds immediately.
- Observe data packet arrival time that is correlated to the true measurement time.
- Fuse accurate but low-resolution timestamp and noisy but high-resolution arrival time by the complementary filter.

## Usages

Topics and major parameters are designed to be compatible with [urg_node](http://wiki.ros.org/urg_node).

### Published Topics

- **scan** (sensor_msgs::LaserScan)

### Parameters

#### urg_node compatible parameters

- **ip_address** (string): device IP address
- **ip_port** (int): device TCP/IP port
- **frame_id** (string): frame_id of published scans
- **publish_intensity** (bool): fill intensity field if true

#### urg_stamped specific parameters

- **sync_interval_min** (double): minimum interval to try observing sensor internal timestamp in seconds
- **sync_interval_max** (double): maximum interval to try observing sensor internal timestamp in seconds
- **delay_estim_interval** (double): communication delay estimation interval in seconds (dropping 2 or 3 scans during delay estimation)

## Known Limitations

- Timestamp estimation is designed for sensors connected by ethernet interface.
  - Tested only for UTM-30LX-EW and UST-20LX at now.
- Some scans are dropped due to the clock synchronization and delay estimation.

## Comparison with urg_node

### Configurations

Three UTM-30LX-EWs are mounted on a velocity controlled turntable, as shown below, to reconstruct 3-D point cloud from 2-D scans.
The accuracy of the timestamp affects offset and precision of the timestamp affects the distribution of the pointcloud.

![SQ-LIDAR](doc/images/sqlidar.jpg)

### Results

The image below shows point cloud with 1 rad/s of the turntable which can be assumed as a reference.
(decay time of the point cloud: 10 seconds)

![urg_stamped 1 rad/s](doc/images/urg_stamped_1radps.png)

urg_node has large error even if calibrate_time and synchronize_time options are enabled (captioned as urg_node (sync)).
urg_stamped has better timestamp characteristics comparing with urg_node.

&nbsp;             | 10 rad/s                                                        | 20 rad/s
---                | ---                                                             | ---
urg_stamped        | ![urg_stamped 10 rad/s](doc/images/urg_stamped_10radps.png)     | ![urg_stamped 20 rad/s](doc/images/urg_stamped_20radps.png)
urg_node           | ![urg_node 10 rad/s](doc/images/urg_node_10radps.png)           | ![urg_node 20 rad/s](doc/images/urg_node_20radps.png)
urg_node<br>(sync) | ![urg_node sync 10 rad/s](doc/images/urg_node_sync_10radps.png) | ![urg_node sync_20 rad/s](doc/images/urg_node_sync_20radps.png)
