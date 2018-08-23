^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urg_stamped
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2018-08-23)
------------------
* Fix license in manifest (`#39 <https://github.com/seqsense/urg_stamped/issues/39>`_)
* Update README (`#36 <https://github.com/seqsense/urg_stamped/issues/36>`_)
* Estimate sub-millisecond timestamp (`#35 <https://github.com/seqsense/urg_stamped/issues/35>`_)

  * Estimate sub-millisecond timestamp by complementary filter fusing timestamp and packet arrival time
  * Add packet arrival time outlier removal
  * Add zero-delay moving average
  * Add unit tests for filters

* Add build matrix for ROS indigo/kinetic/melodic (`#38 <https://github.com/seqsense/urg_stamped/issues/38>`_)

  * Add build matrix
  * Fix workspace init
  * Fix rosdep argument
  * Fold test details
  * Fix test for latest g++

* Merge pull request `#34 <https://github.com/seqsense/urg_stamped/issues/34>`_ from seqsense/update-manifest-format
* Update manifest format
* Receive both MD and ME response by one callback (`#33 <https://github.com/seqsense/urg_stamped/issues/33>`_)
* Make some info messages debug level (`#30 <https://github.com/seqsense/urg_stamped/issues/30>`_)
* Fix step chage of estimated time origin (`#28 <https://github.com/seqsense/urg_stamped/issues/28>`_)
* Update CI settings (`#26 <https://github.com/seqsense/urg_stamped/issues/26>`_)
* Apply Apache License 2.0 (`#25 <https://github.com/seqsense/urg_stamped/issues/25>`_)
* Add periodic communication delay estimation (`#23 <https://github.com/seqsense/urg_stamped/issues/23>`_)

  * Add periodic communication delay estimation
  * Make timeSync and delayEstimation exclusive
  * Retry TM command if not responded
  * Reduce duration for delay estimation

* Fix time origin calculation (`#21 <https://github.com/seqsense/urg_stamped/issues/21>`_)

  * Fix delay check
  * Estimate time using received time and estimated delay
  * Fix time origin calculation

* Randomize time sync timing (`#20 <https://github.com/seqsense/urg_stamped/issues/20>`_)
* Tweak UTM behavior with intensity (`#18 <https://github.com/seqsense/urg_stamped/issues/18>`_)
* Fix II response parsing on UTM (`#17 <https://github.com/seqsense/urg_stamped/issues/17>`_)
* Add TCP connection watchdog (`#15 <https://github.com/seqsense/urg_stamped/issues/15>`_)
* Handle device timestamp overflow (`#12 <https://github.com/seqsense/urg_stamped/issues/12>`_)

  * Handle device timestamp overflow
  * Add test for Walltime

* Add test for Decoder (`#14 <https://github.com/seqsense/urg_stamped/issues/14>`_)
* Validate checksum (`#11 <https://github.com/seqsense/urg_stamped/issues/11>`_)
* Add publish_intensity parameter (`#9 <https://github.com/seqsense/urg_stamped/issues/9>`_)
* Fix clock gain estimation (`#7 <https://github.com/seqsense/urg_stamped/issues/7>`_)

  * Rely on sinle clock gain estimation

* Make debug outputs detailed (`#6 <https://github.com/seqsense/urg_stamped/issues/6>`_)
* Add CI (`#4 <https://github.com/seqsense/urg_stamped/issues/4>`_)

  * Add CI
  * Fix lint errors

* Estimate device clock gain (`#3 <https://github.com/seqsense/urg_stamped/issues/3>`_)
* Increase outlier removal thresholds
* Fix message header
* Fix boost placeholder namespace
* Calculate timestamp in system time
* Use urg_node compatible parameter names
* Change path and namespace to scip2
* Add communication delay estimation
* Output LaserScan messages
* Add stream data processors
* Add parameter response processors
* Add base protocol layer
* Add TCP connection layer
* Contributors: Atsushi Watanabe, So Jomura, jojo43
