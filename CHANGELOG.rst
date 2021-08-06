^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urg_stamped
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.8 (2021-03-10)
------------------
* Fix token name for releasing (`#86 <https://github.com/seqsense/urg_stamped/issues/86>`_)
* Fix prerelease test (`#85 <https://github.com/seqsense/urg_stamped/issues/85>`_)
* Remove unnecessary newline from log (`#82 <https://github.com/seqsense/urg_stamped/issues/82>`_)
* Migrate to GitHub Actions (`#81 <https://github.com/seqsense/urg_stamped/issues/81>`_)
* Contributors: Atsushi Watanabe

0.0.13 (2021-08-03)
-------------------
* Remove doubled QT command request (`#112 <https://github.com/seqsense/urg_stamped/issues/112>`_)
* Contributors: Atsushi Watanabe

0.0.12 (2021-07-11)
-------------------
* Fix gpg key source in prerelease test (`#111 <https://github.com/seqsense/urg_stamped/issues/111>`_)
* Reboot sensor if time sync error count exceeded limit and never succeeded (`#108 <https://github.com/seqsense/urg_stamped/issues/108>`_)
* Rotate bot token (`#106 <https://github.com/seqsense/urg_stamped/issues/106>`_)
* Drop Kinetic (`#105 <https://github.com/seqsense/urg_stamped/issues/105>`_)
* Contributors: Atsushi Watanabe

0.0.11 (2021-04-23)
-------------------
* Send QT command twice to avoid being ignored (`#103 <https://github.com/seqsense/urg_stamped/issues/103>`_)
* Change TM0 status 10 error log level (`#100 <https://github.com/seqsense/urg_stamped/issues/100>`_)
* Fallback timeout during time synchronization (`#97 <https://github.com/seqsense/urg_stamped/issues/97>`_)
* Contributors: Atsushi Watanabe

0.0.10 (2021-04-07)
-------------------
* Add codecov.yml (`#96 <https://github.com/seqsense/urg_stamped/issues/96>`_)
* Fix error handling during delay estimation (`#92 <https://github.com/seqsense/urg_stamped/issues/92>`_)
* Add option to enable debug log output at launch (`#93 <https://github.com/seqsense/urg_stamped/issues/93>`_)
* Send TM command after receiving QT response (`#91 <https://github.com/seqsense/urg_stamped/issues/91>`_)
* Refactor directory and namespace (`#90 <https://github.com/seqsense/urg_stamped/issues/90>`_)
* Contributors: Atsushi Watanabe

0.0.9 (2021-03-10)
------------------
* Release 0.0.8 (`#84 <https://github.com/seqsense/urg_stamped/issues/84>`_)
* Fix token name for releasing (`#86 <https://github.com/seqsense/urg_stamped/issues/86>`_)
* Fix prerelease test (`#85 <https://github.com/seqsense/urg_stamped/issues/85>`_)
* Remove unnecessary newline from log (`#82 <https://github.com/seqsense/urg_stamped/issues/82>`_)
* Migrate to GitHub Actions (`#81 <https://github.com/seqsense/urg_stamped/issues/81>`_)
* Contributors: Atsushi Watanabe, github-actions[bot]

0.0.7 (2020-08-12)
------------------
* Remove travis_retry from prerelease_test.sh (`#78 <https://github.com/seqsense/urg_stamped/issues/78>`_)
* Use downloaded gh-pr-comment binary in docker container (`#77 <https://github.com/seqsense/urg_stamped/issues/77>`_)
* Download gh-pr-comment binary instead of using pip (`#75 <https://github.com/seqsense/urg_stamped/issues/75>`_)
* Reboot lidar when it is in abnormal state (`#71 <https://github.com/seqsense/urg_stamped/issues/71>`_)
* Create GitHub Release after bloom release (`#72 <https://github.com/seqsense/urg_stamped/issues/72>`_)
* Update CI config (`#69 <https://github.com/seqsense/urg_stamped/issues/69>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.0.6 (2020-06-23)
------------------
* Add check for device timestamp jump to node (`#66 <https://github.com/seqsense/urg_stamped/issues/66>`_)
* Add timestamp jump detector to Walltime (`#65 <https://github.com/seqsense/urg_stamped/issues/65>`_)
* Contributors: Yuta Koga

0.0.5 (2020-04-07)
------------------
* Support Noetic (`#61 <https://github.com/seqsense/urg_stamped/issues/61>`_)
* Contributors: Atsushi Watanabe

0.0.4 (2020-01-29)
------------------
* Automate bloom release (`#58 <https://github.com/seqsense/urg_stamped/issues/58>`_)
* Add error count check (`#57 <https://github.com/seqsense/urg_stamped/issues/57>`_)
* Fix response status check (`#56 <https://github.com/seqsense/urg_stamped/issues/56>`_)
* Format pointer alignment (`#55 <https://github.com/seqsense/urg_stamped/issues/55>`_)
* Contributors: Atsushi Watanabe

0.0.3 (2019-08-15)
------------------
* Run prerelease-test on release- branch (`#49 <https://github.com/seqsense/urg_stamped/issues/49>`_)
* Refactor logging (`#48 <https://github.com/seqsense/urg_stamped/issues/48>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#47 <https://github.com/seqsense/urg_stamped/issues/47>`_)
* Remove old_boost_fix.h (`#42 <https://github.com/seqsense/urg_stamped/issues/42>`_)
* Contributors: Atsushi Watanabe

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
