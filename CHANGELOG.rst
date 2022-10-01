========================
Kobuki Velocity Smoother
========================

Forthcoming
-----------

0.15.0 (2022-10-01)
-------------------
* Rename the package to kobuki_velocity_smoother. (`#20 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/20>`_)
* Make a few more style fixes for older cpplint. (`#18 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/18>`_)
* Update the README.md (`#17 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/17>`_)
* Don't install test artifacts. (`#16 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/16>`_)
* Minor cleanup of the includes.
* Remove the internal accel_lim_w variable.
* Remove internal accel_lim_v.
* Remove decel_factor internal variable.
* Remove internal decel_lim_w variable.
* Remove internal decel_lim_v variable.
* Remove speed_lim_w internal variable.
* Remove speed_lim_v internal variable.
* Remove the quiet internal variable.
* Make sure to allow other parameters in the list.
* Enable the style checkers, and fix the style to conform. (`#14 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/14>`_)
* Fix the tests to work with modern ROS 2. (`#13 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/13>`_)
* Small fix to the velocity smoother. (`#12 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/12>`_)
* Fix the test so it works with 'colcon test' (`#11 <https://github.com/kobuki-base/kobuki_velocity_smoother/issues/11>`_)

0.14.0 (2020-01-20)
-------------------
* [ros2] parameters/topics renamed more sensibly, `#8 <https://github.com/kobuki-base/velocity_smoother/pull/8>`_
* [docs] moved from the ROS1 wiki and updated, `#8 <https://github.com/kobuki-base/velocity_smoother/pull/8>`_
* [tests] translational smoothing test added, `#7 <https://github.com/kobuki-base/velocity_smoother/pull/7>`_

0.13.0 (2020-01-20)
-------------------
* [infra] refactored for ROS 2 and renamed to velocity_smoother, `#1 <https://github.com/kobuki-base/velocity_smoother/pull/1>`_

0.6.3 (2014-12-05)
------------------

0.6.2 (2014-11-30)
------------------
* yocs_velocity_smoother: adds node name param to launcher
* adds a little launcher restructing for muxer and smoother
* Contributors: Marcus Liebhardt

0.6.0 (2014-07-08)
------------------
* updating package informations. remove email for authors. updating maintainer
* Contributors: Jihoon Lee

0.5.3 (2014-03-24)
------------------

0.5.2 (2013-11-05)
------------------

0.5.1 (2013-10-14)
------------------
* Unify naming politics for binaries and plugins.

0.5.0 (2013-10-11)
------------------

0.4.1 (2013-10-08)
------------------

0.4.0 (2013-08-29)
------------------
* Add bugtracker and repo info URLs.
* Changelogs at package level.
* Separate and comment velocity feedback remaps.
* License link fixed.

0.3.0 (2013-07-02)
------------------
* Fix on velocity smoother to deal with low-rate simulated time (namely Stage).
* Allow using end velocity commands as robot feedback (until now we can use only odometry).

0.2.3 (2013-04-15)
------------------

0.2.2 (2013-02-10)
------------------

0.2.1 (2013-02-08)
------------------

0.2.0 (2013-02-07)
------------------
* Catkinized.

0.1.3 (2013-01-08)
------------------
* Dynamic reconfigure for velocity/acceleration limits.
* Fix on deceleration smoothing.

0.1.2 (2013-01-02)
------------------
* Add test program.
* Add licensing.

0.1.1 (2012-12-21)
------------------
* Keep direction constant when smoothing velocities, i.e. draw constant arcs. To do so we must sometimes over-limit dv or dw. 
* Bound velocity in addition to acceleration. Also set physically meaningful values for acceleration.

0.1.0 (2012-12-05)
------------------
* Initial version.
