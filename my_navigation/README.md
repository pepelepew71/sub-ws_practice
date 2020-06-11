# my_navigation

Navigation with different map localization method.

## Localization

Methods for tf odom -> base_link

- gazebo
- ekf odom

Methods for tf map -> odom

- Navigation
  - amcl
  - ekf amcl
  - ekf gps
  - ekf gps + amcl

## Planning

Use [move_base](http://wiki.ros.org/move_base) for planning.

Use [ros_gps_nav](https://github.com/pepelepew71/ros_gps_nav) to create gps waypoint.
