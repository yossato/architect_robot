-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  sensor_bridge = {
    horizontal_laser_min_range = 0.,
    horizontal_laser_max_range = 50.,
    horizontal_laser_missing_echo_ray_length = 5.,
    constant_odometry_translational_variance = 0.,
    constant_odometry_rotational_variance = 0.,
  },
  map_frame = "map",
  tracking_frame = "lrf_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry_data = true,
  use_horizontal_laser = true,
  use_horizontal_multi_echo_laser = false,
  num_lasers_3d = 0,
  lookup_transform_timeout_sec = 3.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 4e-2,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

return options
