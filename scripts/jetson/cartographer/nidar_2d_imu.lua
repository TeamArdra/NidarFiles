-- Cartographer 2D SLAM with the Pixhawk's IMU (/mavros/imu/data, frame
-- "base_link"): the IMU gives gravity (so a tilted LiDAR scan is levelled)
-- and the turn rate (so fast yaw doesn't lose the scan match).
-- start_jetson.sh uses this only when the Pixhawk is connected and the IMU
-- arrives fast enough; otherwise nidar_2d.lua (LiDAR only).
--
-- Cartographer needs the IMU at the tracking frame, so we track base_link
-- (the Pixhawk) and start_jetson.sh publishes the static TF base_link ->
-- laser. lidar_node already turns the scan to the drone's nose
-- (NIDAR_LIDAR_YAW_DEG), so that TF has no rotation, only the LiDAR's offset
-- from the Pixhawk (NIDAR_LIDAR_X_M / _Y_M / _Z_M).

include "nidar_2d.lua"

options.tracking_frame = "base_link"
options.published_frame = "base_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true

return options
