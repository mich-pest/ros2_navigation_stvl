Site: https://navigation.ros.org/tutorials/docs/navigation2_with_stvl.html

file: /opt/ros/galactic/share/nav2_bringup/params/nav2_params.yaml

at 
```bash
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: True
      plugins: ["static_layer", "stvl_layer"] # CHANGE THIS LINE
```
      
and add this part in the local costmap:
```bash
stvl_layer:
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer" # For Foxy and later
  enabled: true
  voxel_decay: 15.
  decay_model: 0
  voxel_size: 0.05
  track_unknown_space: true
  unknown_threshold: 15
  mark_threshold: 0
  update_footprint_enabled: true
  combination_method: 1
  origin_z: 0.0
  publish_voxel_map: true
  transform_tolerance: 0.2
  mapping_mode: false
  map_save_duration: 60.0
  observation_sources: pointcloud
  pointcloud:
    data_type: PointCloud2
    topic: /intel_realsense_r200_depth/points
    marking: true
    clearing: true
    obstacle_range: 3.0
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
    expected_update_rate: 0.0
    observation_persistence: 0.0
    inf_is_valid: false
    voxel_filter: false
    clear_after_reading: true
    max_z: 7.0
    min_z: 0.1
    vertical_fov_angle: 0.8745
    horizontal_fov_angle: 1.048
    decay_acceleration: 15.0
    model_type: 0
```
