
LASER 3D:
- Re-written laser 3d xacro.
- Added in generic_laser.xacro the laser_3d.xacro, since in there, there are every possible laser.
- Added in 2wd.xacro the new macro of the laser_3d, which was added in the generic_laser.xacro.
- Added in 2wd_properties.xacro the new laser_3d block and changed its position.
- Changed the nav2 param file, according to the vlp16_config.yaml which is present in the example folder of stvl repo.

CHANGE MAP:
- change .world file in linorobot2_description/description.launch (having the new .world in worlds folder)
- put the files stored in models folder in https://github.com/chaolmu/gazebo_models_worlds_collection inside the model folder of Gazebo at /usr/share/gazebo-11

