## Creating Your Own Robot

1. Upload .stl/.dae of thrusters and robot model into `/meshes`. 
   * If you're using the T200 thrusters, a model is already in `/meshes`.
2. 
3. Modify `/config/ros_gz_bridge` to add more thrusters

## Overiview

Wanted to simulate depth sensor and gps readings for an ROV. A custom node publishes gps readings at 5 hz when the rov is above water. Both depth and gps are from the gazebo odometry publisher.