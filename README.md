# gazebo_sfm_plugin

A plugin for simulation of human pedestrians in ROS 2 and Gazebo Ignition.  
The persons are affected by the obstacles and other persons using the [Social Force Model](https://github.com/robotics-upo/lightsfm)

> [!NOTE]
> **Tested in ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8).**
> For the legacy ROS 2 Galactic / Gazebo Classic 11 version, see the `galactic` branch history (commit `4e84fed` and earlier).

![](https://github.com/robotics-upo/gazebo_sfm_plugin/blob/master/media/images/capture3.jpg)

## Plugin configuration

The plugin can be applied to each Gazebo Actor indicated in the Gazebo world file.  
An example snippet is shown next:

```html
<actor name="actor1">
	<pose>-1 2 1.25 0 0 0</pose>
	<skin>
		<filename>walk.dae</filename>
		<scale>1.0</scale>
	</skin>
	<animation name="walking">
		<filename>walk.dae</filename>
		<scale>1.000000</scale>
		<interpolate_x>true</interpolate_x>
	</animation>
	<!-- plugin definition -->
	<plugin name="gazebo_sfm_plugin::PedestrianSFMPlugin" filename="PedestrianSFMPlugin">
		<velocity>0.9</velocity>
		<radius>0.4</radius>
		<animation_factor>5.1</animation_factor>
		<animation_name>walking</animation_name>
		<people_distance>6.0</people_distance>
		<!-- weights -->
		<goal_weight>2.0</goal_weight>
		<obstacle_weight>80.0</obstacle_weight>
		<social_weight>15</social_weight>
		<group_gaze_weight>3.0</group_gaze_weight>
		<group_coh_weight>2.0</group_coh_weight>
		<group_rep_weight>1.0</group_rep_weight>
		<ignore_obstacles>
			<model>cafe</model>
			<model>ground_plane</model>
		</ignore_obstacles>
		<trajectory>
			<cyclic>true</cyclic>
			<waypoint>-1 2 1.25</waypoint>
			<waypoint>-1 -8 1.25</waypoint>
		</trajectory>
	</plugin>
</actor>
```
The parameters that can be configured for each pedestrian are:

### General params

*  ```<velocity>```. Maximum velocity (*m/s*) of the pedestrian.
*  ```<radius>```. Approximate radius of the pedestrian's body (m).
*  ```<animation_factor>```. Factor employed to coordinate the animation with the walking velocity.
* ```<people_distance>```.  Maximum detection distance of the surrounding pedestrians.
* ```<animation_name>```.  Name of the actor `<animation>` to play (defaults to `walking`).

### SFM Weights

*  The weight factors that modify the navigation behavior. See the [Social Force Model](https://github.com/robotics-upo/lightsfm) for further information.

### Obstacle params

* ```<ignore_obstacles>```.  All the models that must be ignored as obstacles, must be indicated here. The other actors in the world are included automatically.

### Trajectory params

* ```<trajectory>```. The list of waypoints that the actor must reach must be indicated here. 
	- ```<waypoint>```. Each waypoint must be indicated by its coordinates X, Y, Z in the world frame.
	- ```<cyclic>```. If true, the actor will start the waypoint sequence when the last waypoint is reached.

## Dependencies

* **ROS 2 Jazzy** and **Gazebo Harmonic**. On Jazzy, Gazebo Harmonic is provided
  through the ROS vendor packages — installing `ros-jazzy-ros-gz` pulls them in:
  ```bash
  sudo apt install ros-jazzy-ros-gz
  ```
* The **Social Force Model** library, lightsfm (standalone, header-only):
  https://github.com/robotics-upo/lightsfm — install it under `/usr/local`.
  ```bash
  git clone https://github.com/robotics-upo/lightsfm.git
  cd lightsfm
  make
  sudo make install
  sudo ldconfig
  ```

## Compilation

This is a ROS 2 package, so place it inside a ROS 2 workspace and build it with
colcon:
```bash
colcon build --symlink-install --packages-select gazebo_sfm_plugin -cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Example

An example Gazebo world can be launched through:
```bash
ros2 launch gazebo_sfm_plugin cafe_ros2.launch.py
```

The example world (`worlds/cafe3.sdf`) pulls the `Cafe`, `Cafe table` and `Ground Plane` models from [Gazebo Fuel](https://app.gazebosim.org/fuel) on the first run, so an internet connection is required the first time.

### Notes for the Gazebo Harmonic port

* The plugin is now a `gz::sim::System` (`ISystemConfigure` + `ISystemPreUpdate`)
  instead of a Gazebo Classic `ModelPlugin`.
* Each actor is driven kinematically through its `TrajectoryPose` and
  `AnimationTime` components, so its base `Pose` is left at the origin.
* Obstacle avoidance uses the world-frame `AxisAlignedBox` component, which the
  physics system populates for the relevant models.
* When embedding the plugin in your own world, reference it as
  `filename="PedestrianSFMPlugin"` with
  `name="gazebo_sfm_plugin::PedestrianSFMPlugin"`, and make sure the plugin
  library directory is on `GZ_SIM_SYSTEM_PLUGIN_PATH` (the provided launch file
  sets it).
* Actor `<skin>`/`<animation>` `<filename>` paths are resolved **relative to the
  world file** (not via `GZ_SIM_RESOURCE_PATH`). In `cafe3.sdf` the meshes are
  therefore referenced as `../models/walk.dae`, since the installed layout places
  the world in `share/gazebo_sfm_plugin/worlds/` and the meshes in
  `share/gazebo_sfm_plugin/models/`.
* The texture images referenced by the Fuel cafe / cafe-table models
  (`Maple.jpg`, `Wood_Floor_Dark.jpg` and the cafe's `__auto_*.jpg` set) are
  bundled under `media/models/` (installed onto `GZ_SIM_RESOURCE_PATH`) so the
  scene renders with textures instead of emitting "Could not resolve file"
  warnings. They are resolved by bare filename through the resource path.
