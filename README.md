# Aquanaute

Repository to simulate the Aquanaute boat in maritime conditions such as wind and waves with the plugins created for the VRX and VRX-Challenge in Gazebo and ROS.

## Installation

* Clone this repo to a Catkin Workspace
* Clone the VRX repo to the same Workspace (https://github.com/osrf/vrx)
* In the path
    ```
    "workspace"/src/vrx/vrx_gazebo/launch/
    ```
    create a new launch file "name".launch and copy the code provided below
* In the path
    ```
    "workspace"/src/vrx/vrx_gazebo/worlds/ocean.world.xacro
    ```
    delete the contents of the file and replace them with the code provided below

## Usage
### modify wind and waves
  in the ocean.world.xacro you have
  ```xml
  <!--Waves-->
  <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
  <xacro:ocean_waves gain="0.0" period="5" direction_x="-1.0" direction_y="0.0" angle="0.0"/>

  <!--Wind for the WAM-V. Note, wind parameters are set in the plug.-->
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/usv_wind_plugin.xacro"/>
  <xacro:usv_wind_gazebo direction="90" mean_vel="0" var_gain="5" var_time="1">
    <wind_objs>
      <wind_obj>
        <name>wamv</name>
        <link_name>base_link</link_name>
        <coeff_vector>0.5 0.5 0.33</coeff_vector>
      </wind_obj>
    </wind_objs>
  </xacro:usv_wind_gazebo>
  ```
  it is configured to have no waves and no wind, also the wind and the waves move in the same direction, to activate wind and waves change the "mean_vel" and "gain" parameters respectively (recomended values are smaller than 1.0).

### run
in the workspace run the command at the beginning (and every time you change a parameter)

  ```bash
  catkin_make
  ```

then launch the file you created

  ```bash
  roslaunch vrx_gazebo name.launch
  ```
## Author
Ricardo RICO URIBE intern at U2IS in the summer of 2020

## Code
### .launch
```xml
<?xml version="1.0"?>
<launch>
      <env name="ROSCONSOLE_CONFIG_FILE" value="$(find vrx_gazebo)/config/custom_rosconsole.conf"/>
      <!-- Gazebo world to load -->
      <arg name="world" default="$(find vrx_gazebo)/worlds/ocean.world" />
      <!-- If true, run gazebo GUI -->
      <arg name="gui" default="true" />
      <!-- If true, run gazebo in verbose mode -->
      <arg name="verbose" default="true"/>
      <!-- If true, start in paused state -->
      <arg name="paused" default="false"/>
      <!-- Set various other gazebo arguments-->
      <arg name="extra_gazebo_args" default=""/>
      <!-- Start in a default namespace -->
      <arg name="namespace" default="aquanaute"/>

      <!-- Initial USV location and attitude-->
      <arg name="x" default="0" />
      <arg name="y" default="0" />
      <arg name="z" default="0.1" />
      <arg name="P" default="0" />
      <arg name="R" default="0" />
      <arg name="Y" default="0" />

      <env name="VRX_DEBUG" value="true"/>
     
      <!-- Allow user specified thruster configurations
       H = stern trusters on each hull
       T = H with a lateral thruster
       X = "holonomic" configuration -->
      <arg name="thrust_config" default="H" />

      <!-- Do you want to enable sensors? -->
      <arg name="camera_enabled" default="false" />
      <arg name="gps_enabled" default="false" />
      <arg name="imu_enabled" default="false" />
      <arg name="lidar_enabled" default="false" />
      <arg name="ground_truth_enabled" default="false" />

      <!-- Start Gazebo with the world file -->
      <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="world_name" value="$(arg world)"/>
            <arg name="verbose" value="$(arg verbose)"/>
            <arg name="paused" value="$(arg paused)"/>
            <arg name="use_sim_time" value="true"/>
            <arg name="gui" value="$(arg gui)" />
            <arg name="enable_ros_network" value="true"/>
            <arg name="extra_gazebo_args" value="$(arg extra_gazebo_args)"/>
      </include>

      <!-- Load robot model -->
      <arg name="urdf" default="$(find aquanaute_gazebo)/urdf/aquanaute_gazebo.urdf.xacro"/>
      <param name="$(arg namespace)/robot_description" command="$(find xacro)/xacro '$(arg urdf)'"/>

      <!-- Spawn model in Gazebo, script depending on non_competition_mode -->
      <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" args="-x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg R) -P $(arg P) -Y $(arg Y)
              -urdf -param $(arg namespace)/robot_description -model aquanaute"/>
</launch>
```
### ocean.world.xacro
```xml
<?xml version="1.0" ?>
<!--
  Copyright (C) 2019  Rhys Mainwaring


   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
 
       http://www.apache.org/licenses/LICENSE-2.0
 
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

-->
<sdf version="1.6" 
  xmlns:xacro="http://ros.org/wiki/xacro">
  <world name="ocean_world">

    <!-- GUI -->
    <gui>
      <plugin name="keyboard_gui_plugin" filename="libKeyboardGUIPlugin.so"/>
      <camera name='user_camera'>
        <pose frame=''>40 40 20 0 0.4 180</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <!-- Scene -->
    <scene>
      <sky>
        <time>10</time>
        <sunrise>6</sunrise>
        <sunset>18</sunset>
        <clouds>
          <speed>12</speed>
          <direction>1.57079</direction>
        </clouds>
      </sky>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <!-- Lights -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!--Waves-->
    <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
    <xacro:ocean_waves gain="0.0" period="5" direction_x="-1.0" direction_y="0.0" angle="0.0"/>

    <!--Wind for the WAM-V. Note, wind parameters are set in the plug.-->
    <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/usv_wind_plugin.xacro"/>
    <xacro:usv_wind_gazebo direction="90" mean_vel="0.0" var_gain="5" var_time="1">
      <wind_objs>
        <wind_obj>
          <name>aquanaute</name>
          <link_name>base_link</link_name>
          <coeff_vector>0.5 0.5 0.33</coeff_vector>
        </wind_obj>
      </wind_objs>
    </xacro:usv_wind_gazebo>
  </world>
</sdf>


```

## License


