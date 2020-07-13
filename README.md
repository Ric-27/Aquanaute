# Aquanaute

Repository to simulate the Aquanaute boat in maritime conditions such as wind and waves with the plugins created for the VRX and VRX-Challenge in Gazebo and ROS.

## Installation

* Clone this repo to a Catkin Workspace

* Clone the VRX repo (hosted at ensta) to the same Workspace (http://git-u2is.ensta.fr/ssh/vrx) and checkout to the branch "aquanaute"

## Usage
### modify wind and waves
  in the ocean.world.xacro (/vrx_gazebo/worlds) you have
  ```xml
  <!--Waves-->
  <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
  <xacro:ocean_waves gain="0.0" period="5" direction_x="-1.0" direction_y="0.0" angle="0.0"/>

  <!--Wind-->
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
  roslaunch aquanaute_gazebo aquanaute.launch testing:=false
  ```
  this parameter can be changed to true to deactivate the plugins from vrx (the world will be changed to an empty world)

on another terminal (inside the ardupilot root) run
  ```bash
  ./Tools/autotest/sim_vehicle.py --vehicle=Rover --frame=sailboat-motor --map --console --location=Xlac
  ```

## Author
Ricardo RICO URIBE intern at U2IS in the summer of 2020

## License


