# Aquanaute

Repository to simulate the Aquanaute boat in maritime conditions such as wind and waves and with autopilot provided by ardupilot, using the plugins created for the VRX and VRX-Challenge in Gazebo and ROS and the plugin ardupilot-gazebo.

---

## Installation

* Clone this repo to a Catkin Workspace

* Clone the VRX repo (hosted at ensta) to the same Workspace (<http://git-u2is.ensta.fr/ssh/vrx>) and change to the branch "aquanaute"

### ardupilot parameters

in the file "vehiculeinfo.py" (/ardupilot/Tools/autotest/pysim) you need to change the default parameters file for the "gazebo-rover" in line 280 to the aquanaute.parm file provided in /aquanaute/aquanaute_description/ardu_params you can copy and paste the file in the default_params folder located inside (/ardupilot/Tools/autotest).
*There is an error when you launch ardupilot, the motors in the gazebo simulation will start to turn in a fixed direction and wont respond to commands, this issue has not yet been fixed and it's root cause it's not yet defined, it may be the ardupilot_gazebo plugin or the ardupilot frame*

---

## Usage

### modify wind and waves

in the ocean.world.xacro (/vrx_gazebo/worlds) you have

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

It is configured to have no waves and no wind, also the wind and the waves move in the same direction, to activate wind and waves change the "mean_vel" and "gain" parameters respectively (recomended values are smaller than 1.0).  
Even when these values are set to 0, the boat will move from right to left.

---

### run

#### compilation

in the workspace run the command at the beginning (and every time you change a parameter in the vrx xacros)

    catkin_make

#### gazebo simulation

then launch the file you created

    roslaunch aquanaute_gazebo aquanaute.launch testing:=false

this parameter can be changed to true to deactivate the plugins from vrx (the world will be changed to an empty world without gravity)

#### ardupilot

on another terminal (inside the ardupilot root) run

    ./Tools/autotest/sim_vehicle.py -v APMrover2 -f gazebo-rover --map --console -l 48.71603264538473,2.213283777236939,155,0 

---

## Author

Ricardo RICO URIBE intern at U2IS in the summer of 2020
