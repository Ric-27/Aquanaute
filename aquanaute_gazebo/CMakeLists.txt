cmake_minimum_required(VERSION 2.8.3)
project(aquanaute_gazebo)
find_package(catkin REQUIRED COMPONENTS
  aquanaute_description
  usv_gazebo_plugins
  xacro
)
catkin_package(
  CATKIN_DEPENDS aquanaute_description usv_gazebo_plugins xacro
)

# Generate urdf files from xacro and install
xacro_add_files(
  urdf/aquanaute_gazebo.urdf.xacro
  INORDER INSTALL DESTINATION urdf
)

# Install xacro files / macros
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

