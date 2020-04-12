cmake_minimum_required(VERSION 3.5)
project(wamv_description)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  urdf/wamv_base.urdf.xacro
    INSTALL DESTINATION urdf
)

# add_custom_target(urdf ALL COMMAND "ros2" "launch" ${PROJECT_NAME} "wamv_description.launch.py")

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()