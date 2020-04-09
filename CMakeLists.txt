cmake_minimum_required(VERSION 3.5)
project(wamv_description)
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files(
  urdf/wamv_base.urdf.xacro
    INSTALL DESTINATION urdf
)

install(DIRECTORY models/
  DESTINATION share/wamv_description/models)

install(DIRECTORY urdf/
  DESTINATION share/wamv_description/urdf)

ament_package()