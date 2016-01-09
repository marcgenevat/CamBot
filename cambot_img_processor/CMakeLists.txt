cmake_minimum_required(VERSION 2.8.3)
project(cambot_img_processor)

#set C++11 support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

#source files
SET(SRCS 
      src/cambot_img_processor_node.cpp
      src/cambot_img_processor_main.cpp)

## The catkin_package macro generates cmake config files for your package
catkin_package(
      CATKIN_DEPENDS 
            roscpp 
            rospy 
            image_transport 
            cv_bridge)

## Specify additional locations of header files
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare a cpp executable for pipol_tracker_node
add_executable(cambot_img_processor ${SRCS})
#add_dependencies(pipol_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(cambot_img_processor ${OpenCV_LIBS} ${catkin_LIBRARIES})

#install binaries
install(TARGETS cambot_img_processor
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
#   # myfile1
#   # myfile2
	data/haarcascades/haarcascade_frontalface_default.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )



#Share launch and configs
foreach(dir launch config)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
