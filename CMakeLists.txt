cmake_minimum_required(VERSION 2.8.3)
project(bin_picking_region_grow)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tf 
  pcl_ros
  message_generation
  std_msgs
  cmake_modules
  )

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/bin_picking_region_grow.cpp
            ${PROJECT_SOURCE_DIR}/src/bin_picking_region_grow_lib.cpp
            )

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/bin_picking_region_grow_lib.h
            )


add_service_files(FILES bin_picking_region_grow_service.srv)
generate_messages(DEPENDENCIES)
            
catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
  cmake_modules
  
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
${PCL_LIBRARIES})

