cmake_minimum_required(VERSION 2.8.3)
project(coordinator)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package

#cs_add_executable(coordinator src/coordinator.cpp)
cs_add_executable(arm_test src/arm_test.cpp)
cs_add_executable(visit_all src/visit_all.cpp)
cs_add_executable(grab_tote src/grab_tote.cpp)
cs_add_executable(grab_tote_v2 src/grab_tote_v2.cpp)
cs_add_executable(grab_all src/grab_all.cpp)
cs_add_executable(grab_gearbox src/grab_gearbox.cpp)
cs_add_executable(grab_object_interactive src/grab_object_interactive.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)

cs_install()
cs_export()
