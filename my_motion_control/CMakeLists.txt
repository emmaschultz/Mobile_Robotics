cmake_minimum_required(VERSION 2.8.3)
project(my_motion_control)

find_package(catkin_simple REQUIRED)

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(my_action_server src/my_action_server.cpp)
cs_add_executable(my_action_client src/my_action_client.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

cs_install()
cs_export()
    
