cmake_minimum_required(VERSION 3.5)
project(vanetza)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    etsi_its_msgs
    message_filters
    message_generation
    nodelet
    roscpp
    rospy
    std_msgs
	geometry_msgs
	sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS coroutine system)
find_package(Vanetza 0.8.1 REQUIRED)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/cmc-ccu.proto)
add_library(ccu-protobuf INTERFACE)
target_sources(ccu-protobuf INTERFACE ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(ccu-protobuf INTERFACE ${PROTOBUF_LIBRARIES})
target_include_directories(ccu-protobuf INTERFACE ${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(FILES
    BtpDataIndication.msg
    GeoNetArea.msg
    GeoNetDestination.msg
    LinkLayer.msg
    LinkLayerStamped.msg
    PositionVector.msg
    SecurityIdChange.msg
    TrafficClass.msg
	LocalisationBody.msg
	LocalisationStatus.msg
	LocalisationVehicleInformation.msg
)

add_service_files(FILES
    BtpData.srv
    DenmTx.srv
)

generate_messages(DEPENDENCIES etsi_its_msgs sensor_msgs std_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    LIBRARIES vanetza_nodelets
    CATKIN_DEPENDS etsi_its_msgs message_runtime nodelet std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_nodelets
    src/ca_message.cpp
    src/ca_rx_nodelet.cpp
    src/ca_tx_nodelet.cpp
    src/den_rx_nodelet.cpp
    src/den_tx_nodelet.cpp
	src/cp_message.cpp
	src/cp_rx_nodelet.cpp
	src/cp_tx_nodelet.cpp
    src/message_table.cpp
    src/tai_clock.cpp
		)
add_dependencies(${PROJECT_NAME}_nodelets ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelets ${catkin_LIBRARIES} Vanetza::vanetza)

add_executable(${PROJECT_NAME}_node
    src/btp_data_service.cpp
    src/btp_publisher.cpp
    src/dcc_pass_through.cpp
    src/multicast_receiver.cpp
    src/position_provider.cpp
    src/protobuf_access_interface.cpp
    src/radio_configurator.cpp
    src/security_id_publisher.cpp
    src/tai_clock.cpp
    src/time_trigger.cpp
    src/vanetza_ros_node.cpp
    src/vanetza_context.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES
   COMPILE_DEFINITIONS BOOST_COROUTINES_NO_DEPRECATION_WARNING
   OUTPUT_NAME node
   PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   Boost::coroutine Boost::system
   ccu-protobuf
   Vanetza::vanetza
)

add_executable(${PROJECT_NAME}_virtual_ccu src/virtual_ccu.cpp)
set_target_properties(${PROJECT_NAME}_virtual_ccu PROPERTIES OUTPUT_NAME "virtual_ccu" PREFIX "")
target_link_libraries(${PROJECT_NAME}_virtual_ccu ${catkin_LIBRARIES} ccu-protobuf Boost::system)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelets ${PROJECT_NAME}_virtual_ccu
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vanetza_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
