cmake_minimum_required(VERSION 3.0.0)
project(nerian_stereo)

cmake_policy(SET CMP0057 NEW)
include(CMakeParseArguments)

# Find ROS build dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME} msg/StereoCameraInfo.msg DEPENDENCIES std_msgs sensor_msgs)

#######################
## libvisiontransfer ##
#######################

set(VT_VERSION 5_0_1)
set(BUILD_VT 1)

if(BUILD_VT)
    # Extract sources while configuring
    execute_process(COMMAND tar --keep-newer-files --warning none -xJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR})

    # Re-extract source files if updated
    add_custom_target(nerian_stereo_untar_src ALL
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz
        COMMAND tar --keep-newer-files --warning none -xJf ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz -C ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Build sources at runtime
    add_custom_target(nerian_stereo_visiontransfer_stub ALL
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nerian-vision-software-${VT_VERSION}-src.tar.xz
        COMMAND ${CMAKE_COMMAND} .
        COMMAND make
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer
    )

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer/lib/libvisiontransfer.so
        DESTINATION lib
    )

    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer/visiontransfer
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN "CMakeFiles" EXCLUDE
    )

    link_directories(${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer/lib)

else()

    # Development settings: Use local libvisiontransfer
    link_directories($ENV{VISION_TRANSFER_LIB})
    include_directories($ENV{VISION_TRANSFER_INCLUDE})
endif()

###########
## Build ##
###########

# Activate c++11 support
include(CheckCXXCompilerFlag)
if(NOT WIN32)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
    if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    else()
        message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
    endif()
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
#${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
include_directories(
    #${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    ${nerian_stereo_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/libvisiontransfer
    ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/nvcom/helpers
    ${OpenCV_INCLUDE_DIRS}
    )

# Declare a C++ executable
add_executable(nerian_stereo_node
    src/nerian_stereo_node.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/nerian-vision-software-${VT_VERSION}-src/nvcom/helpers/colorcoder.cpp
)

# Dependencies to ROS pacakges
ament_target_dependencies(nerian_stereo_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rmw_implementation"
)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(nerian_stereo_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
    nerian_stereo_visiontransfer_stub)

# Specify libraries to link a library or executable target against
target_link_libraries(nerian_stereo_node ${OpenCV_LIBS} visiontransfer)


# Make locally defined messages available t onode
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(nerian_stereo_node ${PROJECT_NAME} ${typesupport_impl})
endforeach()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS nerian_stereo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES data/calib.yaml
    DESTINATION data
)

install(FILES launch/nerian_stereo.launch
  DESTINATION launch
)

###################################
## ament specific configuration ##
###################################
ament_export_dependencies(
    rosidl_default_runtime
)
ament_export_libraries(visiontransfer)
ament_package()

