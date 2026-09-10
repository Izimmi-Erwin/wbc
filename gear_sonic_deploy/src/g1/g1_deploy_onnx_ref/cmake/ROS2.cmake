# =============================================================================
# ROS2 Configuration
# =============================================================================
# This file contains ROS2 library discovery for the main executable

if(NOT rclcpp_FOUND OR NOT std_msgs_FOUND)
  message(WARNING "ROS2.cmake included but ROS2 not found")
  return()
endif()

message(STATUS "Configuring ROS2 support...")

# =============================================================================
# ROS2 Library Discovery
# =============================================================================

# Auto-discover ROS2 include directories dynamically
set(ROS2_INCLUDE_DIRS "")

# Common ROS2 distributions in order of preference (newest first)  
set(ROS2_DISTROS "jazzy" "iron" "humble" "galactic" "foxy" "eloquent" "dashing" "crystal")
set(ROS2_INSTALL_PATHS "/opt/ros" "/usr/local/ros")

set(ROS2_DISTRO_FOUND "")
foreach(install_path ${ROS2_INSTALL_PATHS})
  if(ROS2_DISTRO_FOUND)
    break()
  endif()
  
  foreach(distro ${ROS2_DISTROS})
    set(ros2_include_path "${install_path}/${distro}/include")
    if(EXISTS "${ros2_include_path}")
      set(ROS2_INCLUDE_DIRS "${ros2_include_path}")
      file(GLOB ROS2_INCLUDE_SUBDIRS "${ros2_include_path}/*")
      foreach(subdir ${ROS2_INCLUDE_SUBDIRS})
        if(IS_DIRECTORY ${subdir})
          list(APPEND ROS2_INCLUDE_DIRS ${subdir})
        endif()
      endforeach()
      set(ROS2_DISTRO_FOUND "${distro}")
      set(ROS2_INSTALL_BASE "${install_path}/${distro}")
      message(STATUS "📦 Found ROS2 ${distro} at ${install_path}/${distro}")
      break()
    endif()
  endforeach()
endforeach()

if(NOT ROS2_DISTRO_FOUND)
  message(WARNING "❌ No ROS2 installation found in common locations")
  return()
endif()

list(REMOVE_DUPLICATES ROS2_INCLUDE_DIRS)

# Essential ROS2 libraries for basic pub/sub functionality
set(ESSENTIAL_LIB_PATTERNS
  "librclcpp.so*" "librcl.so*" "librcl_yaml_param_parser.so*"
  "librcpputils.so*" "librcutils.so*" "librmw.so*"
  "librmw_implementation.so*" "librmw_fastrtps_cpp.so*"
  "libstd_msgs__rosidl_typesupport_cpp.so*" "libstd_msgs__rosidl_generator_cpp.so*"
  "librosidl_typesupport_cpp.so*" "librosidl_runtime_cpp.so*" "librosidl_runtime_c.so*"
  "libbuiltin_interfaces__rosidl_typesupport_cpp.so*" "libbuiltin_interfaces__rosidl_generator_cpp.so*"
  "libstatistics_msgs__rosidl_typesupport_cpp.so*" "libstatistics_msgs__rosidl_generator_cpp.so*"
  "libtracetools.so*" "liblibstatistics_collector.so*"
  "libfastrtps.so*" "libfastcdr.so*" "libtinyxml2.so*" "libspdlog.so*" "libfmt.so*"
)

# Use CMAKE_SYSTEM_PROCESSOR for consistency with other CMakeLists
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(SYSTEM_LIB_DIR "/usr/lib/x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(SYSTEM_LIB_DIR "/usr/lib/aarch64-linux-gnu")
else()
  set(SYSTEM_LIB_DIR "/usr/lib")
endif()

# Auto-discover ROS2 libraries using dynamic distro path
set(ROS2_LIBS "")
foreach(pattern ${ESSENTIAL_LIB_PATTERNS})
  # Use dynamically detected ROS2 installation
  file(GLOB ros2_libs "${ROS2_INSTALL_BASE}/lib/${pattern}")
  list(APPEND ROS2_LIBS ${ros2_libs})
  # Also check system library paths
  file(GLOB system_libs "/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/${pattern}")
  list(APPEND ROS2_LIBS ${system_libs})
  file(GLOB usr_system_libs "${SYSTEM_LIB_DIR}/${pattern}")
  list(APPEND ROS2_LIBS ${usr_system_libs})
endforeach()

list(REMOVE_DUPLICATES ROS2_LIBS)
list(LENGTH ROS2_LIBS ROS2_LIB_COUNT)

message(STATUS "✅ ROS2 configuration: ${ROS2_LIB_COUNT} libraries discovered")
