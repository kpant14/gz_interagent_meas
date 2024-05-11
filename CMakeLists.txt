cmake_minimum_required(VERSION 3.8)
project(gz_interagent_meas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find the gz_transport library
find_package(gz-transport12
	#REQUIRED COMPONENTS core
	NAMES
		gz-transport12
)
find_package(gz-msgs9 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-sensors7 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(gz-rendering7 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
if(gz-transport12_FOUND)
	add_compile_options(-frtti -fexceptions)

	set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

	if(GZ_TRANSPORT_VER GREATER_EQUAL 12)
		set(GZ_TRANSPORT_LIB gz-transport${GZ_TRANSPORT_VER}::core)
	endif()
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# install cpp executables
# include cpp "include" directory
include_directories(include)

# create cpp executable
add_executable(interagent_range_pub src/interagent_range_pub.cpp)
ament_target_dependencies(interagent_range_pub rclcpp
                        gz-transport12 gz-sensors7 gz-rendering7 gz-msgs9 gz-math7 tf2
                        tf2_ros std_msgs geometry_msgs)                        
target_include_directories(interagent_range_pub PUBLIC ${Eigen3_INCLUDE_DIRS})


install(TARGETS
interagent_range_pub
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
