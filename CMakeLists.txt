cmake_minimum_required(VERSION 3.5)
project(chris_ros_turtlebot2)

# Load catkin and all dependencies required for this package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


###########
## Build ##
###########


#############
## Install ##
#############


# Mark anything (useful) else for installation
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY robots
        DESTINATION share/${PROJECT_NAME}
)

install(FILES chris_ros_turtlebot2.config
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/model_spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/chris_ros_turtlebot2.dsv.in")

ament_package()
