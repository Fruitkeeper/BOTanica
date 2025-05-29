cmake_minimum_required(VERSION 3.0.2)
project(robomaster_driver)

# Use catkin_simple for simplicity
find_package(catkin_simple REQUIRED)

catkin_simple()

# Install Python scripts
cs_install_scripts(
    src/pathPlanner.py  # Add only the scripts you have
)

# ✅ Install the launch folder
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# Export the package
cs_export()
