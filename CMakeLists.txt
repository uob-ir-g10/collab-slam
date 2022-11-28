# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(socspioneer)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

catkin_install_python(PROGRAMS nav/motion_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY 
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
