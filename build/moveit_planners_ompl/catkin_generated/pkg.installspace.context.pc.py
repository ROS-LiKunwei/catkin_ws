# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/lkw/catkin_ws/devel/include/ompl-1.6;/usr/include;/usr/include/eigen3".split(';') if "${prefix}/include;/home/lkw/catkin_ws/devel/include/ompl-1.6;/usr/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;moveit_core;roscpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmoveit_ompl_interface;/home/lkw/catkin_ws/devel/lib/libompl.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;-lode".split(';') if "-lmoveit_ompl_interface;/home/lkw/catkin_ws/devel/lib/libompl.so;/usr/lib/x86_64-linux-gnu/libboost_serialization.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;-lode" != "" else []
PROJECT_NAME = "moveit_planners_ompl"
PROJECT_SPACE_DIR = "/home/lkw/catkin_ws/install"
PROJECT_VERSION = "1.0.11"
