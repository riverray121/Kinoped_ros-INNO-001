#! /bin/sh

# CATKIN_IGNORE file can be placed in projects to ignore, but they can't be built until the file is deleted.
# Exclude packages that are not supported yet or not required by default using CATKIN_BLACKLIST_PACKAGES
#  CATKIN_BLACKLIST_PACKAGES is sticky, so even running "catkin_make" without this option still ignores those
#    packages, so you need to run catkin_make with CATKIN_BLACKLIST_PACKAGES set to only packages to be excluded.
#
. /opt/ros/noetic/setup.sh
# catkin_make -DCATKIN_BLACKLIST_PACKAGES="roswww;rvizweb;kinoped_msg_test"
catkin_make
