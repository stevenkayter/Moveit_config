#-------------------------------------------------
#
# Project created by QtCreator 2016-06-27T09:41:38
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = Boxy_neck
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp

OTHER_FILES += \
    ../src/kinect_controller2.py \
    ../src/kinect_planner2.py \
    ../src/move_group_interface.py \
    ../launch/move_group_mine3.launch \
    ../launch/demo.launch \
    ../launch/move_group.launch \
    ../launch/moveit_planning_execution.launch \
    ../launch/ur3_ros_control_modified.launch \
    ../../../iai_robots/iai_boxy_bringup/launch/boxy_ur3_test.launch
