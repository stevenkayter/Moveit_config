#-------------------------------------------------
#
# Project created by QtCreator 2016-08-05T11:17:55
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
    ../src/kinect_planner.py \
    ../launch/move_neck.launch \
    ../src/kinect_controller.py \
    ../launch/move_group.launch \
    ../launch/planning_context.launch
