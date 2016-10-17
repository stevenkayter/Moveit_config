#-------------------------------------------------
#
# Project created by QtCreator 2016-08-23T09:32:49
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
    ../../launch/move_neck.launch \
    ../../launch/planning_context.launch \
    ../../src/kinect_controller.py \
    ../../src/kinect_planner.py
