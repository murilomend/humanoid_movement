#-------------------------------------------------
#
# Project created by QtCreator 2018-01-19T21:31:56
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = robot_qt
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        #main.cpp \
        #mainwindow.cpp
        src/main.cpp \
        src/main_window.cpp \
        src/qnode.cpp \
        src/qcustomplot.cpp

HEADERS += \
        #mainwindow.h
        include/robot_qt/main_window.hpp \
        include/robot_qt/qnode.hpp \
        include/robot_qt/qcustomplot.hpp \
        include/robot_qt/graphicswindow.h

FORMS += \
        #mainwindow.ui
        ui/main_window.ui


#resources/images.qrc
#resources/images/icon.png


