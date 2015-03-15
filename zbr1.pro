#-------------------------------------------------
#
# Project created by QtCreator 2015-03-13T10:39:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = zbr1
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    kinematics.cpp \
    trajectory.cpp

HEADERS  += mainwindow.h \
    kinematics.h \
    trajectory.h \
    typedefs.h

FORMS    += mainwindow.ui
