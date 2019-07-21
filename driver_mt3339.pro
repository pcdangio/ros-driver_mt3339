TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    /opt/ros/melodic/include

SOURCES += \
    src/driver.cpp \
    src/message.cpp

DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    package.xml

HEADERS += \
    src/driver.h \
    src/message.h
