TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH += \
    /opt/ros/melodic/include

SOURCES += \
    src/driver.cpp \
    src/main.cpp \
    src/message.cpp \
    src/ros_node.cpp

DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    package.xml

HEADERS += \
    src/driver.h \
    src/message.h \
    src/ros_node.h
