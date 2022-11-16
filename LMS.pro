QT       += core gui
QT += widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14
INCLUDEPATH += /usr/lib/gcc/x86_64-linux-gnu/9/include/
INCLUDEPATH += ./inc/

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv4
}

SOURCES += \
    sdk/src/arch/linux/net_serial.cpp \
    sdk/src/arch/linux/net_socket.cpp \
    sdk/src/arch/linux/timer.cpp \
    sdk/src/hal/thread.cpp \
    sdk/src/rplidar_driver.cpp \
    src/joypad.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/parameters.cpp \
    src/sim_env.cpp \
    src/simulation.cpp \
    src/work.cpp

HEADERS += \
    inc/joypad.h \
    inc/levmarq.h \
    inc/mainwindow.h \
    inc/parameters.h \
    inc/sim_env.h \
    inc/simulation.h \
    inc/slam.hpp \
    inc/work.h \
    sdk/include/rplidar.h \
    sdk/include/rplidar_cmd.h \
    sdk/include/rplidar_driver.h \
    sdk/include/rplidar_protocol.h \
    sdk/include/rptypes.h \
    sdk/src/arch/linux/arch_linux.h \
    sdk/src/arch/linux/net_serial.h \
    sdk/src/arch/linux/thread.hpp \
    sdk/src/arch/linux/timer.h \
    sdk/src/hal/abs_rxtx.h \
    sdk/src/hal/assert.h \
    sdk/src/hal/byteops.h \
    sdk/src/hal/event.h \
    sdk/src/hal/locker.h \
    sdk/src/hal/socket.h \
    sdk/src/hal/thread.h \
    sdk/src/hal/types.h \
    sdk/src/hal/util.h \
    sdk/src/rplidar_driver_TCP.h \
    sdk/src/rplidar_driver_impl.h \
    sdk/src/rplidar_driver_serial.h \
    sdk/src/sdkcommon.h

FORMS += \
    forms/mainwindow.ui \
    forms/parameters.ui


TARGET = LMS

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES +=

RESOURCES += ./res/resources.qrc
