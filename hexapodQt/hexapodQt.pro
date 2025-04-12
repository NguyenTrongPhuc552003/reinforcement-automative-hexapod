QT += core gui network widgets quick qml quickcontrols2

TARGET = hexapodQt
TEMPLATE = app

CONFIG += c++17 qml

DEFINES += QT_DEPRECATED_WARNINGS

# Input sources
SOURCES += \
    main.cpp \
    mainwindow.cpp \
    hexapodconnection.cpp \
    hexapodprotocol.cpp \
    hexapodvisualization.cpp \
    hexapodcontext.cpp

HEADERS += \
    mainwindow.h \
    hexapodconnection.h \
    hexapodprotocol.h \
    hexapodvisualization.h \
    hexapodcontext.h

FORMS += \
    mainwindow.ui

# Resources
RESOURCES += \
    res.qrc \
    qml.qrc

# Default rules for deployment
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

# Link with hexapod library if available (for local development/testing)
unix {
    exists(../app/lib/libhexapod.so) {
        LIBS += -L../app/lib -lhexapod
        INCLUDEPATH += ../app/inc
        DEPENDPATH += ../app/inc
    }
}
