QT -= gui
QT += serialport \
    widgets
CONFIG += c++17 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        abstractreader.cpp \
        framereader.cpp \
        main.cpp \
        numberformat.cpp \
        samplepack.cpp \
        sinkdata.cpp \
        sourcedata.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    abstractreader.h \
    framereader.h \
    numberformat.h \
    samplepack.h \
    sinkdata.h \
    sourcedata.h
