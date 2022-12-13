QT       += core gui serialport multimedia multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    adddoordialog.cpp \
    addlightdialog.cpp \
    addwindowdialog.cpp \
    camera.cpp \
    doordialog.cpp \
    lightsdialog.cpp \
    main.cpp \
    mainwindow.cpp \
    secdialog.cpp \
    windowdialog.cpp

HEADERS += \
    adddoordialog.h \
    addlightdialog.h \
    addwindowdialog.h \
    camera.h \
    doordialog.h \
    lightsdialog.h \
    mainwindow.h \
    secdialog.h \
    windowdialog.h

FORMS += \
    adddoordialog.ui \
    addlightdialog.ui \
    addwindowdialog.ui \
    camera.ui \
    doordialog.ui \
    lightsdialog.ui \
    mainwindow.ui \
    secdialog.ui \
    windowdialog.ui


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
