## We use g++-13, for that setup the environment variable
##   export QMAKE_CXX=g++-13
## before running qmake, or inside QtCreator, set Projects > Build & Run > Build Environment
##


QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++20

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

# Define the dependencies
LIBS += -jack -sndfile -boost_program_options -fftw3f -qcustomplot

SOURCES += \
    freq_filter.cpp \
    jack_client.cpp \
    main.cpp \
    mainwindow.cpp \
    dsp_client.cpp \
    qcustomplot.cpp \
    sndfile_thread.cpp

HEADERS += \
    freq_filter.h \
    jack_client.h \
    mainwindow.h \
    dsp_client.h \
    prealloc_ringbuffer.h \
    prealloc_ringbuffer.tpp \
    qcustomplot.h \
    sndfile_thread.h

FORMS += \
    mainwindow.ui
