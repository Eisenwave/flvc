TARGET = flvc
TEMPLATE = lib

SUBDIRS = voxelio

CONFIG -= qt
CONFIG += c++17 strict_c strict_c++

CONFIG(release, debug|release) {
    DEFINES += VXIO_RELEASE
}
CONFIG(debug, debug|release) {
    DEFINES += VXIO_DEBUG
}

LIBS += -Lvoxelio \
        -lvoxelio \
        -lstdc++fs

INCLUDEPATH += voxelio/include

HEADERS += flvattrib.hpp \
           flvccodec.hpp \
           flvcconfig.hpp \
           flvcconst.hpp \
           permutation.hpp \
           svo.hpp
           
SOURCES += flvccodec.cpp \
           main.cpp
