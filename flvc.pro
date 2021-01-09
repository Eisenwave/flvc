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

INCLUDEPATH += include voxelio/include

HEADERS += include/flvc/flvattrib.hpp \
           include/flvc/flvccodec.hpp \
           include/flvc/flvcconfig.hpp \
           include/flvc/flvcconst.hpp \
           include/flvc/permutation.hpp \
           include/flvc/svo.hpp \
           include/flvc/3rd_party/args.hpp
           
SOURCES += src/flvccodec.cpp \
           src/main.cpp
