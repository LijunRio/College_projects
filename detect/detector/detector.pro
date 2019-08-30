QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += /usr/local/include \
INCLUDEPATH += /usr/local/include/opencv \
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/lib/arm-linux-gnueabihf/libv4l/v4l1compat.so
CONFIG   += c++11 (Qt5)

LIBS += -L/usr/local/lib \
     -lopencv_core \
     -lopencv_highgui \
     -lopencv_imgproc \
     -lopencv_videoio \
     -lopencv_imgcodecs \
     -lopencv_features2d \
     -lopencv_flann \
     -lopencv_dnn

HEADERS += \
    caffe_model.h \
    caffe_use.h \
    cctag_detector.h \
    coordinate_process.h \
    detector_methods.h \
    read_class.h \
    serialport.h \
    Structure.h \
    v4l2_set.h

SOURCES += \
    caffe_model.cpp \
    cctag_detector.cpp \
    coordinate_process.cpp \
    detector_methods.cpp \
    main.cpp \
    read_camera.cpp \
    read_image.cpp \
    serialport.cpp \
    Structure.cpp \
    v4l2_set.cpp
