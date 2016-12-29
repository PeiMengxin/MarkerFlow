TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/flow/debug.cpp \
    src/flow/VarFlow.cpp \
    src/flow/OpticalFlow.cpp \
    src/my_serial.cpp \
    src/aruco.cpp \
    src/MarkerWorldCoornidate.cpp \
    src/AttitudePosition.cpp

HEADERS += \
    src/flow/iOpticalFlow.h \
    src/flow/debug.h \
    src/flow/VarFlow.h \
    src/flow/ProfTimer.h \
    src/flow/OpticalFlow.h \
    src/my_serial.h \
    src/MarkerWorldCoornidate.h \
    src/AttitudePosition.h

LIBS    += -lopencv_highgui \
            -lopencv_photo \
            -lopencv_calib3d \
            -lopencv_imgproc \
            -lopencv_stitching \
            -lopencv_contrib \
            -lopencv_legacy \
            -lopencv_superres \
            -lopencv_core \
            -lopencv_ml \
            -lopencv_video \
            -lopencv_features2d \
            -lopencv_nonfree \
            -lopencv_videostab \
            -lopencv_flann \
            -lopencv_objdetect \
            -lopencv_gpu \
            -lopencv_ocl \
            -laruco \
            -lpthread \
            -lserial

