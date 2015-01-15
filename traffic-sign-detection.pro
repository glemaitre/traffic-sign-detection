TEMPLATE   = app
CONFIG    += console
QT        -= gui

# Header files
INCLUDES  += ./src/segmentation.h \
             ./src/colorConversion.h \
             ./src/imageProcessing.h \
             ./src/smartOptimisation.h \
             ./src/math_utils.h

# Source files
SOURCES   += ./src/main.cpp \
             ./src/segmentation.cpp \
             ./src/colorConversion.cpp \
             ./src/smartOptimisation.cpp \
             ./src/imageProcessing.cpp

# Configuration via pkg-config
CONFIG += link_pkgconfig

# Add the library needed
PKGCONFIG += opencv
