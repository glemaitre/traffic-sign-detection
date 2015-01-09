TEMPLATE   = app
CONFIG    += console
QT        -= gui

# Header files
INCLUDES  += ./src/ihls.h \
             ./src/nhs.h \
             ./src/math_utils.h

# Source files
SOURCES   += ./src/main.cpp \
             ./src/ihls.cpp \
             ./src/nhs.cpp \
             ./src/math_utils.cpp

# Configuration via pkg-config
CONFIG += link_pkgconfig

# Add the library needed
PKGCONFIG += opencv
