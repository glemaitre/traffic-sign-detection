# Copyright (c) 2015 
# Guillaume Lemaitre (g.lemaitre58@gmail.com)
# Yohan Fougerolle (Yohan.Fougerolle@u-bourgogne.fr)
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

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

# Support c++11
QMAKE_CXXFLAGS += -std=c++11

# Configuration via pkg-config
CONFIG += link_pkgconfig

# Add the library needed
PKGCONFIG += opencv
