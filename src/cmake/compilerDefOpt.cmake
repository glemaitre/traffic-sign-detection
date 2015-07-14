
# By downloading, copying, installing or using the software you agree to this license.
# If you do not agree to this license, do not download, install,
# copy or use the software.


#                           License Agreement
#                For Open Source Computer Vision Library
#                        (3-clause BSD License)

# Copyright (C) 2015,
# 	  Guillaume Lemaitre (g.lemaitre58@gmail.com),
# 	  Johan Massich (mailsik@gmail.com),
# 	  Gerard Bahi (zomeck@gmail.com),
# 	  Yohan Fougerolle (Yohan.Fougerolle@u-bourgogne.fr).
# Third party copyrights are property of their respective owners.

# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:

#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.

#   * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.

#   * Neither the names of the copyright holders nor the names of the contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.

# This software is provided by the copyright holders and contributors "as is" and
# any express or implied warranties, including, but not limited to, the implied
# warranties of merchantability and fitness for a particular purpose are disclaimed.
# In no event shall copyright holders or contributors be liable for any direct,
# indirect, incidental, special, exemplary, or consequential damages
# (including, but not limited to, procurement of substitute goods or services;
# loss of use, data, or profits; or business interruption) however caused
# and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of
# the use of this software, even if advised of the possibility of such damage.

add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN)


###### Compiler options

set (CMAKE_CXX_FLAGS                "-std=c++11 -Wextra -Wall -Wno-delete-non-virtual-dtor -Werror=return-type")
set (CMAKE_CXX_FLAGS_DEBUG          "-g -O0 -DDEBUG")
set (CMAKE_CXX_FLAGS_RELEASE        "-O3")

if(WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werr")
endif()



if(OPT_ASAN)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    if(NOT EXISTS ${CMAKE_CXX_COMPILER})
        message( FATAL_ERROR "Clang++ not found. " )
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-register -Qunused-arguments")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-const-variable")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")

endif()
