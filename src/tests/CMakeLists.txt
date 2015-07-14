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

include_directories(${PROJECT_SOURCE_DIR}/)
include_directories(SYSTEM ${GTEST_INCLUDE_DIRS})

include_directories(${external_includes})

add_subdirectory(integration)
add_subdirectory(unit)

### TEST MODULE COMMON ###
add_executable(test_common
                tests_all.cpp
		${srcs_common_all}
                )

target_link_libraries(test_common
                      ${GTEST_BOTH_LIBRARIES}
                      common
                      ${external_libs}
		      )


### TEST MODULE IMG_PROC ###
add_executable(test_img_proc
                tests_all.cpp
		${srcs_img_proc_all}
                )

target_link_libraries(test_img_proc
                      ${GTEST_BOTH_LIBRARIES}
                      img_processing
                      ${external_libs}
		      )

### TEST MODULE OPT ###
add_executable(test_opt
                tests_all.cpp
		${srcs_opt_all}
                )

target_link_libraries(test_opt
                      ${GTEST_BOTH_LIBRARIES}
                      optimization
                      ${external_libs}
		      )

### TEST MODULE INTEGRATION ###
add_executable(test_integration
                tests_all.cpp
		${srcs_integration_all}
                )

target_link_libraries(test_integration
                      ${GTEST_BOTH_LIBRARIES}
                      common
                      img_processing
                      optimization
                      ${external_libs}
		      )

### TEST INTEGRATION AND MODULES ###
add_executable(test_all
                tests_all.cpp
		${srcs_common_all}
		${srcs_img_proc_all}
                ${srcs_integration_all}
                )

target_link_libraries(test_all
                      ${GTEST_BOTH_LIBRARIES}
                      common
                      img_processing
                      optimization
                      ${external_libs}
		      )




