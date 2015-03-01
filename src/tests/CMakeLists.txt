include_directories(${PROJECT_SOURCE_DIR}/)
include_directories(SYSTEM ${GTEST_INCLUDE_DIRS})

include_directories(${external_includes})

add_subdirectory(integration)

add_executable(test_all
                tests_all.cpp
                ${srcs_integration_all}
                )

target_link_libraries(test_all
                      ${GTEST_BOTH_LIBRARIES}
                      common
                      ${external_libs}
)
