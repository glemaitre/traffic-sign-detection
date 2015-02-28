file(GLOB_RECURSE common_sources *.cpp *.cc)
file(GLOB_RECURSE common_headers *.h *.hpp)

include_directories(${external_includes})

add_library(common STATIC
        ${common_sources}
        ${common_headers}
)

target_link_libraries(common ${external_libs})
