include_directories("../common")

include_directories(${external_includes})
#include_directories( ${OpenCV_INCLUDE_DIRS} )

# Create test executables
set(app_programs
	main)

foreach(app ${app_programs})
        add_executable(${app} ${app}.cpp)
        target_link_libraries(${app} common ${external_libs})
endforeach()
