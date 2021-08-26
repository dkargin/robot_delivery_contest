add_executable(robot_dispatcher
	pathfinder.h
	dispatcher.h
	main.cpp
	svg.h
	svg.cpp
	draw.h
	draw.cpp)

option(LOG_SVG "Draw debug data to SVG files" OFF)

set(DEFINES -DLOG_STDIO)
if (LOG_SVG)
	list(APPEND DEFINES -DLOG_SVG)
endif ()

target_compile_definitions(robot_dispatcher PUBLIC ${DEFINES})

add_executable(final final.cpp)