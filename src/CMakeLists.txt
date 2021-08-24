add_executable(robot_dispatcher
	pathfinder.h
	dispatcher.h
	dispatcher.cpp
	problem_parser.h
	svg.h
	svg.cpp
	draw.h
	draw.cpp)

target_compile_definitions(robot_dispatcher PUBLIC -DLOG_STDIO)