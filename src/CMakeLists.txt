add_executable(robot_dispatcher
	pathfinder.h
	priority_heap.h
	dispatcher.h
	dispatcher.cpp
	problem_parser.h
	problem_parser.cpp
	svg.h
	svg.cpp
	draw.h
	draw.cpp)

target_compile_definitions(robot_dispatcher PUBLIC -DLOG_STDIO)