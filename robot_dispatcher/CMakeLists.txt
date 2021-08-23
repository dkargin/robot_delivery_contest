add_executable(robot_dispatcher
	pathfinder.h
	priority_heap.h
	robot_dispatcher.cpp
	problem_parser.h
	problem_parser.cpp)

target_compile_definitions(robot_dispatcher PUBLIC -DLOG_STDIO)