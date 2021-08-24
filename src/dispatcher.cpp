#include <fstream>


#include "problem_parser.h"

#include "dispatcher.h"

int main(int argc, const char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: problem_parser <inFile> [<outFile>]" << std::endl;
        return -1;
    }

    const char* inPath = argv[1];
    std::ifstream in(inPath);
    if (!in) {
        std::cerr << "Failed to open input file " << inPath << std::endl;
        return -1;
    }

#ifdef OUT_FILE
    const char* outPath = argv[2];
    std::ofstream outFile(outPath);
    if (!out) {
        std::cerr << "Failed to open output file " << outPath << std::endl;
        return -1;
    }
#endif
    std::ostream& out = std::cout;

    ProblemParser parser(in);
    Map map;
    if (!parser.parseObstacleMap(map)) {
        std::cerr << "Failed to parse obstacle map" << std::endl;
        return -1;
    }

    if (!parser.parseRequestsHeader()) {
        std::cerr << "Failed to parse requests" << std::endl;
        return -1;
    }

    Dispatcher dispatcher(map);
    dispatcher.processIslands();
    dispatcher.calculateInitialPositions(parser.getMaxSteps(), parser.getMaxOrders(),
        parser.getOrderPrice(), parser.getRobotPrice());
    dispatcher.publishInitialPositions();

    for (int step = 0; step < parser.getMaxSteps(); step++)
    {
        // Calculate paths for each new request.
        int added = parser.parseStepRequests(dispatcher.m_orders, 60*step);

        dispatcher.processNewOrders(added);

        // Assign tasks to robots.
        // TODO: for each task: find shortest path from task start to each of robots (or their finish points).
        for (int tick = 0; tick < 60; tick++)
        {
            // TODO: Make steps.
        }

        dispatcher.publishRobots();
    }
    return 0;
}
