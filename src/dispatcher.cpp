#include <fstream>

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
    auto timeStart = Clock::now();
    Dispatcher dispatcher(map);
    dispatcher.processIslands();
    dispatcher.calculateInitialPositions(parser.getMaxSteps(), parser.getMaxOrders(),
        parser.getOrderPrice(), parser.getRobotPrice());
    dispatcher.publishInitialPositions();
    int hitIterations = -1;
    constexpr int maxTimeMs = 18000;

    for (int step = 0; step < parser.getMaxSteps(); step++)
    {
        // Calculate paths for each new request.
        int added = parser.parseStepRequests(dispatcher.m_orders, 60*step);

        // Process tasks and them to robots.
        dispatcher.processNewOrders(added);

        for (int tick = 0; tick < 60; tick++)
            dispatcher.moveRobots(step, tick);

        dispatcher.publishRobots();

        auto now = Clock::now();
        if (MS(timeStart, now).count() > maxTimeMs && hitIterations == -1)
        {
            hitIterations = step;
        }
    }

    auto timeFinish = Clock::now();
#ifdef LOG_STDIO
    std::cout << "Done in " << MS(timeStart, timeFinish).count() << "ms." << std::endl;
    std::cout << "Revenue=" << dispatcher.m_revenue << ". Loss=" << dispatcher.m_revenueLoss << std::endl;
    if (hitIterations != -1)
    {
        std::cout << "Managed to process only " << hitIterations
                << " steps out of " << parser.getMaxSteps()
                << " (" << (100*hitIterations / parser.getMaxSteps()) << "%)" << std::endl;
    }
#endif
    return 0;
}
