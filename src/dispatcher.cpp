#include <fstream>

#include "problem_parser.h"

#include "dispatcher.h"
#include "draw.h"

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

    // TODO: Process 
    for (int step = 0; step < parser.getMaxSteps(); step++)
    {
        // Calculate paths for each new request.
        int added = parser.parseStepRequests(dispatcher.orders, step);

        TargetSearchPredicate pred(dispatcher.m_search);
        for (size_t o = dispatcher.orders.size() - added; o < dispatcher.orders.size(); o++)
        {
            Order& order = *dispatcher.orders[o];
            dispatcher.m_search.beginSearch();
            // Doing reverse search.
            dispatcher.m_search.addNode(order.finish.x, order.finish.y, 0);
            pred.target = order.start;
            auto result = dispatcher.m_search.runWave(pred);
            assert(result == SearchGrid::WaveResult::Goal);
            order.path = std::make_unique<Path>();
            dispatcher.m_search.tracePath(order.start.x, order.start.y, order.path->points);
#ifdef LOG_STDIO
            std::cout << "Order #" << o << " length=" << order.distance() << std::endl;
#endif
        }
        // TODO: Calculate data.

        for (int tick = 0; tick < 60; tick++)
        {
            // TODO: Make steps.
            // TODO: Write steps for each robot.
        }
    }
}
