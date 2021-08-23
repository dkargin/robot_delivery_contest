#include <fstream>

#include "problem_parser.h"
#include "pathfinder.h"

class Dispatcher
{
public:
    Dispatcher(Map& map)
        :m_search(map)
    {

    }

    void calculateInitialPositions(Map& map)
    {
        // 0. Build a list of free cells. It takes considerable time in Debug.
        for (int row = 0; row < map.dimension; row++) {
            for (int col = 0; col < map.dimension; col++) {
                auto index = map.index(col, row);
                if (!map.isOccupied(col, row))
                    map.availableTiles.insert(index);
            }
        }

        if (map.availableTiles.empty())
            return;
        EmptySearchPredicate ep;
        // 1. Estimate connectivity components. We can have disconnected map.
        do {
            Coord x, y;
            auto index = *map.availableTiles.begin();
            map.indexToCoord(index, x, y);
            auto searchId = m_search.beginSearch();
            m_search.addNode(x, y, 0);
            m_search.runWave(ep);
            std::vector<int> selection;
            for (int row = 0; row < map.dimension; row++) {
                for (int col = 0; col < map.dimension; col++) {
                    auto index = map.index(col, row);
                    auto* node = m_search.getNode(col, row);
                    if (node->waveId == searchId)
                    {
                        map.availableTiles.erase(index);
                        selection.push_back(index);
                    }
                }
            }
#ifdef LOG_STDIO
            std::cout << "Generated island with " << selection.size() << " elements" << std::endl;
#endif
            map.islands.push_back(std::move(selection));
        } while (!map.availableTiles.empty());
        // 2. Generate positions in each connectivity domain.
    }

    SearchGrid m_search;
};

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

    std::vector<Order> requests;
    if (!parser.parseRequestsHeader(requests)) {
        std::cerr << "Failed to parse requests" << std::endl;
        return -1;
    }

    Dispatcher dispatcher(map);
    dispatcher.calculateInitialPositions(map);
}
