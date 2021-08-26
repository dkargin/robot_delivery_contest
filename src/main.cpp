#include <fstream>

#define LOCAL_TEST
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
    auto simStart = PClock::now();
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
    Dispatcher dispatcher(map, simStart);
    dispatcher.runSimulation(parser, true);
    return 0;
}
