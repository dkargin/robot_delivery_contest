#include "svg.h"
#include "pathfinder.h"

struct PathDrawer
{
    PathDrawer(const SearchGrid& grid, const char* path);

    void drawGrid(bool drawOccupancy, bool drawCosts, bool drawTree);

    void drawTargets(const std::vector<Point2>& targets);
    void drawStarts(const std::vector<Point2>& starts);
    void drawPath(const std::vector<Point2>& path);

protected:
    std::unique_ptr<SvgWriter> m_svg;
    const SearchGrid& m_grid;
};
