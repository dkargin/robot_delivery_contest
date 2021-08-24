#include "svg.h"
#include "pathfinder.h"

void dumpPathfinder(const SearchGrid& grid, const char* path) {
    int charWidth = 8;
    // Spacing between adjacent blocks.
    int blockSpacing = 1;
    // Size of a single cell.
    int cellWidth = 3;
    // Offsets from left top corner to a "board" with cells.
    int topMargin = 40;
    int leftMargin = 80;

    int dimension = grid.getWidth();
    int canvasWidth = cellWidth * dimension + leftMargin;
    int canvasHeight = cellWidth * dimension + topMargin;
    SvgWriter svg(path, canvasWidth, canvasHeight);
    char msg[255];

    for (int cy = 0; cy < dimension; cy++)
    {
        for (int cx = 0; cx < dimension; cx++)
        {
            int x = leftMargin + cx * cellWidth;
            int y = topMargin + cy * cellWidth;
            const auto* node = grid.getNode(x, y);
            if (grid.isVisited(cx, cy))
            {
                // Draw cost.
                snprintf(msg, sizeof(msg), "%d", node->cost);
                svg.htext(x, y, msg, "");
                // Draw links.
                if (node->parent != Node::InvalidID) {
                    auto parent = grid.getNode(node->parent);
                    auto pt = grid.nodeCoords(parent);
                    int x0 = leftMargin + pt.x * cellWidth;
                    int y0 = topMargin + pt.y * cellWidth;
                    svg.line(x, y, x0, y0, "stroke-width:1;stroke:rgb(0,0,100)");
                }
            }
        }
    }
}