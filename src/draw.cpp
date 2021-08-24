#include "draw.h"

void dumpPathfinder(const SearchGrid& grid, const char* path) {
    int charWidth = 8;
    // Spacing between adjacent blocks.
    int blockSpacing = 1;
    // Size of a single cell.
    int cellWidth = 14;
    // Offsets from left top corner to a "board" with cells.
    int bottomMargin = 10;
    int topMargin = 15;
    int leftMargin = 15;

    int dimension = grid.getWidth();
    int canvasWidth = cellWidth * dimension + leftMargin + bottomMargin;
    int canvasHeight = cellWidth * dimension + topMargin + bottomMargin;
    SvgWriter svg(path, canvasWidth, canvasHeight);
    char msg[255];

    const char* borderStyle = "stroke-width:2;stroke:black";
    svg.rect(leftMargin, topMargin, cellWidth * dimension, cellWidth * dimension, "none", borderStyle);

    const char* tickStyle = "stroke-width:1;stroke:black";
    const char* textStyle = "font-size:xx-small";
    constexpr int tickLen = 2;
    constexpr int tickStep = 10;
    // Vertical annotation.
    for (int v = tickStep; v < dimension; v += tickStep)
    {
        int y = topMargin + v * cellWidth;
        snprintf(msg, sizeof(msg), "%d", v);
        svg.htext(0, y, msg, textStyle);
        svg.hline(leftMargin, y- tickLen, tickLen, tickStyle);
    }
    snprintf(msg, sizeof(msg), "%d", dimension);
    svg.htext(0, topMargin + dimension * cellWidth, msg, textStyle);

    // Horizontal annotation.
    for (int h = tickStep; h < dimension; h += tickStep)
    {
        int x = topMargin + h * cellWidth;
        snprintf(msg, sizeof(msg), "%d", h);
        svg.vtext(x, 0, msg, textStyle);
        svg.vline(x - tickLen, topMargin, tickLen, tickStyle);
    }
    snprintf(msg, sizeof(msg), "%d", dimension);
    svg.vtext(leftMargin + dimension * cellWidth, 0, msg, textStyle);

    for (int cy = 0; cy < dimension; cy++)
    {
        for (int cx = 0; cx < dimension; cx++)
        {
            int x = leftMargin + cx * cellWidth;
            int y = topMargin + cy * cellWidth;
            const auto* node = grid.getNode(cx, cy);
            if (grid.isOccupied(cx, cy))
            {
                svg.rect(x, y, cellWidth, cellWidth, "black", "");
            }
            else if (grid.isVisited(cx, cy))
            {
                // Draw cost.
                snprintf(msg, sizeof(msg), "%d", node->cost);
                svg.htext(x, y + cellWidth, msg, textStyle);
                // Draw links.
                if (node->parent != Node::InvalidID) {
                    auto parent = grid.getNode(node->parent);
                    auto pt = grid.nodeCoords(parent);
                    int x0 = leftMargin + pt.x * cellWidth;
                    int y0 = topMargin + pt.y * cellWidth;
                    svg.line(x + cellWidth/2, y + cellWidth / 2, 
                        x0 + cellWidth / 2, y0 + cellWidth / 2, "stroke-width:1;stroke:green");
                }
            }
        }
    }
}