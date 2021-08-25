#include "draw.h"

namespace {
    constexpr int charWidth = 8;
    // Spacing between adjacent blocks.
    constexpr int blockSpacing = 1;
    // Size of a single cell.
    constexpr int cellWidth = 14;
    // Offsets from left top corner to a "board" with cells.
    constexpr int bottomMargin = 10;
    constexpr int topMargin = 15;
    constexpr int leftMargin = 15;

    const char* borderStyle = "stroke-width:2;stroke:black";
    const char* tickStyle = "stroke-width:1;stroke:black";
    const char* textStyle = "font-size:xx-small";
    const char* pathTreeStyle = "stroke-width:1;stroke:green";
    constexpr int tickLen = 2;
    constexpr int tickStep = 10;
}

PathDrawer::PathDrawer(const SearchGrid& grid, const char* path)
    :m_grid(grid)
{
    int dimension = grid.getWidth();
    int canvasWidth = cellWidth * dimension + leftMargin + bottomMargin;
    int canvasHeight = cellWidth * dimension + topMargin + bottomMargin;
    m_svg = std::make_unique<SvgWriter>(path, canvasWidth, canvasHeight);


    {
        auto&& targetSymbol = m_svg->symbol("target", cellWidth, cellWidth);
        m_svg->circle(cellWidth / 2, cellWidth / 2, cellWidth / 2 - 1, "red", "stroke:red");
        m_svg->rect(1, 1, cellWidth - 1, cellWidth - 1, "red", "");
    }
    {
        auto&& startSymbol = m_svg->symbol("start", cellWidth, cellWidth);
        m_svg->circle(cellWidth / 2, cellWidth / 2, cellWidth / 2 - 1, "red", "stroke:green");
        m_svg->rect(1, 1, cellWidth - 1, cellWidth - 1, "green", "");
    }
}

void PathDrawer::drawGrid(bool drawOccupancy, bool drawCosts, bool drawTree)
{
    int dimension = m_grid.getWidth();
    char msg[255];
    m_svg->rect(leftMargin, topMargin, cellWidth * dimension, cellWidth * dimension, "none", borderStyle);

    // Vertical annotation.
    for (int v = tickStep; v < dimension; v += tickStep)
    {
        int y = topMargin + v * cellWidth;
        snprintf(msg, sizeof(msg), "%d", v);
        m_svg->htext(0, y, msg, textStyle);
        m_svg->hline(leftMargin, y - tickLen, tickLen, tickStyle);
    }
    snprintf(msg, sizeof(msg), "%d", dimension);
    m_svg->htext(0, topMargin + dimension * cellWidth, msg, textStyle);

    // Horizontal annotation.
    for (int h = tickStep; h < dimension; h += tickStep)
    {
        int x = topMargin + h * cellWidth;
        snprintf(msg, sizeof(msg), "%d", h);
        m_svg->vtext(x, 0, msg, textStyle);
        m_svg->vline(x - tickLen, topMargin, tickLen, tickStyle);
    }
    snprintf(msg, sizeof(msg), "%d", dimension);
    m_svg->vtext(leftMargin + dimension * cellWidth, 0, msg, textStyle);

    for (int cy = 0; cy < dimension; cy++)
    {
        for (int cx = 0; cx < dimension; cx++)
        {
            int x = leftMargin + cx * cellWidth;
            int y = topMargin + cy * cellWidth;
            const auto* node = m_grid.getNode(cx, cy);
            if (m_grid.isOccupied(cx, cy))
            {
                if (drawOccupancy)
                {
                    m_svg->rect(x, y, cellWidth, cellWidth, "black", "");
                }
            }
            else if (m_grid.isVisited(cx, cy))
            {
                // Draw cost.
                if (drawCosts)
                {
                    snprintf(msg, sizeof(msg), "%d", node->cost);
                    m_svg->htext(x, y + cellWidth, msg, textStyle);
                }
                // Draw links.
                if (node->parent != Node::InvalidID && drawTree) {
                    auto parent = m_grid.getNode(node->parent);
                    auto pt = m_grid.nodeCoords(parent);
                    int x0 = leftMargin + pt.x * cellWidth;
                    int y0 = topMargin + pt.y * cellWidth;
                    m_svg->line(x + cellWidth / 2, y + cellWidth / 2,
                        x0 + cellWidth / 2, y0 + cellWidth / 2, pathTreeStyle);
                }
            }
        }
    }
}

void PathDrawer::drawTargets(const std::vector<Point2>& targets)
{
    for (const auto& pt : targets)
    {
        int x = leftMargin + pt.x * cellWidth;
        int y = topMargin + pt.y * cellWidth;
        m_svg->circle(x + cellWidth / 2, y + cellWidth / 2, cellWidth / 2 - 1, "red", "stroke:red");
    }
}

void PathDrawer::drawStarts(const std::vector<Point2>& starts)
{
    for (const auto& pt : starts)
    {
        int x = leftMargin + pt.x * cellWidth;
        int y = topMargin + pt.y * cellWidth;
        //m_svg->use("start", x, y);

        m_svg->rect(x, y, cellWidth - 1, cellWidth - 1, "blue", "stroke:blue");
    }
}

void PathDrawer::drawPath(const std::vector<Point2>& path)
{

}