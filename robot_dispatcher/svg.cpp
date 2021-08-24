/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description: Part of Robusta non-linear optimization toolkit.
 *
 * Author: Dmitry Kargin wx872845 kargin.dmitry@huawei.com
 */

#include <vector>
#include "draw_svg.h"


std::vector<RGB> palette = {};

/// Get color from palette.
RGB getElementColor(int index)
{
    if (index < palette.size()) {
        return palette[index];
    }
    int curSize = (int)palette.size();
    palette.resize(index + 1);
    int clrStart = 20;
    int clrRange = 200;
    for (int i = curSize; i < palette.size(); i++) {
        auto& rgb = palette[i];
        // Do several attempts to pick a color if result is too "white".
        do {
            // This is non-encryption scenario and we can freely use rand() here.
            rgb.r = clrStart + rand() % clrRange;
            rgb.g = clrStart + rand() % clrRange;
            rgb.b = clrStart + rand() % clrRange;
        } while ((RGB(255, 255, 255) - rgb).sqrt() < 30);
    }
    return palette[index];
}

/// Get mixed RGB color for intersection between class "a" and class "b"
RGB getMixedElementColor(int a, int b)
{
    RGB clrA = getElementColor(a);
    RGB clrB = getElementColor(b);
    return {0.5 * (clrA.r + clrB.r), 0.5 * (clrA.g + clrB.g), 0.5 * (clrA.b + clrB.b)};
}

#ifdef USE_DEPRECATED
void dumpProfileSVG(const ResidualBlockStorage& problem, const char* output)
{
    int charWidth = 8;
    // Spacing between adjacent blocks.
    int blockSpacing = 1;
    // Size of a single cell.
    int cellWidth = 3;
    // Offsets from left top corner to a "board" with cells.
    int topMargin = 40;
    int leftMargin = 80;

    std::set<uint64_t> constantParameters;
    int numVariables = (int)problem.getUniqueParametersSize();
    const auto& orderingHandler = problem.getOrderingHandler();
    std::vector<std::vector<uint64_t>> adjacencyList(numVariables);
    orderingHandler.prepareAdjacencyListAndConstantParameters(adjacencyList, constantParameters);

    const auto& classInfo = problem.getVariablesClassesInfo();
    const auto& geometries = classInfo.classGeometry;
    const auto& reducedLayout = problem.getReducedMarkup();
    const auto& varClassesIds = problem.getVariablesClassesInfo().getVariableClasses();
    int reducedDimension = (int)problem.getReducedStateSize();

    int cellStep = blockSpacing + cellWidth;

    int maxClassNameLen = 0;
    for (int c = 0; c < classInfo.classData.size(); c++) {
        const auto& cls = classInfo.classData[c];
        if (cls.name.size() > maxClassNameLen)
            maxClassNameLen = (int)cls.name.size();
    }

    // Adjusting topMargin and leftMargin to fit our class names.
    // We are using monospace 10px font.
    int maxTextLen = maxClassNameLen * 6;
    if (maxTextLen > topMargin) {
        topMargin = maxTextLen;
    }
    if (maxTextLen > leftMargin) {
        leftMargin = maxTextLen;
    }

    SvgWriter svg(output, leftMargin + cellStep * reducedDimension + blockSpacing,
        topMargin + cellStep * reducedDimension + blockSpacing);

    // Registering block configurations. It will save some time in future.
    for (int cx = 0; cx < classInfo.classData.size(); cx++) {
        const auto& geomx = geometries[cx];
        for (int cy = 0; cy < classInfo.classData.size(); cy++) {
            const auto& geomy = geometries[cy];
            int height = geomy.dimension * cellStep - blockSpacing;
            int width = geomx.dimension * cellStep - blockSpacing;
            std::string name = std::string("B") + std::to_string(cx) + "X" + std::to_string(cy);
            auto clr = getMixedElementColor(cx, cy);
            auto && tag = svg.symbol(name.c_str(), width, height);
            svg.rect(0, 0, width, height, clr.text().c_str(), nullptr);
        }
    }

    // Registering symbols for vertical annotation
    for (int c = 0; c < classInfo.classData.size(); c++) {
        const auto& geom = geometries[c];
        const auto& cls = classInfo.classData[c];
        std::string name = std::string("V") + std::to_string(c);
        int height = geom.dimension * cellStep;
        auto && tag = svg.symbol(name.c_str(), leftMargin, height);
        int offset = cellStep;
        std::string color("color:");
        color += getElementColor(c).text();
        for (int i = 0; i < geom.reducedDimension; i++) {
            int strokeWidth = 3;
            if (i == geom.reducedDimension - 1)
                strokeWidth = leftMargin / 2;
            svg.hline(leftMargin - strokeWidth, offset, strokeWidth,
                "stroke-width:1;stroke:rgb(0,0,100)");
            offset += cellStep;
        }
        svg.htext(leftMargin - 3, offset / 2, cls.name.c_str(), color.c_str(), "end");
    }

    // Registering symbols for horizontal annotation
    for (int c = 0; c < classInfo.classData.size(); c++) {
        const auto& geom = geometries[c];
        const auto& cls = classInfo.classData[c];
        std::string name = std::string("H") + std::to_string(c);
        int width = geom.dimension * cellStep - cellWidth;
        auto && tag = svg.symbol(name.c_str(), width, topMargin);
        int offset = cellStep;
        for (int i = 1; i < geom.reducedDimension; i++) {
            int strokeWidth = 3;
            if (i == geom.reducedDimension - 1) {
                strokeWidth *= 3;
            }
            offset += cellStep;
        }
        svg.vtext(offset / 2, 0, cls.name.c_str(), "");
    }

    auto orderingMode = orderingHandler.getOrderingMode();
    std::string orderingText = "unordered";
    if (orderingMode == ParameterSorting::cuthill) {
        orderingText = "cuthill";
    }
    if (orderingMode == ParameterSorting::minHeapBased) {
        orderingText = "minheap";
    } else if (orderingMode == ParameterSorting::priority) {
        orderingText = "priority";
    }
    svg.htext(0, topMargin / 2, orderingText.c_str(), "");
    // Border for all blocks
    svg.rect(leftMargin, topMargin, cellStep * reducedDimension + blockSpacing,
        cellStep * reducedDimension + blockSpacing, "none", "stroke-width:1;stroke:rgb(0,0,0)");

    // Vertical annotation.
    for (int v = 0; v < numVariables; v++) {
        auto cid = varClassesIds[v];
        const auto& geom = geometries[cid];
        auto y = topMargin + reducedLayout[v] * cellStep;
        svg.use(std::string("#V") + std::to_string(cid), 0, (int)y);
    }

    // Horizontal annotation.
    for (int v = 0; v < numVariables; v++) {
        auto cid = varClassesIds[v];
        const auto& geom = geometries[cid];
        auto x = leftMargin + reducedLayout[v] * cellStep;
        svg.use(std::string("#H") + std::to_string(cid), (int)x, 0);
    }

    char blockName[128];
    // Drawing cells.
    for (int vy = 0; vy < adjacencyList.size(); vy++) {
        const auto& row = adjacencyList[vy];
        auto y = topMargin + reducedLayout[vy] * cellStep;
        auto cidy = varClassesIds[vy];
        for (auto vx : row) {
            auto cidx = varClassesIds[vx];
            auto x = leftMargin + reducedLayout[vx] * cellStep;
            if (reducedLayout[vx] > reducedLayout[vy]) {
                continue;
            }

            // C++11 version of snprintf ensures zero at the end of the buffer.
            std::snprintf(blockName, sizeof(blockName) - 1, "#B%dX%d", cidx, cidy);
            svg.use(blockName, (int)x, (int)y);
        }

        // Diagonal elements
        auto x = leftMargin + reducedLayout[vy] * cellStep;
        // C++11 version of snprintf ensures zero at the end of the buffer.
        std::snprintf(blockName, sizeof(blockName) - 1, "#B%dX%d", cidy, cidy);
        svg.use(blockName, (int)x, (int)y);
    }
}

#endif