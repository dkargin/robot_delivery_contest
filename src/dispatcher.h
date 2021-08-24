#pragma once

#include "pathfinder.h"

class Dispatcher
{
public:
    Dispatcher(Map& map)
        :m_search(map), map(map)
    {

    }

    /// Process connectivity components in a map.
    void processIslands()
    {
        // Indexes of available tiles.
        std::set<int> availableTiles;

        // 0. Build a list of free cells. It takes considerable time in Debug.
        for (int row = 0; row < map.dimension; row++) {
            for (int col = 0; col < map.dimension; col++) {
                auto index = map.index(col, row);
                if (!map.isOccupied(col, row))
                    availableTiles.insert(index);
            }
        }

        if (availableTiles.empty())
            return;
        EmptySearchPredicate ep;
        // 1. Estimate connectivity components. We can have disconnected map.
        do {
            auto index = *availableTiles.begin();
            auto pt = map.indexToCoord(index);
            auto searchId = m_search.beginSearch();
            m_search.addNode(pt.x, pt.y, 0);
            m_search.runWave(ep);
            std::vector<int> selection;
            for (int row = 0; row < map.dimension; row++) {
                for (int col = 0; col < map.dimension; col++) {
                    auto index = map.index(col, row);
                    auto* node = m_search.getNode(col, row);
                    if (node->waveId == searchId)
                    {
                        availableTiles.erase(index);
                        selection.push_back(index);
                    }
                }
            }
#ifdef LOG_STDIO
            std::cout << "Generated island with " << selection.size() << " elements" << std::endl;
#endif
            map.islands.push_back(std::move(selection));
        } while (!availableTiles.empty());
    }

    void calculateInitialPositions(int maxSteps, int maxOrders, int orderPrice, int robotPrice)
    {
        // Average time for delivery.
        int avgTimeToDeliver = 2 * map.dimension;
        int orderDelay = 60 * maxSteps / maxOrders;

        int needRobots = avgTimeToDeliver / orderDelay;
        if (needRobots < 1)
            needRobots = 1;
        int maxRevenue = maxOrders * orderPrice;
        int expectedRevenue = maxRevenue - needRobots * robotPrice - avgTimeToDeliver * maxOrders;
#ifdef LOG_STDIO
        std::cout << "Avg order delay = " << orderDelay
            << ". Avg delivery time = " << avgTimeToDeliver << " steps."
            << " Need " << needRobots << " robots" << std::endl;
        std::cout << "Max revenue = " << maxRevenue << ", expected revenue = " << expectedRevenue << std::endl;
#endif
        robots.resize(needRobots);
        for (int i = 0; i < robots.size(); i++)
        {
            int islandIndex = rand() % map.islands.size();
            const auto& island = map.islands[islandIndex];
            // Choosing a random available position for now.
            int positionIndex = rand() % island.size();
            robots[i].pos = map.indexToCoord(positionIndex);
        }
    }

    void publishInitialPositions()
    {
        std::cout << robots.size() << std::endl;
        for (const auto& robot : robots)
        {
            std::cout << robot.pos.x << " " << robot.pos.y << std::endl;
        }
    }

public:
    SearchGrid m_search;
    Map& map;
    std::vector<Robot> robots;
    std::vector<std::unique_ptr<Order>> orders;
};
