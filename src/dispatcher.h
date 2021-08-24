#pragma once

#include "pathfinder.h"

#ifdef LOG_SVG
#include "draw.h"
#endif

class Dispatcher
{
public:
    Dispatcher(Map& map)
        :m_search(map), m_map(map)
    {

    }

    /// Process connectivity components in a map.
    void processIslands()
    {
        // Indexes of available tiles.
        std::set<int> availableTiles;

        // 0. Build a list of free cells. It takes considerable time in Debug.
        for (int row = 0; row < m_map.dimension; row++) {
            for (int col = 0; col < m_map.dimension; col++) {
                auto index = m_map.index(col, row);
                if (!m_map.isOccupied(col, row))
                    availableTiles.insert(index);
            }
        }

        if (availableTiles.empty())
            return;
        EmptySearchPredicate ep;
        // 1. Estimate connectivity components. We can have disconnected map.
        do {
            auto index = *availableTiles.begin();
            auto pt = m_map.indexToCoord(index);
            auto searchId = m_search.beginSearch();
            m_search.addNode(pt.x, pt.y, 0);
            m_search.runWave(ep);
            std::vector<int> selection;
            for (int row = 0; row < m_map.dimension; row++) {
                for (int col = 0; col < m_map.dimension; col++) {
                    auto index = m_map.index(col, row);
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
            m_map.islands.push_back(std::move(selection));
        } while (!availableTiles.empty());
    }

    void calculateInitialPositions(int maxSteps, int maxOrders, int orderPrice, int robotPrice)
    {
        // Average time for delivery.
        int avgTimeToDeliver = 2 * m_map.dimension;
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
        m_robots.resize(needRobots);
        for (int i = 0; i < m_robots.size(); i++)
        {
            int islandIndex = rand() % m_map.islands.size();
            const auto& island = m_map.islands[islandIndex];
            // Choosing a random available position for now.
            int positionIndex = rand() % island.size();
            m_robots[i].pos = m_map.indexToCoord(positionIndex);
        }
    }

    void processNewOrders(int added)
    {
        TargetSearchPredicate pred(m_search);

        for (size_t o = m_orders.size() - added; o < m_orders.size(); o++)
        {
            m_freeOrders.push_back(o);
            Order& order = *m_orders[o];
            // Finding path from start to finish.
            auto timeStart = Clock::now();
            m_search.beginSearch();
            // Doing reverse search.
            m_search.addNode(order.finish.x, order.finish.y, 0);
            pred.target = order.start;
            auto result = m_search.runWave(pred);
            assert(result == SearchGrid::WaveResult::Goal);
            auto timeEnd = Clock::now();
            order.path.timeSearch = MS(timeStart, timeEnd);
#ifdef LOG_SVG
            char svgPath[255];
            std::snprintf(svgPath, sizeof(svgPath), "tree_%d.svg", (int)o);
            dumpPathfinder(m_search, svgPath);
#endif
            auto traceStart = Clock::now();
            m_search.tracePath(order.start.x, order.start.y, order.path.points);
            auto traceEnd = Clock::now();
            order.path.timeTrace = MS(traceStart, traceEnd);
#ifdef LOG_STDIO
            std::cout << "Order #" << o << " length=" << order.distance()
                    << " wave in " << order.path.timeSearch.count() << "ms"
                    << " trace in " << order.path.timeTrace.count() << std::endl;
#endif
        }
        // TODO: Find best robots for this task.

        for (int i = 0; i < m_freeOrders.size(); i++)
        {
            Order* order = m_orders[m_freeOrders[i]].get();
            if (!order)
            {
                // Swap and remove from list.
                std::swap(m_freeOrders[i], m_freeOrders.back());
                m_freeOrders.pop_back();
                continue;
            }

        }
    }

    void publishInitialPositions()
    {
        std::cout << m_robots.size() << std::endl;
        for (const auto& robot : m_robots)
        {
            std::cout << robot.pos.x << " " << robot.pos.y << std::endl;
        }
    }

    /// Publish robot commands for last simulation minute.
    void publishRobots()
    {
        for (const auto& robot: m_robots)
        {
            for (auto cmd: robot.commands)
            {
                char ch = Robot::cmdToChar(cmd);
                std::cout << ch;
            }
            std::cout << std::flush;
        }
    }

public:
    std::vector<std::unique_ptr<Order>> m_orders;

protected:
    SearchGrid m_search;
    Map& m_map;
    std::vector<Robot> m_robots;
    // A list of IDs of free orders.
    std::vector<int> m_freeOrders;
};
