#pragma once

#include "pathfinder.h"

#ifdef LOG_SVG
#include "draw.h"
#endif

class Dispatcher
{
public:
    Dispatcher(Map& map)
        :m_search(map), m_groupPred(m_search), m_map(map)
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
            m_freeOrders.push_back((int)o);
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
            auto traceStart = Clock::now();
            m_search.tracePath(order.start.x, order.start.y, order.path.points);
            auto traceEnd = Clock::now();
            order.path.timeTrace = MS(traceStart, traceEnd);
#ifdef LOG_SVG
            char svgPath[255];
            std::snprintf(svgPath, sizeof(svgPath), "tree_o%d.svg", (int)o);
            {
                PathDrawer drawer(m_search, svgPath);
                drawer.drawGrid(true, false, true);
                drawer.drawStarts({ order.start });
                drawer.drawTargets({ order.finish });
                drawer.drawPath(order.path.points);
            }
#endif

#ifdef LOG_STDIO
            std::cout << "Order #" << o << " length=" << order.distance()
                    << " wave in " << order.path.timeSearch.count() << "ms"
                    << " trace in " << order.path.timeTrace.count() << std::endl;
#endif
        }

        std::vector<int> robotCandidates;
        robotCandidates.reserve(m_robots.size());
        // Find best robots for this task.
        // Using greedy algorithm for now.
        for (int i = 0; i < m_freeOrders.size(); i++)
        {
            int orderId = m_freeOrders[i];
            Order* order = m_orders[orderId].get();
            if (!order)
            {
                removeFreeOrder(i);
                continue;
            }

            m_groupPred.clear();
            robotCandidates.clear();
            for (int r = 0; r < m_robots.size(); r++)
            {
                const Robot& robot = m_robots[r];
                // Skip robots which are occupied now.
                if (!robot.orders.empty())
                {
                    // TODO: can queue tasks.
                    continue;
                }
                m_groupPred.addTarget(robot.pos);
                robotCandidates.push_back(r);
            }

            if (robotCandidates.empty())
            {
#ifdef LOG_STDIO
                std::cout << "No free candidates for task " << orderId << std::endl;
#endif
                break;
            }

            m_search.beginSearch();
            m_search.addNode(order->start.x, order->start.y, 0);
            auto waveResult = m_search.runWave(m_groupPred);

            char svgPath[255];
            std::snprintf(svgPath, sizeof(svgPath), "tree_mo%d.svg", (int)orderId);
            {
                PathDrawer drawer(m_search, svgPath);
                drawer.drawGrid(true, false, true);
                drawer.drawTargets(m_groupPred.targets);
                drawer.drawStarts({ order->start });
            }
            if (waveResult == SearchGrid::WaveResult::Collapsed)
                throw std::runtime_error("Multiwave has collapsed");

            int nearestRobot = -1;
            int nearestDistance = m_search.getWidth() * m_search.getWidth();
            for (int r: robotCandidates)
            {
                Robot& robot = m_robots[r];
                m_search.tracePath(robot.pos.x, robot.pos.y, robot.tmpPath);
                int distance = robot.tmpPath.size();
                if (distance < nearestDistance || nearestRobot == -1)
                {
                    nearestRobot = r;
                    nearestDistance = distance;
                }
            }

#ifdef LOG_STDIO
            std::cout << "Assigning task " << orderId << " to robot " << nearestRobot << std::endl;
#endif
            m_robots[nearestRobot].orders.push_back(orderId);
            removeFreeOrder(i);
        }
    }

    // Remove free order at specified index.
    void removeFreeOrder(int i)
    {
        std::swap(m_freeOrders[i], m_freeOrders.back());
        m_freeOrders.pop_back();
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
    GroupSearchPredicate m_groupPred;
    Map& m_map;
    std::vector<Robot> m_robots;
    // A list of IDs of free orders.
    std::vector<int> m_freeOrders;
};
