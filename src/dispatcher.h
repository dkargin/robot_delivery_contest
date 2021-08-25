#pragma once

#include <iostream>
#include <cstdio>
#include "pathfinder.h"

#ifdef LOG_SVG
#include "draw.h"
#endif

/// Parser for input data:
///  - obstacle map
///  - delivery requests
class ProblemParser
{
public:
    enum class State {
        Initial,
        ParsedMap,
        ParsedRequestsNumber,
        Done,
    };

    ProblemParser(std::istream& in)
        :m_in(in)
    {
    }

    bool parseObstacleMap(Map& map) {
        if (m_state != State::Initial)
            return false;

        m_in >> m_mapSize >> m_maxTips >> m_robotPrice;
        constexpr int maxMapSize = 2000;
        if (m_mapSize > maxMapSize)
            return false;
        constexpr int maxTips = 50000;
        if (m_maxTips > maxTips)
            return false;

        map.occupancy.resize(m_mapSize);
        map.dimension = m_mapSize;

        std::string row;
        for (int r = 0; r < m_mapSize; r++) {
            m_in >> row;
            if (row.size() != m_mapSize) {
                std::cerr << "Unexpected row length: " << row.size()
                    << " vs expected " << m_mapSize << std::endl;
                return false;
            }
            auto& occupancyRow = map.occupancy[r];
            occupancyRow.resize(m_mapSize);
            for (int c = 0; c < m_mapSize; c++) {
                if (row[c] == '#') {
                    occupancyRow[c] = true;
                    map.numObstacles++;
                }
                else if (row[c] == '.')
                    occupancyRow[c] = false;
                else {
                    std::cerr << "Unexpected occupancy code at row=" << r <<
                        " col=" << c << std::endl;
                    return false;
                }
            }
        }

        m_state = State::ParsedMap;
        return true;
    }

    /// Parses number of input requests and world steps.
    bool parseRequestsHeader() {
        assert(m_state == State::ParsedMap);
        m_in >> m_numSteps >> m_numOrders;
        constexpr int maxSteps = 100000;
        if (m_numSteps > maxSteps)
            return false;
        constexpr int maxOrders = 10000000;
        if (m_numOrders > maxOrders)
            return false;
        m_state = State::ParsedRequestsNumber;
        return true;
    }

    int getMaxOrders() const
    {
        return m_numOrders;
    }

    int getOrderPrice() const
    {
        return m_maxTips;
    }

    int getMaxSteps() const
    {
        return m_numSteps;
    }

    int getRobotPrice() const
    {
        return m_robotPrice;
    }

    /// Parses requests for a current step.
    int parseStepRequests(std::vector<std::unique_ptr<Order>>& requests, int tick) {
        assert(m_state == State::ParsedRequestsNumber);
        int numOrders = 0;
        m_in >> numOrders;
        for (int i = 0; i < numOrders; i++)
        {
            auto order = std::make_unique<Order>();
            // Start_r, Start_c, Finish_r, Finish_c
            m_in >> order->start.y >> order->start.x
                >> order->finish.y >> order->finish.x;
            order->start.x -= 1;
            order->start.y -= 1;
            order->finish.x -= 1;
            order->finish.y -= 1;
            order->time = tick;
            requests.push_back(std::move(order));
        }
        return numOrders;
    }

protected:
    /// Cell size of a map (N).
    int m_mapSize = 0;
    /// Max tips per request.
    int m_maxTips = 0;
    /// Price of a single robot.
    int m_robotPrice = 0;

    /// Number of simulation steps.
    int m_numSteps = 0;
    /// Number of orders.
    int m_numOrders = 0;

    std::istream& m_in;
    State m_state = State::Initial;
};


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
        m_map.bakeOccupancy();
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
                    int index = m_map.index(col, row);
                    auto* node = m_search.getNode(col, row);
                    if (node->waveId == searchId && !m_map.isOccupied(col, row))
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

        m_orderPrice = orderPrice;
        m_orders.reserve(maxOrders);

        int needRobots = avgTimeToDeliver / orderDelay;
        needRobots *= 2;
        if (needRobots < 1)
            needRobots = 1;
        if (needRobots > 100)
            needRobots = 100;
        int maxRevenue = maxOrders * orderPrice;
        int expectedRevenue = maxRevenue - needRobots * robotPrice - avgTimeToDeliver * maxOrders;
        m_parkCost = needRobots * robotPrice;
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
            int positionIndex = island[rand() % island.size()];
            auto pos = m_map.indexToCoord(positionIndex);
            m_robots[i].pos = pos;
            assert(!m_map.isOccupied(pos.x, pos.y));
        }
    }

    void processNewOrders(int added)
    {
        TargetSearchPredicate pred(m_search);

        for (size_t o = m_orders.size() - added; o < m_orders.size(); o++)
        {
            m_freeOrders.insert((int)o);
            Order& order = *m_orders[o];
            int siteIndex = getSiteIndex(order.start);
            m_sites[siteIndex].orders.insert((int)o);
            order.siteIndex = siteIndex;
        }

        std::vector<int> robotCandidates;
        robotCandidates.reserve(m_robots.size());
        // Find best robots for this task.
        // Using greedy algorithm for now.
        std::vector<int> assignedOrders;
        for (int orderId : m_freeOrders)
        {
            Order* order = m_orders[orderId].get();
            assert(order);
            if (!order)
            {
                assignedOrders.push_back(orderId);
                continue;
            }

            m_groupPred.clear();
            robotCandidates.clear();
            for (int r = 0; r < m_robots.size(); r++)
            {
                const Robot& robot = m_robots[r];
                // Skip robots which are occupied now.
                if (!robot.isIdle())
                {
                    continue;
                }
                m_groupPred.addTarget(robot.pos);
                robotCandidates.push_back(r);
            }
            m_groupPred.addTarget(order->finish);

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

#ifdef LOG_SVG
            char svgPath[255];
            std::snprintf(svgPath, sizeof(svgPath), "tree_mo%d.svg", (int)orderId);
            PathDrawer drawer(m_search, svgPath);
            drawer.drawGrid(true, false, true);
            drawer.drawTargets(m_groupPred.targets);
            drawer.drawStarts({ order->start });
#endif
            if (waveResult == SearchGrid::WaveResult::Collapsed)
                throw std::runtime_error("Multiwave has collapsed");

            m_search.tracePath(order->finish.x, order->finish.y, order->deliveryPath.points);
            order->deliveryPath.reverse();

            int nearestRobot = -1;
            int nearestDistance = m_search.getWidth() * m_search.getWidth();
            for (int r: robotCandidates)
            {
                Robot& robot = m_robots[r];
                const auto* node = m_search.getNode(robot.pos.x, robot.pos.y);
                int distance = node->cost;
                if (distance < nearestDistance || nearestRobot == -1)
                {
                    nearestRobot = r;
                    nearestDistance = distance;
                }
            }

#ifdef LOG_STDIO
            std::cout << "Assigning task " << orderId << " to robot " << nearestRobot << std::endl;
#endif
            auto& robot = m_robots[nearestRobot];
            robot.sites.push_back(order->siteIndex);
            m_search.tracePath(robot.pos.x, robot.pos.y, robot.approachPath.points);
#ifdef LOG_SVG
            drawer.drawPath(order->approachPath.points);
#endif
            assignedOrders.push_back(orderId);
        }
        for (auto orderId : assignedOrders)
            m_freeOrders.erase(orderId);
    }

    void moveRobots(int step, int tick)
    {
        for (int r = 0; r < m_robots.size(); r++)
        {
            auto& robot = m_robots[r];
            if (robot.order == -1 && robot.sites.empty())
            {
                continue;
            }

            // We have some order but we have not started processing it.
            if (robot.state == Robot::State::Idle)
            {
                // Start moving towards pickup point.
                if (!robot.sites.empty())
                {
                    robot.state = Robot::State::MovingStart;
                    robot.pathPosition = 0;
                    assert(robot.pos == robot.approachPath[0]);
                }
            }

            if (robot.state == Robot::State::MovingStart)
            {
                // Move across the path
                if (robot.pathPosition < robot.approachPath.points.size() - 1)
                {
                    robot.pathPosition++;
                    robot.stepTo(robot.approachPath[robot.pathPosition], tick);
                }
                else
                {
                    // Final position.
                    robot.pathPosition = 0;
                    assert(!robot.sites.empty());
                    int siteIndex = robot.sites.front();
                    robot.sites.pop_front();
                    Site& site = m_sites[siteIndex];
                    int orderIndex = site.popOrder();
                    if (orderIndex >= 0)
                    {
                        auto* order = m_orders[orderIndex].get();
                        order->siteIndex = -1;
                        assert(order);
                        assert(robot.pos == order->deliveryPath[0]);
                        assert(robot.pos == order->start);
                        robot.order = orderIndex;
                        robot.commands[tick] = 'T';
                        robot.state = Robot::State::MovingFinish;
                    }
                    else
                    {
                        robot.commands[tick] = 'S';
                        robot.state = Robot::State::Idle;
                    }
                }
            }
            else if (robot.state == Robot::State::MovingFinish)
            {
                int orderIndex = robot.order;
                auto* order = m_orders[orderIndex].get();
                // Move across the path
                if (robot.pathPosition < order->deliveryPath.points.size() - 1)
                {
                    robot.pathPosition++;
                    robot.stepTo(order->deliveryPath[robot.pathPosition], tick);
                }
                else
                {
                    // Final position.
                    robot.pathPosition = -1;
                    robot.commands[tick] = 'P';
                    assert(robot.pos == order->finish);
                    robot.state = Robot::State::Idle;
                    robot.order = -1;
                    closeOrder(step, tick, orderIndex);
                }
            }
        }
    }

    void closeOrder(int step, int tick, int order)
    {
        uint64_t latency = step*60 + tick - m_orders[order]->time;
        m_revenueLoss += latency;
        uint64_t revenue = 0;
        if (latency < m_orderPrice)
        {
            revenue = (m_orderPrice - latency);
        }

        m_revenue += revenue;

        m_orders[order].reset();
#ifdef LOG_STDIO
        std::cout << "Order " << order << " is closed. Revenue =" << revenue << ", loss=" << latency << std::endl;
#endif
    }

    void publishInitialPositions()
    {
        std::cout << m_robots.size() << std::endl;
        for (const auto& robot : m_robots)
        {
            std::cout << robot.pos.y + 1 << " " << robot.pos.x + 1 << std::endl;
        }
    }

    /// Publish robot commands for last simulation minute.
    void publishRobots()
    {
        for (auto& robot: m_robots)
        {
            puts(robot.commands);
            robot.clearCommands();
        }
    }

public:
    std::vector<std::unique_ptr<Order>> m_orders;

    struct Site
    {
        Point2 pos;
        // Unassigned orders from this site.
        std::set<int> orders;

        int popOrder()
        {
            if (orders.empty())
                return -1;
            int order = *orders.begin();
            orders.erase(order);
            return order;
        }
    };

    std::vector<Site> m_sites;

    std::map<Point2, int> m_siteMap;

    int getSiteIndex(const Point2& pt)
    {
        auto it = m_siteMap.find(pt);
        if (it == m_siteMap.end())
        {
            Site site;
            site.pos = pt;
            m_sites.push_back(site);
            m_siteMap[pt] = m_sites.size() - 1;
            return m_sites.size() - 1;
        }
        return it->second;
    }

    // Total revenue.
    int64_t m_revenue = 0;
    // Loss of revenue due to long delivery.
    int64_t m_revenueLoss = 0;
    // Cost of all allocated robots.
    int64_t m_parkCost = 0;

protected:
    SearchGrid m_search;
    GroupSearchPredicate m_groupPred;
    Map& m_map;
    std::vector<Robot> m_robots;
    // A list of IDs of free orders.
    std::set<int> m_freeOrders;

    uint64_t m_orderPrice = 0;
};
