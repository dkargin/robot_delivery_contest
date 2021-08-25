#pragma once

#include <iostream>

#include "pathfinder.h"

#ifdef LOG_SVG
#include "draw.h"
#endif

/// Parser for input data:
///  - obtstacle map
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
            m_in >> order->start.x >> order->start.y
                >> order->finish.x >> order->finish.y;
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
            std::vector<Point2> ptSelection;
            for (int row = 0; row < m_map.dimension; row++) {
                for (int col = 0; col < m_map.dimension; col++) {
                    int index = m_map.index(col, row);
                    auto* node = m_search.getNode(col, row);
                    if (node->waveId == searchId && !m_map.isOccupied(col, row))
                    {
                        availableTiles.erase(index);
                        selection.push_back(index);
                        ptSelection.push_back(Point2(col, row));
                    }
                }
            }
#ifdef LOG_STDIO
            std::cout << "Generated island with " << selection.size() << " elements" << std::endl;
#endif
            m_map.islands.push_back(std::move(selection));
            m_map.ptIslands.push_back(std::move(ptSelection));
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
            order.deliveryPath.timeSearch = MS(timeStart, timeEnd);
            auto traceStart = Clock::now();
            m_search.tracePath(order.start.x, order.start.y, order.deliveryPath.points);
            auto traceEnd = Clock::now();
            order.deliveryPath.timeTrace = MS(traceStart, traceEnd);
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
            std::cout << "Order #" << o << " length=" << order.deliveryDistance()
                    << " wave in " << order.deliveryPath.timeSearch.count() << "ms"
                    << " trace in " << order.deliveryPath.timeTrace.count() << std::endl;
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
#ifdef LOG_SVG
            PathDrawer drawer(m_search, svgPath);
            drawer.drawGrid(true, false, true);
            drawer.drawTargets(m_groupPred.targets);
            drawer.drawStarts({ order->start });
#endif
            if (waveResult == SearchGrid::WaveResult::Collapsed)
                throw std::runtime_error("Multiwave has collapsed");

            int nearestRobot = -1;
            int nearestDistance = m_search.getWidth() * m_search.getWidth();
            for (int r: robotCandidates)
            {
                Robot& robot = m_robots[r];
                m_search.tracePath(robot.pos.x, robot.pos.y, robot.tmpPath);
                int distance = (int)robot.tmpPath.size();
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
            robot.orders.push_back(orderId);
            std::swap(order->approachPath.points, robot.tmpPath);
#ifdef LOG_SVG
            drawer.drawPath(rorder->approachPath.points);
#endif
            removeFreeOrder(i);
        }
    }

    void prepareTurn()
    {
        for (auto& robot : m_robots)
            robot.beginStep();
    }

    void moveRobots(int tick)
    {
        for (int r = 0; r < m_robots.size(); r++)
        {
            auto& robot = m_robots[r];
            if (robot.orders.empty())
            {
                // TODO: should we clean up command log?
                continue;
            }
            int orderId = robot.orders.front();
            Order* order = m_orders[orderId].get();
            assert(order);
            // We have some order but we have not started processing it.
            if (robot.state == Robot::State::Idle)
            {
                // Start moving towards pickup point.
                robot.state = Robot::State::MovingStart;
                robot.pathPosition = 0;
                assert(robot.pos == order->approachPath[0]);
            }

            if (robot.state == Robot::State::MovingStart)
            {
                // Move across the path
                if (robot.pathPosition < order->approachPath.points.size() - 1)
                {
                    robot.pathPosition++;
                    robot.stepTo(order->approachPath[robot.pathPosition], tick);
                }
                else
                {
                    // Final position.
                    robot.pathPosition = 0;
                    robot.commands[tick] = Robot::Command::Pick;
                    robot.state = Robot::State::MovingFinish;
                    assert(robot.pos == order->deliveryPath[0]);
                    assert(robot.pos == order->start);
                }
            }
            else if (robot.state == Robot::State::MovingFinish)
            {
                // Move across the path
                if (robot.pathPosition < order->deliveryPath.points.size() - 1)
                {
                    robot.pathPosition++;
                    robot.stepTo(order->deliveryPath[robot.pathPosition], tick);
                }
                else
                {
                    // Final position.
                    robot.pathPosition = 0;
                    robot.commands[tick] = Robot::Command::Down;
                    assert(robot.pos == order->finish);
                    robot.state = Robot::State::Idle;
                    robot.orders.pop_front();
                }
            }
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
            std::string out;
            for (auto cmd: robot.commands)
            {
                char ch = Robot::cmdToChar(cmd);
                out.push_back(ch);
            }
            std::cout << out.c_str() << std::endl;
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
