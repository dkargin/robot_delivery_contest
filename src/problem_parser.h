#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include <cassert>

#include "state.h"

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

