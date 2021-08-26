#pragma once

#include <cstdint>
#include <vector>
#include <set>
#include <map>
#include <list>
#include <chrono>
#include <memory>
#include <algorithm>

using Clock = std::chrono::steady_clock;
using PClock = Clock;// std::chrono::high_resolution_clock;
using Duration = std::chrono::milliseconds;

inline auto MS(const Clock::time_point& from, const Clock::time_point& to)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(to - from);
}

template <class _Rep, class _Period>
inline auto MS(const std::chrono::duration<_Rep, _Period>& duration)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
}

/// A single row in a compact occupancy map.
using Row = std::vector<bool>;

/// A type for all coordinates.
using Coord = uint16_t;


struct Point2
{
    Coord x = 0;
    Coord y = 0;

    Point2() {}
    Point2(Coord x, Coord y) : x(x), y(y) {}
    Point2(const Point2& p) : x(p.x), y(p.y) {}

    friend bool operator == (const Point2& a, const Point2& b)
    {
        return a.x == b.x && a.y == b.y;
    }

    friend bool operator != (const Point2& a, const Point2& b)
    {
        return a.x != b.x || a.y != b.y;
    }

    friend bool operator<(const Point2 &lhs, const Point2 &rhs)
    {
        return (lhs.x < rhs.x) || ((lhs.x == rhs.x) && (lhs.y < rhs.y));
    }
};

struct Path
{
    std::vector<Point2> points;
    /// Time for wave search.
    Duration timeSearch;
    /// Time for backtracing.
    Duration timeTrace;

    const Point2& operator[](int index) const
    {
        return points[index];
    }

    int distance() const
    {
        return (int)points.size();
    }

    void reverse()
    {
        std::reverse(points.begin(), points.end());
    }
};

struct Order
{
    Point2 start;
    Point2 finish;
    /// A time when this order has been placed.
    uint64_t time = 0;
    /// ID of a robot which has taken this order.
    int robotId = -1;

    int siteIndex = -1;

    /// Path from start to finish.
    Path deliveryPath;

    /// Check if order is assigned to any robot.
    bool isAssigned() const
    {
        return robotId >= 0;
    }

    int deliveryDistance() const
    {
        return (int)deliveryPath.points.size();
    }
};


struct Robot {
    enum class State
    {
        Idle,
        /// Moving to delivery start.
        MovingStart,
        /// Moving to delivery finish.
        MovingFinish,
        /// Moving home.
        MovingHome
    };
    State state = State::Idle;

    Point2 pos;
    /// Sequence of commands for current tick.
    char commands[61] = {};

    /// Path from current position of a robot to pickup point.
    Path approachPath;

    /// List of sites with orders.
    std::list<int> sites;
    /// Identifiers of assigned orders.
    int order = -1;
    /// Position on current path.
    int pathPosition = -1;

    Robot()
    {
        commands[60] = 0;
        clearCommands();
    }

    /// Clears command buffer.
    void clearCommands()
    {
        for (int i = 0; i < 60; i++)
            commands[i] = 'S';
    }

    /// Moves robot to a specified point.
    void stepTo(const Point2& newPos, int step)
    {
        char cmd = 'S';
        if (newPos.x == pos.x && newPos.y == pos.y + 1)
        {
            cmd = 'D';
        }
        else if (newPos.x == pos.x && newPos.y == pos.y - 1)
        {
            cmd = 'U';
        }
        else if (newPos.x == pos.x + 1 && newPos.y == pos.y)
        {
            cmd = 'R';
        }
        else if (newPos.x == pos.x - 1 && newPos.y == pos.y)
        {
            cmd = 'L';
        }
        else
        {
            throw std::runtime_error("Invalid step");
        }
        pos = newPos;
        commands[step] = cmd;
    }

    bool isIdle() const
    {
        return state == State::Idle && order == -1 && sites.empty();
    }
};

struct Map
{
    enum MoveFlags
    {
        Center = 1,
        Left = 1<<2,
        Right = 1<<3,
        Up = 1<<4,
        Down = 1<<5,
    };

    /// Binary rows.
    std::vector<Row> occupancy;

    std::vector<unsigned char> bakedOccupancy;

    /// Cached number of obstacles.
    int numObstacles = 0;

    int dimension = 0;

    /// Returns an index of a tile (x, y)
    int index(Coord x, Coord y) const
    {
        return (int)x + (int)y * dimension;
    }

    Point2 indexToCoord(int index) const
    {
        Point2 result;
        result.y = index / dimension;
        result.x = index % dimension;
        return result;
    }

    /// Check if a tile (x, y) is occupied.
    bool isOccupied(Coord x, Coord y) const {
        return occupancy[y][x];
    }

    std::vector<std::vector<int>> islands;

    void bakeOccupancy()
    {
        bakedOccupancy.resize(dimension*dimension, 0);
        int i = 0;
        for (int y = 0; y < dimension; y++)
        {
            for (int x = 0; x < dimension; x++, i++)
            {
                auto& cell = bakedOccupancy[i];
                if (isOccupied(x, y))
                    cell |= Center;
                if (x == 0 || isOccupied(x-1, y))
                    cell |= Left;
                if (y == 0 || isOccupied(x, y-1))
                    cell |= Up;
                if (x == dimension-1 || isOccupied(x+1, y))
                    cell |= Right;
                if (y == dimension-1 || isOccupied(x, y+1))
                    cell |= Down;
            }
        }
    }
};

