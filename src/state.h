#pragma once

#include <cstdint>
#include <vector>
#include <set>
#include <chrono>

using Clock = std::chrono::steady_clock;
using Duration = std::chrono::milliseconds;

inline auto MS(const Clock::time_point& from, const Clock::time_point& to)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(to - from);
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
};

struct Path
{
    std::vector<Point2> points;
    /// Time for wave search.
    Duration timeSearch;
    /// Time for backtracing.
    Duration timeTrace;
};

struct Order
{
    Point2 start;
    Point2 finish;
    /// Price of specific order.
    int price;
    /// A time when this order has been placed.
    int time;
    /// ID of a robot which has taken this order.
    int robotId = -1;
    /// Path from start to finish.
    Path path;

    /// Check if order is assigned to any robot.
    bool isAssigned() const
    {
        return robotId >= 0;
    }

    int distance() const
    {
        return path.points.size();
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
    Point2 pos;

    enum class Command : char {
        Idle,
        Up,
        Down,
        Left,
        Right,
        Pick,
        Drop,
    };

    static char cmdToChar(Command cmd)
    {
        switch(cmd)
        {
            case Command::Up:
                return 'U';
            case Command::Down:
                return 'D';
            case Command::Left:
                return 'L';
            case Command::Right:
                return 'R';
            case Command::Idle:
                return 'S';
            case Command::Pick:
                return 'T';
            case Command::Drop:
                return 'P';
        }
        return 'S';
    }

    /// Sequence of commands for current tick.
    std::vector<Command> commands;
    /// Identifiers of assigned orders.
    std::vector<int> orders;

    Robot()
    {
        commands.resize(60, Command::Idle);
    }

    std::vector<Point2> tmpPath;
};

struct Map
{
    /// Binary rows.
    std::vector<Row> occupancy;

    /// Cached number of obstacles.
    int numObstacles = 0;

    int dimension = 0;

    /// Returns an index of a tile (x, y)
    int index(Coord x, Coord y) const
    {
        return x + y * dimension;
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
};

