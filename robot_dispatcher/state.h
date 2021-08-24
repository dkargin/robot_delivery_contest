#pragma once

#include <cstdint>
#include <vector>
#include <set>

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
};

struct Order
{
    Point2 start;
    Point2 finish;
    // Price of specific order.
    int price;
    // A time when this order has been placed.
    int time;
    // ID of a robot which has taken this order.
    int robotId;
    // Path from start to finish.
    std::unique_ptr<Path> path;

    int distance() const
    {
        return path ? (int)path->points.size() : -1;
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

