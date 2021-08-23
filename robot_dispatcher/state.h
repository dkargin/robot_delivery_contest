#pragma once

#include <cstdint>
#include <vector>
#include <set>

/// A single row in a compact occupancy map.
using Row = std::vector<bool>;

/// A type for all coordinates.
using Coord = uint16_t;

struct Order
{
    Coord sx;
    Coord sy;
    Coord fx;
    Coord fy;
    // Price of specific order.
    uint16_t price;
    // A time when this order has been placed.
    uint16_t time;
};

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

struct Robot {
    Coord x;
    Coord y;
};

struct State
{
    std::vector<Robot> robots;
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

    void indexToCoord(int index, Coord& x, Coord& y) const
    {
        y = index / dimension;
        x = index % dimension;
    }

    /// Check if a tile (x, y) is occupied.
    bool isOccupied(Coord x, Coord y) const {
        return occupancy[y][x];
    }

    /// Indexes of available tiles.
    std::set<int> availableTiles;

    std::vector<std::vector<int>> islands;
};