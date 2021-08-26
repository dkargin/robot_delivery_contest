#pragma once

#include <iostream>
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <vector>
#include <set>
#include <map>
#include <list>
#include <chrono>
#include <memory>
#include <algorithm>

#pragma region Structs
//using Clock = std::chrono::steady_clock;
using PClock = std::chrono::high_resolution_clock;
using Duration = std::chrono::milliseconds;

template <class Clock>
inline auto MS(const std::chrono::time_point<Clock>& from, const std::chrono::time_point<Clock>& to)
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

    friend bool operator<(const Point2& lhs, const Point2& rhs)
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
    // Own id.
    int id = -1;
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
    Order* order = nullptr;
    /// Position on current path.
    int pathPosition = -1;

    int pathLeft = 0;

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
        else if (newPos.x == pos.x && newPos.y == pos.y)
        {
            cmd = 'S';
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
        return state == State::Idle && order == nullptr && sites.empty();
    }

    int timeToFree() const
    {
        if (state == State::MovingFinish)
            return pathLeft;
        return 0;
    }
};

struct Map
{
    enum MoveFlags
    {
        Center = 1,
        Left = 1 << 2,
        Right = 1 << 3,
        Up = 1 << 4,
        Down = 1 << 5,
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
        bakedOccupancy.resize(dimension * dimension, 0);
        int i = 0;
        for (int y = 0; y < dimension; y++)
        {
            for (int x = 0; x < dimension; x++, i++)
            {
                auto& cell = bakedOccupancy[i];
                if (isOccupied(x, y))
                    cell |= Center;
                if (x == 0 || isOccupied(x - 1, y))
                    cell |= Left;
                if (y == 0 || isOccupied(x, y - 1))
                    cell |= Up;
                if (x == dimension - 1 || isOccupied(x + 1, y))
                    cell |= Right;
                if (y == dimension - 1 || isOccupied(x, y + 1))
                    cell |= Down;
            }
        }
    }
};

#pragma endregion

#pragma region Pathfinder
template <class NodeType> struct ContainerTraits;
/// @brief Binary heap container that implements priority queue.
template<class NodeType, class Traits_ = ContainerTraits<NodeType>>
class SearchContainerBH
{
public:
    using Traits = Traits_;
    typedef std::vector<NodeType*> Array;

    /// Creates container for specified max size
    SearchContainerBH(size_t _size)
    {
        maxSize = _size;
        array.reserve(_size);
    }

    /// @brief Check if node is already stored in container
    bool contains(NodeType* node)
    {
        auto id = Traits::getID(node);
        return (id >= 0 && id < array.size()) ? array[id] == node : false;
    }

    /// Get current size.
    size_t getSize() const
    {
        return array.size();
    }

    size_t getMaxSize() const
    {
        return maxSize;
    }

    // @brief Add element to binary heap
    bool push(NodeType* node)
    {
        assert(node != NULL);

        size_t v = array.size();
        size_t u = v;

        Traits::setID(node, v);
        array.push_back(node);

        while (v)
        {
            u = v / 2;

            if (Traits::compare(array[u], array[v]))
            {
                NodeType* tmp = array[u];
                array[u] = array[v];
                array[v] = tmp;
                //fix ID's
                Traits::setID(array[u], u);
                Traits::setID(array[v], v);
                v = u;
            }
            else
                break;
        }
        return true;
    }

    // Removing first node
    NodeType* pop()
    {
        // We swap first node with the last node, shrink and rebuild heap
        unsigned int chLeft = 0;
        unsigned int chRight = 0;
        unsigned int u = 0;
        unsigned int v = 0;

        NodeType* res = array[0];
        array[0] = array.back();

        Traits::setID(array[0], 0);

        array.pop_back();
        size_t size = array.size();

        while (true)
        {
            u = v;
            chLeft = u * 2 + 1;
            chRight = u * 2 + 2;
            if (chRight < size)
            {
                if (Traits::compare(array[u], array[chLeft]))
                    v = chLeft;
                if (Traits::compare(array[v], array[chRight]))
                    v = chRight;
            }
            else if (chLeft < size)
            {
                if (Traits::compare(array[u], array[chLeft]))
                    v = chLeft;
            }
            if (u != v)
            {
                NodeType* tmp = array[u];
                array[u] = array[v];
                array[v] = tmp;

                //fix ID's
                Traits::setID(array[u], u);
                Traits::setID(array[v], v);
            }
            else
                break;
        }
        return res;
    }

    /// Remove all contents
    void clear()
    {
        array.clear();
    }

    size_t getSize()
    {
        return array.size();
    }

public:
    // Max size of a heap.
    size_t maxSize;
    Array array;
};

/// A from in a grid search.
/// N < 2096 -> 11 bits. So we can limit cost to a N*N -> 22 bits
/// int32_t is enough to fit longest path. 
/// The same for NodeId.
struct Node {
    using ID = int32_t;
    using CostType = uint32_t;
    static constexpr ID InvalidID = -1;
    ID parent = InvalidID;
    /// Position in priority queue/binary heap.
    ID heapIndex = InvalidID;
    /// Search ID.
    uint32_t waveId = 0;
    /// Accumulated path cost.
    CostType cost = 0;

#ifdef USE_HEURISTIC
    /// Heuristic estimate of a cost.
    CostType estimate = 0;
#endif

    void reset()
    {
        cost = 0;
#ifdef USE_HEURISTIC
        estimate = 0;
#endif
        parent = InvalidID;
    }
};

/// Wraps access to specific from type
template<> struct ContainerTraits<Node>
{
    using CostType = Node::CostType;

    static CostType getCost(const Node* a)
    {
#ifdef USE_HEURISTIC
        return a->cost + a->estimate;
#else
        return a->cost;
#endif
    }

    /// Implements operator a > b
    static bool compare(const Node* a, const Node* b)
    {
        return getCost(a) > getCost(b);
    }

    static void setID(Node* a, size_t id)
    {
        a->heapIndex = static_cast<Node::ID>(id);
    }

    static int getID(const Node* a)
    {
        return a->heapIndex;
    }
};

/**
 * Tricks used to achieve better performance.
 * 1. Priority queue is implemented by a binary heap. Searches for position in tree
 *   is further improved by intrusive index.
 * 2. Map is not cleared after/before the search. Instead we add special `waveId` marks to each
 *   from and increment it on every new search.
 * 3. Eliminated most of allocations in a search core. It is always uses preallocated data.
 */
class SearchGrid
{
public:
    using NodeID = Node::ID;
    static constexpr uint32_t MaxWaveId = uint32_t(0) - 2;

    SearchGrid(const Map& map) : m_map(map), m_priorityHeap(map.dimension* map.dimension)
    {
        m_width = map.dimension;
        m_grid.resize(m_width * m_width);
    }

    /// Begins a new search.
    uint32_t beginSearch()
    {
        m_priorityHeap.clear();
        if (m_lastWaveId >= MaxWaveId)
        {
            m_lastWaveId = 1;
            clearGrid();
        }
        else
        {
            m_lastWaveId++;
        }
        return m_lastWaveId;
    }

    /// Adds from to search frontier.
    Node* addNode(Coord x, Coord y, int cost, NodeID parentId = Node::InvalidID)
    {
        Node* node = &m_grid[x + m_width * y];
        node->cost = cost;
        node->parent = parentId;
        node->waveId = m_lastWaveId;
        m_priorityHeap.push(node);
        return node;
    }

    Node* addNodeId(NodeID id, int cost, NodeID parentId)
    {
        Node* node = &m_grid[id];
        node->cost = cost;
        node->parent = parentId;
        node->waveId = m_lastWaveId;
        m_priorityHeap.push(node);
        return node;
    }

    /// Clears all the contents of the grid.
    void clearGrid()
    {
        for (auto& node : m_grid)
            node.reset();
    }
    // Get index of a from.
    NodeID nodeIndex(const Node* node) const
    {
        return static_cast<NodeID>(node - &m_grid.front());
    }

    Point2 nodeCoords(const Node* node) const
    {
        auto index = nodeIndex(node);
        return Point2(index % m_width, index / m_width);
    }

    NodeID pointIndex(const Point2& pt) const
    {
        return pt.x + pt.y * m_width;
    }

    bool isOccupied(Coord x, Coord y) const
    {
        return m_map.isOccupied(x, y);
    }

    bool isVisited(Coord x, Coord y) const
    {
        return m_grid[x + y * m_width].waveId == m_lastWaveId;
    }

    bool isVisited(const Node* node) const
    {
        return node->waveId == m_lastWaveId;
    }

    const Node* getNode(Coord x, Coord y) const
    {
        return &m_grid[x + y * m_width];
    }

    const Node* getNode(NodeID id) const
    {
        return &m_grid[id];
    }

    int getWidth() const
    {
        return m_width;
    }

    inline void visitNode(int x, int y, Node* from)
    {
        constexpr int moveCost = 1;
        int newCost = moveCost + from->cost;
        if (isVisited(x, y) || isOccupied(x, y))
            return;
        Node* node = &m_grid[x + y * m_width];
        auto index = nodeIndex(from);
        addNode(x, y, newCost, index);
    }

    inline void visitNodeId(NodeID to, Node* from)
    {
        constexpr int moveCost = 1;
        int newCost = moveCost + from->cost;
        if (m_grid[to].waveId == m_lastWaveId)
            return;
        auto index = nodeIndex(from);
        addNodeId(to, newCost, index);
    }

    void addAdjacentNodes(Node* from)
    {
        auto pt = nodeCoords(from);
        if (pt.x > 0)
            visitNode(pt.x - 1, pt.y, from);
        if (pt.y > 0)
            visitNode(pt.x, pt.y - 1, from);
        if (pt.x < m_width - 1)
            visitNode(pt.x + 1, pt.y, from);
        if (pt.y < m_width - 1)
            visitNode(pt.x, pt.y + 1, from);
    }

    void addAdjacentNodesBaked(Node* from)
    {
        auto fromIndex = nodeIndex(from);
        auto occ = m_map.bakedOccupancy[fromIndex];
        if (!(occ & Map::Right))
            visitNodeId(fromIndex + 1, from);
        if (!(occ & Map::Up))
            visitNodeId(fromIndex - m_width, from);
        if (!(occ & Map::Left))
            visitNodeId(fromIndex - 1, from);
        if (!(occ & Map::Down))
            visitNodeId(fromIndex + m_width, from);
    }

    /// Trace back path from specified point.
    /// It returns reversed path.
    bool tracePath(Coord x, Coord y, std::vector<Point2>& path) const
    {
        path.clear();
        const Node* current = getNode(x, y);
        int iteration = 0;
        while (current)
        {
            auto pt = nodeCoords(current);
            path.push_back(pt);
            const Node* parent = nullptr;
            if (current->parent == Node::InvalidID)
                break;
            parent = &m_grid[current->parent];
            assert(parent != current);
            iteration++;

            assert(iteration < m_width* m_width);
            if (iteration >= m_width * m_width)
                return false;
            current = parent;
        };
        return true;
    }

    enum class WaveResult
    {
        Collapsed,
        Goal,
    };

    template <class Predicate>
    WaveResult runWave(const Predicate& pred)
    {
        int iterations = 0;
        bool findGoal = false;
        while (m_priorityHeap.getSize() > 0)
        {
            Node* top = m_priorityHeap.pop();
            if (!top)
                break;

            if (pred.isGoal(top))
            {
                findGoal = true;
                break;
            }

            //addAdjacentNodes(top);
            // Baked version is about 20-30% faster.
            addAdjacentNodesBaked(top);
            iterations++;
        }
        if (findGoal)
            return WaveResult::Goal;
        return WaveResult::Collapsed;
    }

protected:
    const Map& m_map;
    std::vector<Node> m_grid;

    SearchContainerBH<Node> m_priorityHeap;

    /// Width of a search grid.
    int m_width = 0;

    /// ID of last search.
    uint32_t m_lastWaveId = 0;
};

struct EmptySearchPredicate
{
    bool isGoal(Node* node) const
    {
        return false;
    }
};

/// A predicate for finding path to a specidic target.
struct TargetSearchPredicate
{
    Point2 target;

    TargetSearchPredicate(SearchGrid& grid) : grid(grid) {}

    bool isGoal(Node* node) const
    {
        auto point = grid.nodeCoords(node);
        return target == point;
    };

    SearchGrid& grid;
};

/// A predicate for pathfinding task with multiple goals.
/// isGoal will return true only when wave visits all specified targets.
struct GroupSearchPredicate
{

    GroupSearchPredicate(SearchGrid& grid) : grid(grid)
    {
        width = grid.getWidth();
        mask.resize(width * width);
    }

    /// Checks if specified position is masked.
    bool isMasked(Node::ID id) const
    {
        return mask[id] != 0;
    }

    /// Clears search space.
    void clear()
    {
        for (const auto& pt : targets)
        {
            mask[pt.x + pt.y * width] = 0;
        }
        targets.clear();
        hits = 0;
    }

    /// Adds target to search space.
    void addTarget(const Point2& pt)
    {
        Node::ID id = pt.x + pt.y * width;
        if (!mask[id])
        {
            mask[id] = 1;
            targets.push_back(pt);
        }
    }
#define MULTIWAVE_FIRST
    void setMainTarget(const Point2& pt)
    {
#ifdef MULTIWAVE_FIRST
        centerId = grid.pointIndex(pt);
#else
        addTarget(pt);
#endif
    }

    bool isGoal(Node* node) const
    {
        auto id = grid.nodeIndex(node);
        if (id == centerId)
            centerId = -1;
        if (isMasked(id))
            hits++;
#ifdef MULTIWAVE_FIRST
        return hits > 0 && centerId == -1;
#else
        return hits == targets.size();
#endif
    }

    bool empty() const
    {
        return targets.empty();
    }

    SearchGrid& grid;
    int width = 0;
    std::vector<Point2> targets;
    std::vector<char> mask;
    mutable int centerId = -1;
    mutable int hits = 0;
};

#pragma endregion


#ifdef LOG_SVG
#include "draw.h"
#endif

#pragma region Parser
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

    int dropRequests(int tick) {
        assert(m_state == State::ParsedRequestsNumber);
        int numOrders = 0;
        char buffer[256] = {};
        m_in.getline(buffer, 255);
        //std::gets(buffer);
        numOrders = atoi(buffer);
        for (int i = 0; i < numOrders; i++)
            m_in.getline(buffer, 255);
        return 0;
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
#pragma endregion

class Dispatcher
{
public:
    Dispatcher(Map& map, const PClock::time_point& simStart)
        :m_search(map), m_groupPred(m_search), m_map(map)
    {
        m_simStart = simStart;

        m_simEndLimit = m_simStart + std::chrono::duration_cast<PClock::duration>(std::chrono::milliseconds(19500));
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
        m_maxPathCost = 0;
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
                        if (m_maxPathCost < node->cost)
                            m_maxPathCost++;
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
        //int maxCells = 0;
        //for (const auto& island : m_map.islands)
        //    maxCells += island.size();
        int avgTimeToDeliver = 1.5*m_maxPathCost;// 2 * sqrt(maxCells);
        int orderDelay = 60 * maxSteps / maxOrders;

        m_orderPrice = orderPrice;
        m_orders.reserve(maxOrders);

        int needRobots = avgTimeToDeliver / orderDelay + 1;
        // The situation when we have considerable delivery time for each robot.
        if (avgTimeToDeliver * 3 > orderPrice)
            needRobots *= 8;
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
        m_inactiveRobots = m_robots.size();
    }

    void registerNewOrders(int added)
    {
        for (size_t o = m_orders.size() - added; o < m_orders.size(); o++)
        {
            m_freeOrders.insert((int)o);
            Order& order = *m_orders[o];
            order.id = o;
            int siteIndex = getSiteIndex(order.start);
            m_sites[siteIndex].orders.insert((int)o);
            order.siteIndex = siteIndex;
        }
    }

    void processNewOrders()
    {
        TargetSearchPredicate pred(m_search);
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
                if (!robot.isIdle()/* && robot.state != Robot::State::MovingFinish*/)
                {
                    continue;
                }
                if (robot.state == Robot::State::MovingFinish)
                {
                    //m_groupPred.addTarget(robot.);
                }
                else
                {
                    m_groupPred.addTarget(robot.pos);
                }
                robotCandidates.push_back(r);
            }

            if (robotCandidates.empty())
            {
#ifdef LOG_STDIO
                std::cout << "No free candidates for task " << orderId << std::endl;
#endif
                break;
            }

            m_groupPred.setMainTarget(order->finish);

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
            {
#ifdef LOG_STDIO
                std::cerr << "Unexpected collapse of the wave for task " << orderId << std::endl;
#endif
            }
            else
            {
                m_search.tracePath(order->finish.x, order->finish.y, order->deliveryPath.points);
                order->deliveryPath.reverse();

                int nearestRobot = -1;
                int nearestDistance = m_search.getWidth() * m_search.getWidth();
                for (int r : robotCandidates)
                {
                    Robot& robot = m_robots[r];
                    const auto* node = m_search.getNode(robot.pos.x, robot.pos.y);
                    if (!m_search.isVisited(node))
                        continue;
                    int distance = node->cost + robot.timeToFree();
                    if (distance < nearestDistance || nearestRobot == -1)
                    {
                        nearestRobot = r;
                        nearestDistance = distance;
                    }
                }

                if (nearestRobot == -1)
                {
#ifdef LOG_STDIO
                    std::cerr << "Wave for task " << orderId << " have not reached any robot" << std::endl;
                    continue;
#endif
                }
#ifdef LOG_STDIO
                std::cout << "Assigning task " << orderId << " to robot " << nearestRobot << std::endl;
#endif
                auto& robot = m_robots[nearestRobot];
                robot.sites.push_back(order->siteIndex);
                m_search.tracePath(robot.pos.x, robot.pos.y, robot.approachPath.points);
#ifdef LOG_SVG
                drawer.drawPath(robot.approachPath.points);
#endif
                assignedOrders.push_back(orderId);
            }
        }
        for (auto orderId : assignedOrders)
            m_freeOrders.erase(orderId);
    }

    void moveRobots(int step, int tick)
    {
        for (int r = 0; r < m_robots.size(); r++)
        {
            auto& robot = m_robots[r];
            if (robot.order == nullptr && robot.sites.empty())
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
                    m_inactiveRobots--;
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
                        robot.order = order;
                        robot.pathLeft = order->deliveryDistance();
                        robot.commands[tick] = 'T';
                        robot.state = Robot::State::MovingFinish;
                    }
                    else
                    {
                        robot.commands[tick] = 'S';
                        robot.state = Robot::State::Idle;
                        m_inactiveRobots++;
                    }
                }
            }
            else if (robot.state == Robot::State::MovingFinish)
            {
                auto* order = robot.order;
                // Move across the path
                if (robot.pathPosition < order->deliveryPath.points.size() - 1)
                {
                    robot.pathPosition++;
                    robot.pathLeft--;
                    robot.stepTo(order->deliveryPath[robot.pathPosition], tick);
                }
                else
                {
                    // Final position.
                    robot.pathPosition = -1;
                    robot.commands[tick] = 'P';
                    assert(robot.pos == order->finish);
                    if (robot.sites.empty())
                    {
                        robot.state = Robot::State::Idle;
                        m_inactiveRobots++;
                    }
                    else
                    {

                    }
                    robot.order = nullptr;
                    closeOrder(step, tick, order);
                }
            }
        }
    }

    void closeOrder(int step, int tick, Order* order)
    {
        uint64_t latency = step*60 + tick - order->time;
        m_revenueLoss += latency;
        uint64_t revenue = 0;
        if (latency < m_orderPrice)
        {
            revenue = (m_orderPrice - latency - 1);
        }

        m_revenue += revenue;
        m_orders[order->id].reset();
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

    void runStep(int step, int tasksAdded, bool halt)
    {
        registerNewOrders(tasksAdded);

        if (!halt)
            processNewOrders();

        if (!halt || m_inactiveRobots > 0)
        {
            for (int tick = 0; tick < 60; tick++)
            {
                moveRobots(step, tick);
                if (m_inactiveRobots > 0)
                    processNewOrders();
            }
        }
    }

    // Process all calculation.
    void runSimulation(ProblemParser& parser, bool hardTimeLimit)
    {
        processIslands();
        calculateInitialPositions(parser.getMaxSteps(), parser.getMaxOrders(),
            parser.getOrderPrice(), parser.getRobotPrice());
        publishInitialPositions();

        int hitIterations = -1;
        constexpr int maxTimeMs = 10000;

        // A first step when we start considering timeout.
        int considerTimeoutStep = 2000;
        int hitRevenue = 0;
        int hitLoss = 0;
        // Time-conserving mode when we read requests in a most simple way.
        bool dropInput = false;
        // Time-conserving mode when we stop all the robots.
        bool haltRobots = false;

        for (int step = 0; step < parser.getMaxSteps(); step++)
        {
            // Calculate paths for each new request.
            int added = 0;
            if (!dropInput)
            {
                auto readStart = PClock::now();
                added = parser.parseStepRequests(m_orders, 60 * step);
                m_readTime += (PClock::now() - readStart);
            }
            else
            {
                parser.dropRequests(60*step);
            }
            

            int stepsLeft = parser.getMaxSteps() - step;
            // Process tasks and them to robots.
            runStep(step, added, haltRobots);
            publishRobots();

            auto now = PClock::now();
            int passedMs = MS(m_simStart, now).count();
            if (step > 1 && (passedMs > maxTimeMs || step > considerTimeoutStep))
            {
                if (hitIterations == -1)
                {
                    // Estimated time to publish all the data.
                    auto estimatePublish = stepsLeft * m_publishTime / step;
                    // Estimated time to read the rest of requests.
                    auto estimateRead = stepsLeft * m_readTime / step;
                    auto totalLeft = estimatePublish + estimateRead;
                    //std::cout << "estimatePublish=" << MS(estimatePublish).count()
                    //    << " estimateRead=" << MS(estimateRead).count() << std::endl;


                    if (now + totalLeft > m_simEndLimit)
                    {
                        auto overtime = MS(now + totalLeft - m_simEndLimit);
                        hitIterations = step;
                        hitRevenue = m_revenue;
                        hitLoss = m_revenueLoss;
                        if (hardTimeLimit)
                        {
                            dropInput = true;
                            haltRobots = true;
#if LOG_STDIO
                            std::cerr << "Enabling hard time limit at iteration " << hitIterations
                                <<" overtime=" << overtime.count() << std::endl;
#endif
                        }
                        else
                        {
#if LOG_STDIO
                            std::cerr << "Enabling soft time limit at iteration " << hitIterations
                                << " overtime=" << overtime.count() << std::endl;
#endif
                        }
                    }
                }
            }
        }

        auto timeFinish = PClock::now();
#ifdef LOG_STDIO
        std::cout << "Done in " << MS(m_simStart, timeFinish).count() << "ms." << std::endl;
        std::cout << "Revenue=" << m_revenue - m_parkCost << ". Loss=" << m_revenueLoss << std::endl;
        if (hitIterations != -1)
        {
            std::cout << "Managed to process only " << hitIterations
                << " steps out of " << parser.getMaxSteps()
                << " (" << (100 * hitIterations / parser.getMaxSteps()) << "%)" << std::endl;

            std::cout << "Limited revenue=" << hitRevenue - m_parkCost
                << " limited loss=" << hitLoss << std::endl;
        }
#endif
    }

    /// Publish robot commands for last simulation minute.
    void publishRobots()
    {
        auto start = PClock::now();
        for (auto& robot: m_robots)
        {
            puts(robot.commands);
            robot.clearCommands();
        }
        auto delta = (PClock::now() - start);
        m_publishTime += delta;
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

    // Number of inactive robots.
    int m_inactiveRobots = 0;
    // Number of robots which are finishing current task.
    int m_finishingRobots = 0;

    int m_maxPathCost = 0;
    // Starting time point for simulation.
    PClock::time_point m_simStart;
    // Time point when we must finish all calculations.
    PClock::time_point m_simEndLimit;

    // Time spent on publishing data.
    PClock::duration m_publishTime = {};
    // Time spent on making steps.
    PClock::duration m_stepTime = {};
    // Time spent on reading requests.
    PClock::duration m_readTime = {};
};

#ifndef LOCAL_TEST
int main(int argc, const char* argv[])
{
    ProblemParser parser(std::cin);
    Map map;

    auto simStart = PClock::now();

    if (!parser.parseObstacleMap(map)) {
        std::cerr << "Failed to parse obstacle map" << std::endl;
        return -1;
    }

    if (!parser.parseRequestsHeader()) {
        std::cerr << "Failed to parse requests" << std::endl;
        return -1;
    }

    Dispatcher dispatcher(map, simStart);
    dispatcher.runSimulation(parser, true);
    return 0;
}
#endif
