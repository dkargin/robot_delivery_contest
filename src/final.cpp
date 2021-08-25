#include <cstdint>
#include <vector>
#include <set>
#include <list>
#include <chrono>
#include <memory>
#include <cassert>

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

    const Point2& operator[](int index) const
    {
        return points[index];
    }

    int distance() const
    {
        return (int)points.size();
    }
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
    Path deliveryPath;

    /// Path from current position of a robot to pickup point.
    Path approachPath;

    /// Check if order is assigned to any robot.
    bool isAssigned() const
    {
        return robotId >= 0;
    }

    int deliveryDistance() const
    {
        return (int)deliveryPath.points.size();
    }

    int approachDistance() const
    {
        return (int)approachPath.points.size();
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
        switch (cmd)
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
    std::list<int> orders;
    /// Position on current path.
    int pathPosition = -1;

    Robot()
    {
        commands.resize(60, Command::Idle);
    }

    /// Clears command buffer.
    void beginStep()
    {
        for (int i = 0; i < 60; i++)
            commands[i] = Command::Idle;
    }

    /// Moves robot to a specified point.
    void stepTo(const Point2& newPos, int step)
    {
        Command cmd = Command::Idle;
        if (newPos.x == pos.x && newPos.y == pos.y + 1)
        {
            cmd = Command::Down;
        }
        else if (newPos.x == pos.x && newPos.y == pos.y - 1)
        {
            cmd = Command::Up;
        }
        else if (newPos.x == pos.x + 1 && newPos.y == pos.y)
        {
            cmd = Command::Left;
        }
        else if (newPos.x == pos.x - 1 && newPos.y == pos.y)
        {
            cmd = Command::Right;
        }
        else
        {
            // Invalid step.
            assert(false);
        }
        pos = newPos;
        commands[step] = cmd;
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
    std::vector<std::vector<Point2>> ptIslands;
};

/// END_STATE_H

/// BEGIN_PATHFINDER_H

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
        peak = 0;
        nodesVisited = 0;
        maxSize = _size;
        array.reserve(_size);
        size = 0;
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
        return size;
    }

    size_t getMaxSize() const
    {
        return maxSize;
    }

    // @brief Add element to binary heap
    bool push(NodeType* node)
    {
        assert(node != NULL);

        if (this->size == maxSize)
        {
            return false;
        }

        size_t v = size;
        size_t u = v;

        int id = Traits::getID(node);

        //check if node is already in container
        if (id >= 0 && id < size && array[id] == node)
        {
            v = id;
            u = v;
            array[v] = node;
        }
        else
        {
            Traits::setID(node, v);
            array.push_back(node);

            nodesVisited++;
            if (array.size() > peak)
                peak = array.size();
        }

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
        size = array.size();
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
        size = array.size();

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

    NodeType* firstNode()
    {
        if (!array.empty())
            return array.front();
        return NULL;
    }

    NodeType* lastNode()
    {
        if (!array.empty())
            return array.back();
        return NULL;
    }


    /// remove Node from binary heap
    void removeNode(NodeType* node)
    {
        assert(node != NULL);

        long i = Traits::getID(node);
        bool flag = false;// if we  have not found this node
        // We cache array index for node
        if (array[i] == node)
        {
            flag = true;
        }
        // Or we've failed to cache it, so we search this node manually
        else
        {
            for (i = 0; i < size; i++)
                if (node == array[i])
                {
                    flag = true;
                    break;
                }
        }
        if (!flag)
            return;
        // Swapping nodes
        unsigned int chLeft = i;
        unsigned int chRight = i;
        unsigned int u = i;
        unsigned int v = i;
        size--;
        array[i] = array[size];
        Traits::setID(array[i], i);
        array[size] = NULL;

        while (true)
        {
            u = v;
            chLeft = u * 2 + 1;
            chRight = u * 2 + 2;;
            if (2 * u + 2 < size)
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
    }

    /// Remove all contents
    void clear()
    {
        nodesVisited = 0;
        peak = 0;
        size = 0;

        array.clear();
    }

    size_t getPeakSize() const
    {
        return peak;
    }

    size_t getSize()
    {
        return array.size();
    }

public:
    // Current size of a heap.
    size_t size;
    // Max size of a heap.
    size_t maxSize;
    size_t peak;
    // Number of nodes visited.
    int nodesVisited;
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
    /// Heuristic estimate of a cost.
    CostType estimate = 0;

    void reset()
    {
        cost = 0;
        estimate = 0;
        parent = InvalidID;
    }
};

/// Wraps access to specific from type
template<> struct ContainerTraits<Node>
{
    using CostType = Node::CostType;

    static CostType getCost(const Node* a)
    {
        return a->cost + a->estimate;
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

    bool isOccupied(Coord x, Coord y) const
    {
        return m_map.isOccupied(x, y);
    }

    bool isVisited(Coord x, Coord y) const
    {
        return m_grid[x + y * m_width].waveId == m_lastWaveId;
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

            addAdjacentNodes(top);
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

    bool isGoal(Node* node) const
    {
        auto id = grid.nodeIndex(node);
        if (isMasked(id))
            hits++;
        return hits == targets.size();
    }

    bool empty() const
    {
        return targets.empty();
    }

    SearchGrid& grid;
    int width = 0;
    std::vector<Point2> targets;
    std::vector<char> mask;
    mutable int hits = 0;
};

/// END_PATHFINDER_H

/// DISPATCHER_H

#include <iostream>

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
            for (int r : robotCandidates)
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
        for (const auto& robot : m_robots)
        {
            std::string out;
            for (auto cmd : robot.commands)
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

/// END_DISPATCHER_H

/// MAIN_CPP

int main(int argc, const char* argv[])
{
    ProblemParser parser(std::cin);
    Map map;

    if (!parser.parseObstacleMap(map)) {
        std::cerr << "Failed to parse obstacle map" << std::endl;
        return -1;
    }

    if (!parser.parseRequestsHeader()) {
        std::cerr << "Failed to parse requests" << std::endl;
        return -1;
    }

    Dispatcher dispatcher(map);
    dispatcher.processIslands();
    dispatcher.calculateInitialPositions(parser.getMaxSteps(), parser.getMaxOrders(),
        parser.getOrderPrice(), parser.getRobotPrice());
    dispatcher.publishInitialPositions();

    for (int step = 0; step < parser.getMaxSteps(); step++)
    {
        // Calculate paths for each new request.
        int added = parser.parseStepRequests(dispatcher.m_orders, 60 * step);

        // Process tasks and them to robots.
        dispatcher.processNewOrders(added);

        dispatcher.prepareTurn();
        for (int tick = 0; tick < 60; tick++)
            dispatcher.moveRobots(tick);

        dispatcher.publishRobots();
    }
    return 0;
}

/// END_MAIN_CPP
