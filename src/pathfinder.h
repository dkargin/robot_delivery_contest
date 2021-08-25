#pragma once

#include <vector>
#include <memory>
#include <cassert>

#include "state.h"


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

    SearchGrid(const Map& map) : m_map(map), m_priorityHeap(map.dimension * map.dimension)
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
        for (auto& node: m_grid)
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
            visitNodeId(fromIndex+1, from);
        if (!(occ & Map::Up))
            visitNodeId(fromIndex-m_width, from);
        if (!(occ & Map::Left))
            visitNodeId(fromIndex-1, from);
        if (!(occ & Map::Down))
            visitNodeId(fromIndex+m_width, from);
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
        for (const auto& pt: targets)
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
