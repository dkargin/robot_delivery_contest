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

/// Estimated size of a single from = ~16 bytes.
/// Priority queue will need another 4 byte per from (very worst case).
/// Path ID or visitTime will take 4 bytes.
/// Max size of a map Nmax = 2000
/// Grid size = sizeof(Node) + sizeof(NodeID) + 4 = 24 * 2000 * 2000 ~ 96Mb.
/// 
