#pragma once


#include <vector>
#include <memory>
#include "state.h"
#include "priority_heap.h"

/// A from in a grid search.
/// N < 2096 -> 11 bits. So we can limit cost to a N*N -> 22 bits
/// int32_t is enough to fit longest path. 
/// The same for NodeId.
struct Node {
    using ID = int32_t;
    static constexpr ID InvalidID = -1;
    ID parent = InvalidID;
    /// Position in priority queue/binary heap.
    ID heapIndex = InvalidID;
    /// Search ID.
    uint32_t waveId = 0;
    using CostType = uint32_t;
    /// Accumulated path cost.
    CostType cost = 0;

    void reset()
    {
        cost = 0;
        parent = InvalidID;
    }
};

/// Wraps access to specific from type
template<> struct ContainerTraits<Node>
{
    using CostType = Node::CostType;

    static CostType getCost(const Node* a)
    {
        return a->cost /* + a->estimateCost*/;
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
 * 3. Eliminated most of allocations in a search core. It is always
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
        if (m_lastWaveId >= MaxWaveId)
        {
            clearGrid();
            m_lastWaveId = 1;
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

    // Clears all the contents of the grid.
    void clearGrid()
    {
        assert(false);
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

    void addAdjacentNodes(Node* from)
    {
        auto index = nodeIndex(from);
        auto visitNode = [this, index, from](int x, int y)
            {
                constexpr int moveCost = 1;
                int newCost = moveCost + from->cost;
                if (isVisited(x, y))
                    return;
                if (isOccupied(x, y))
                    return;
                Node* node = &m_grid[x + y * m_width];
                addNode(x, y, newCost, index);
            };

        auto pt = nodeCoords(from);
        if (pt.x > 0)
            visitNode(pt.x - 1, pt.y);
        if (pt.y > 0)
            visitNode(pt.x, pt.y - 1);
        if (pt.x < m_width - 1)
            visitNode(pt.x + 1, pt.y);
        if (pt.y < m_width - 1)
            visitNode(pt.x, pt.y + 1);
    }

    /// Trace back path from specified point.
    /// It returns reversed path.
    void tracePath(Coord x, Coord y, std::vector<Point2>& path) const
    {
        const Node* current = getNode(x, y);
        int iteration = 0;
        std::vector<const Node*> trace;
        while (current)
        {
            auto pt = nodeCoords(current);
            path.push_back(pt);
            trace.push_back(current);
            const Node* parent = nullptr;
            if (current->parent == Node::InvalidID)
                break;
            parent = &m_grid[current->parent];
            assert(parent != current);
            iteration++;
            assert(iteration < m_width* m_width);
            current = parent;
        };
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

/// Estimated size of a single from = ~16 bytes.
/// Priority queue will need another 4 byte per from (very worst case).
/// Path ID or visitTime will take 4 bytes.
/// Max size of a map Nmax = 2000
/// Grid size = sizeof(Node) + sizeof(NodeID) + 4 = 24 * 2000 * 2000 ~ 96Mb.
/// 
