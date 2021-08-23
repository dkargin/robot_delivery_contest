#pragma once

#include <vector>

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
    virtual size_t getSize() const
    {
        return size;
    }

    virtual size_t getMaxSize() const
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
