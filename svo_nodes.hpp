#ifndef SVO_NODES_HPP
#define SVO_NODES_HPP

#include "voxelio/src/assert.hpp"
#include "voxelio/src/bits.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <memory>

namespace flvc {

/// The type of a SparseVoxelOctree node.
enum class SvoNodeType {
    /// A branch. This type of node has other nodes as children.
    BRANCH,
    /// A leaf. This type of node has voxels as children.
    LEAF
};

constexpr const char *nameOf(SvoNodeType type)
{
    return type == SvoNodeType::BRANCH ? "BRANCH" : "LEAF";
}

namespace detail {

template <typename T>
struct Voidable {
    T value;
};

template <>
struct Voidable<void> {
};

}  // namespace detail

template <typename BT = void>
class SvoNode {
private:
    static constexpr size_t N = 8;

    detail::Voidable<BT> val;

protected:
    std::bitset<N> mask_{};

    virtual void doClear() = 0;

public:
    SvoNode() = default;
    virtual ~SvoNode() = default;

    SvoNode(SvoNode &&) = default;
    SvoNode(const SvoNode &) = default;

    SvoNode &operator=(SvoNode &&) = default;
    SvoNode &operator=(const SvoNode &) = default;

    const std::bitset<N> &mask() const
    {
        return mask_;
    }

    /**
     * Returns the number of children for branches or the number of colors for leafs.
     * This is not a recursive test.
     * @return the number of children
     */
    size_t count() const
    {
        return mask_.count();
    }

    void clear()
    {
        doClear();
        mask_.reset();
    }

    /**
     * Returns whether this node has no children.
     * This is not a recursive test, meaning that true will be returned, even if all children are empty.
     * @return true if this node has no children
     */
    bool empty() const
    {
        return mask_.none();
    }

    /**
     * Returns whether all children of this node exist.
     * This is not a recursive test.
     * @return true if this node has all possible children
     */
    bool full() const
    {
        return mask_.all();
    }

    /**
     * Returns true if the node has a child at the given index.
     * @param index the index
     * @return true if the node has a child at index
     */
    bool has(size_t index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        return mask_.test(index);
    }

    /**
     * @brief Returns the index of the first child which exists or N if the node is empty.
     * @return the index of the first child
     */
    size_t firstIndex() const
    {
        return voxelio::countTrailingZeros(static_cast<uint8_t>(mask_.to_ullong()));
    }

    template <typename V = BT, std::enable_if_t<not std::is_void_v<V>, int> = 0>
    auto &value()
    {
        return val.value;
    }

    template <typename V = BT, std::enable_if_t<not std::is_void_v<V>, int> = 0>
    const auto &value() const
    {
        return val.value;
    }

    /**
     * Clones this node virtually. The result is an unmanaged pointer on the heap.
     * @return a pointer to the cloned object
     */
    virtual SvoNode *clone() const = 0;

    virtual SvoNodeType type() const = 0;
};

template <typename BT>
std::array<std::unique_ptr<SvoNode<BT>>, 8> deepCopy(const std::array<std::unique_ptr<SvoNode<BT>>, 8> &copyOf)
{
    std::array<std::unique_ptr<SvoNode<BT>>, 8> result;
    for (size_t i = 0; i < 8; ++i) {
        result[i] = std::unique_ptr<SvoNode<BT>>(copyOf[i]->clone());
    }
    return result;
}

template <typename BT = void>
class SvoBranch : public SvoNode<BT> {
private:
    static constexpr size_t N = 8;

public:
    using self_type = SvoBranch<BT>;
    using child_type = SvoNode<BT>;

private:
    template <typename T>
    using uptr = std::unique_ptr<T>;

    std::array<uptr<child_type>, N> children{};

protected:
    void doClear() final;

public:
    SvoBranch() = default;
    SvoBranch(const SvoBranch &copyOf) : SvoNode<BT>{copyOf}, children{deepCopy(copyOf.children)} {}
    SvoBranch(SvoBranch &&moveOf) = default;
    ~SvoBranch() final = default;

    SvoBranch *clone() const final
    {
        return new SvoBranch{*this};
    }

    SvoNodeType type() const final
    {
        return SvoNodeType::BRANCH;
    }

    child_type *child(size_t index)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        return children[index].get();
    }

    const child_type *child(size_t index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        return children[index].get();
    }

    uptr<child_type> extract(size_t index)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        VXIO_DEBUG_ASSERT(this->has(index));
        this->mask_.reset(index);
        return std::move(children[index]);
    }

    void insert(size_t index, uptr<child_type> node)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        children[index] = std::move(node);
        this->mask_.set(index);
    }

    SvoBranch &operator=(const SvoBranch &copyOf)
    {
        children = deepCopy(copyOf.children);
        this->mask_ = copyOf.mask_;
        return *this;
    }

    SvoBranch &operator=(SvoBranch &&moveOf) = default;
};

template <typename T, typename BT = void>
class SvoLeaf : public SvoNode<BT> {
private:
    static constexpr size_t N = 8;

    std::array<T, N> data{};

protected:
    void doClear() final;

public:
    SvoLeaf() = default;
    ~SvoLeaf() final = default;

    SvoLeaf(SvoLeaf &&) = default;
    SvoLeaf(const SvoLeaf &) = default;

    SvoLeaf &operator=(SvoLeaf &&) = default;
    SvoLeaf &operator=(const SvoLeaf &) = default;

    SvoLeaf *clone() const final
    {
        return new SvoLeaf{*this};
    }

    SvoNodeType type() const final
    {
        return SvoNodeType::LEAF;
    }

    // custom methods
    const T &at(size_t index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        VXIO_DEBUG_ASSERT(this->has(index));
        return data[index];
    }

    T &at(size_t index)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        VXIO_DEBUG_ASSERT(this->has(index));
        return data[index];
    }

    const T *get_if(size_t index) const
    {
        return this->has(index) ? &data[index] : nullptr;
    }

    T *get_if(size_t index)
    {
        return this->has(index) ? &data[index] : nullptr;
    }

    T &operator[](size_t index)
    {
        this->mask_.set(index);
        return data[index];
    }
};

template <typename BT>
void SvoBranch<BT>::doClear()
{
    for (auto &child : children) {
        child.release();
    }
}

template <typename T, typename BT>
void SvoLeaf<T, BT>::doClear()
{
    // TODO destroy elements
}

}  // namespace flvc

#endif  // SVO_NODES_HPP
