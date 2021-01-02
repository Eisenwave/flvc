#ifndef FLVC_SVO_HPP
#define FLVC_SVO_HPP

#include "voxelio/src/assert.hpp"
#include "voxelio/src/bits.hpp"
#include "voxelio/src/ileave.hpp"
#include "voxelio/src/stringify.hpp"
#include "voxelio/src/vec.hpp"

#include <array>
#include <bitset>
#include <memory>
#include <stack>
#include <vector>

namespace flvc {

// STATIC UTILITY ======================================================================================================

namespace detail {

/**
 * @brief A two's complement-oriented abs function.
 * This decrements negative values after computing the absolute.
 *
 * The motivation for this is that an SVO can hold one more coordinate in negative direciton than positive, so when
 * deciding whether the SVO needs to be grown for insertion, this is taken into account.
 *
 * Examples: abs_svo(3) = 3, abs_svo(-3) = 2, abs_svo(-1) = 0
 *
 * @param x the input
 * @return x if x is positive, else abs(x) - 1
 */
constexpr uint32_t abs_svo(int32_t x)
{
    // It is crucial that we first add 1, then negate.
    // This prevents signed integer underflow which is ub in C++17-.
    return static_cast<uint32_t>(x < 0 ? -(x + 1) : x);
}

/// Interleaves three single-bit numbers.
constexpr uint64_t ileave3_one(uint32_t x, uint32_t y, uint32_t z)
{
    return (x << 2) | (y << 1) | (z << 0);
}

/// Deinterleaves three single-bit numbers.
constexpr voxelio::Vec3u32 dileave3_one(uint64_t n)
{
    return voxelio::Vec3u64{(n >> 2) & 1, (n >> 1) & 1, (n >> 0) & 1}.cast<uint32_t>();
}

}  // namespace detail

// ENUMS ===============================================================================================================

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

// NODE CLASSES ========================================================================================================

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

// SVO =================================================================================================================

template <typename T, typename BT = void>
class SparseVoxelOctree {
private:
    using i32 = int32_t;
    using u32 = uint32_t;
    using u64 = uint64_t;

    using Vec3i32 = voxelio::Vec3i32;
    using Vec3u32 = voxelio::Vec3u32;

    template <typename Pointer>
    using uptr = std::unique_ptr<Pointer>;

    /**
     * Returns the unilateral capacity for a given depth.
     * Due to the fact that the SVO extends one coordinate further into negative space, this will be an inclusive
     * limit for negative coordinates and an exclusive limit for positive coordinates.
     *
     * In mathematical terms, we support the insertion of coordinates n, when:
     * -unilateralCapacity() <= n < unilateralCapacity()
     *
     * @param depth the depth
     * @return the unilateral capacity
     */
    static constexpr u32 unilateralCapacity(size_t depth)
    {
        // this effectively calculates pow(2, d)
        return 1 << depth;
    }

public:
    /// An exclusive upper bound for coordinates.
    static constexpr i32 COORDINATE_UPPER_LIMIT = 1 << 21;
    /// An inclusive lower bound for coordinates.
    static constexpr i32 COORDINATE_LOWER_LIMIT = -COORDINATE_UPPER_LIMIT;
    /// The amount of bits that one octree node index digit has.
    static constexpr size_t INDEX_DIGIT_BITS = 3;
    /// A mask for a single octree node index digit.
    static constexpr size_t INDEX_DIGIT_MASK = 0b111;
    /// The branching factor. 8 for an unsquashed SVO, 64 for one squash, etc.
    static constexpr size_t BRANCHING_FACTOR = 1 << INDEX_DIGIT_BITS;

    using self_type = SparseVoxelOctree;

    using value_type = T;
    using branch_value_type = BT;

    using node_type = SvoNode<branch_value_type>;
    using branch_type = SvoBranch<branch_value_type>;
    using leaf_type = SvoLeaf<T, branch_value_type>;

private:
    template <typename Consumer>
    static constexpr bool isNodeConsumer = std::is_invocable_r_v<void, Consumer, node_type *, SvoNodeType>;

private:
    uptr<branch_type> root = std::make_unique<branch_type>();
    size_t depth = 1;

public:
    SparseVoxelOctree() = default;
    SparseVoxelOctree(const SparseVoxelOctree &copyOf) : root{copyOf.root->clone()} {}
    SparseVoxelOctree(SparseVoxelOctree &&) = default;

    value_type *getIfExists(const Vec3i32 &pos);
    const value_type *getIfExists(const Vec3i32 &pos) const;

    value_type &at(const Vec3i32 &pos);
    const value_type &at(const Vec3i32 &pos) const;

    /**
     * Removes all nodes from the octree. This is not guaranteed to free all memory associated with the octree.
     */
    void clear();

    bool empty() const
    {
        return root->empty();
    }

    /**
     * Returns true exactly when the octree contains a voxel at the given position.
     * @param pos the position
     * @return true exactly when the octree contains a voxel at the given position
     */
    bool contains(const Vec3i32 &pos) const;

    /**
     * Returns the absolute depth of the SVO.
     * The absolute depth is the number of SVO layers of nodes.
     * Individual voxels do not count as nodes.
     *
     * @return the signed depth
     */
    size_t depthAbsolute() const
    {
        return depth + 1;
    }

    /**
     * Returns the signed depth of the SVO.
     * The signed depth is the absolute depth - 1, because it extends into both positive and negative direction.
     * Thus, one SVO layer is effectively skipped.
     *
     * For example, a 4x4x4 SVO has an absolute depth of 2 and a signed depth of 1.
     * It requires two SVO layers until individual voxels are reached.
     * @return the signed depth
     */
    size_t depthSigned() const
    {
        return depth;
    }

    /**
     * @brief Applies a consumer function to all nodes.
     * The order of traversal is from top to bottom.
     * It is guaranteed that a parent node will be consumed before its child nodes.
     *
     * This function makes many optimizations that are not available to the iterator of depth_first_range, so it should
     * be preferred.
     *
     * @tparam consumer the consumer functor void(node_type*, SvoNodeType)
     */
    template <typename Consumer,
              std::enable_if_t<std::is_invocable_r_v<void, Consumer, node_type *, SvoNodeType>, int> = 0>
    void forEachNodeTopDown(const Consumer &consumer)
    {
        forEachNodeTopDown_impl(consumer);
    }

    /**
     * @brief Applies a consumer function to all nodes.
     * The order of traversal is from bottom to top.
     * It is guaranteed that all child nodes will be consumed before their parent node.
     *
     * This function makes many optimizations that are not available to the iterator of depth_first_range, so it should
     * be preferred.
     *
     * @tparam consumer the consumer functor void(node_type*, SvoNodeType)
     */
    template <typename Consumer,
              std::enable_if_t<std::is_invocable_r_v<void, Consumer, node_type *, SvoNodeType>, int> = 0>
    void forEachNodeBottomUp(const Consumer &consumer)
    {
        forEachNodeBottomUp_impl(consumer);
    }

    /**
     * @brief Converts a position to an index inside the SVO.
     * This index can be used for faster random access in other methods.
     * The index becomes invalid when the SVO is resized.
     * @param pos the position
     * @return the index of the position
     */
    u64 indexOf(Vec3i32 pos) const
    {
        VXIO_DEBUG_ASSERT_EQ(boundsTest(pos), 0);
        Vec3u32 uPos = Vec3i32{pos - minIncl()}.cast<u32>();
        u64 result = voxelio::ileave3(uPos[0], uPos[1], uPos[2]);
        return result;
    }

    /**
     * @brief Converts an index obtained using indexOf(Vec3i32) back to the input position.
     * @param pos the position
     * @return the index of the position
     */
    Vec3i32 indexToPos(u64 index) const
    {
        Vec3u32 uPos;
        voxelio::dileave3(index, uPos.data());
        return uPos.cast<i32>() + minIncl();
    }

    /**
     * @brief Inserts a value into the SVO.
     * If necessary, the SVO will be resized to fit the position.
     * @param pos the position at which to insert
     * @param value the value to insert
     */
    void insert(Vec3i32 pos, value_type value)
    {
        reserve(pos);
        insert_unsafe(indexOf(pos), std::move(value));
    }

    /// Returns the minimum coordinates that the SVO can currently contain.
    Vec3i32 minIncl() const
    {
        return Vec3i32::filledWith(-unilateralCapacity());
    }

    /// Returns the maximum coordinates that the SVO can currently contain.
    Vec3i32 maxIncl() const
    {
        return Vec3i32::filledWith(unilateralCapacity() - 1);
    }

    /// Returns coordinates that are lower in every dimension than any point the SVO can currently contain.
    /// Inserting a voxel into the SVO at this location will resize the SVO exactly once.
    Vec3i32 minExcl() const
    {
        return Vec3i32::filledWith(-unilateralCapacity() - 1);
    }

    /// Returns coordinates that are greater in every dimension than any point the SVO can currently contain.
    /// Inserting a voxel into the SVO at this location will resize the SVO exactly once.
    Vec3i32 maxExcl() const
    {
        return Vec3i32::filledWith(unilateralCapacity());
    }

    /**
     * @brief Ensures that the SVO has enough space to contain a voxel at the position which is to be inserted.
     * @param pos the position
     */
    void reserve(const Vec3i32 &pos)
    {
        if (u32 lim = boundsTest(pos); lim != 0) {
            reserve(lim);
        }
    }

    /**
     * @brief Ensures that the SVO has enough space to contain a coordinate which is to be inserted.
     * @param max the coordinate (inclusive)
     */
    void reserve(u32 max);

    branch_type &rootNode()
    {
        return *root;
    }

    const branch_type &rootNode() const
    {
        return *root;
    }

    SparseVoxelOctree &operator=(const SparseVoxelOctree &copyOf)
    {
        root.reset(copyOf.root->clone());
        depth = copyOf.depth;
        return *this;
    }

    SparseVoxelOctree &operator=(SparseVoxelOctree &&) = default;

    value_type &operator[](const Vec3i32 &pos)
    {
        reserve(pos);
        return findValueOrCreate_unsafe(indexOf(pos));
    }

private:
    /**
     * @brief Returns the unilateral capacity of the SVO.
     * Due to the fact that the SVO extends one coordinate further into negative space, this will be an inclusive
     * limit for negative coordinates and an exclusive limit for positive coordinates.
     *
     * In mathematical terms, we support the insertion of coordinates n, when:
     * -unilateralCapacity() <= n < unilateralCapacity()
     *
     * @return the unilateral capacity
     */
    u32 unilateralCapacity() const
    {
        return unilateralCapacity(depth);
    }

    /**
     * @brief Tests whether the input vector lies within this octree. The result is an unsigned integer which indicates
     * how much the octree has to be enlarged to fit the vector. The dimensions of the octree in all directions need to
     * be > this integer.
     * @param v the input position
     * @return 0 if the test passes or a maximum-like coordinate if the test fails
     */
    uint32_t boundsTest(const Vec3i32 &v) const
    {
        u32 max = detail::abs_svo(v[0]) | detail::abs_svo(v[1]) | detail::abs_svo(v[2]);
        VXIO_DEBUG_ASSERT_LT(max, COORDINATE_UPPER_LIMIT);
        return max >= unilateralCapacity() ? max : 0;
    }

    value_type &findValueOrCreate_unsafe(u64 octreeNodeIndex);
    node_type *findNode_unsafe_impl(u64 octreeNodeIndex) const;
    value_type *findIfExists_unsafe_impl(u64 octreeNodeIndex) const;

    node_type *findNode_unsafe(u64 octreeNodeIndex)
    {
        return findNode_unsafe_impl(octreeNodeIndex);
    }

    const node_type *findNode_unsafe(u64 octreeNodeIndex) const
    {
        return findNode_unsafe_impl(octreeNodeIndex);
    }

    value_type *findValue_unsafe(u64 octreeNodeIndex)
    {
        return findIfExists_unsafe_impl(octreeNodeIndex);
    }

    const value_type *findValue_unsafe(u64 octreeNodeIndex) const
    {
        return findIfExists_unsafe_impl(octreeNodeIndex);
    }

    /**
     * @brief Grows the SVO unilaterally once.
     * This effectively doubles each SVO dimension and increases the depth by one.
     */
    void growOnce();

    /**
     * @brief Inserts a color at the given index. This assumes that space has already been allocated.
     * @param octreeNodeIndex the index at which to insert
     * @param color the color to be inserted
     */
    void insert_unsafe(u64 octreeNodeIndex, value_type color)
    {
        findValueOrCreate_unsafe(octreeNodeIndex) = std::move(color);
    }

    template <typename Consumer>
    void forEachNodeTopDown_impl(const Consumer &consumer);

    template <typename Consumer>
    void forEachNodeBottomUp_impl(const Consumer &consumer);
};

// VOXEL INSERTION/ACCESS/SEARCH =======================================================================================

template <typename T, typename BT>
T &SparseVoxelOctree<T, BT>::findValueOrCreate_unsafe(u64 octreeNodeIndex)
{
    node_type *node = root.get();
    for (size_t s = depth * INDEX_DIGIT_BITS; s != size_t(-INDEX_DIGIT_BITS); s -= INDEX_DIGIT_BITS) {
        u32 octDigit = (octreeNodeIndex >> s) & INDEX_DIGIT_MASK;
        if (s != 0) {
            auto *branch = voxelio::downcast<branch_type *>(node);
            if (branch->has(octDigit)) {
                node = branch->child(octDigit);
            }
            else {
                auto *child = s == INDEX_DIGIT_BITS ? static_cast<node_type *>(new leaf_type)
                                                    : static_cast<node_type *>(new branch_type);
                node = child;
                branch->insert(octDigit, uptr<node_type>{child});
            }
        }
        else {
            auto *leaf = voxelio::downcast<leaf_type *>(node);
            return (*leaf)[octDigit];
        }
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T, typename BranchT>
typename SparseVoxelOctree<T, BranchT>::node_type *SparseVoxelOctree<T, BranchT>::findNode_unsafe_impl(
    u64 octreeNodeIndex) const
{
    node_type *node = root.get();
    for (size_t s = depth * INDEX_DIGIT_BITS;; s -= INDEX_DIGIT_BITS) {
        if (s == 0) {
            return node;
        }

        u32 octDigit = (octreeNodeIndex >> s) & INDEX_DIGIT_MASK;
        auto *branch = voxelio::downcast<branch_type *>(node);
        if (branch->has(octDigit)) {
            node = branch->child(octDigit);
        }
        else {
            return nullptr;
        }
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T, typename BranchT>
T *SparseVoxelOctree<T, BranchT>::findIfExists_unsafe_impl(u64 octreeNodeIndex) const
{
    node_type *node = root.get();
    for (size_t s = depth * INDEX_DIGIT_BITS; s != size_t(-INDEX_DIGIT_BITS); s -= INDEX_DIGIT_BITS) {
        u32 octDigit = (octreeNodeIndex >> s) & INDEX_DIGIT_MASK;
        if (s != 0) {
            auto *branch = voxelio::downcast<branch_type *>(node);
            if (not branch->has(octDigit)) {
                return nullptr;
            }
            else {
                node = branch->child(octDigit);
            }
        }
        else {
            auto *leaf = voxelio::downcast<leaf_type *>(node);
            return leaf->get_if(octDigit);
        }
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T, typename BranchT>
T *SparseVoxelOctree<T, BranchT>::getIfExists(const Vec3i32 &pos)
{
    if (u32 lim = boundsTest(pos); lim != 0) {
        return nullptr;
    }
    return findValue_unsafe(indexOf(pos));
}

template <typename T, typename BranchT>
const T *SparseVoxelOctree<T, BranchT>::getIfExists(const Vec3i32 &pos) const
{
    if (u32 lim = boundsTest(pos); lim != 0) {
        return nullptr;
    }
    return findValue_unsafe(indexOf(pos));
}

template <typename T, typename BranchT>
T &SparseVoxelOctree<T, BranchT>::at(const Vec3i32 &pos)
{
    auto *result = getIfExists(pos);
    VXIO_ASSERT_NOTNULL(result);
    return *result;
}

template <typename T, typename BranchT>
const T &SparseVoxelOctree<T, BranchT>::at(const Vec3i32 &pos) const
{
    auto *result = getIfExists(pos);
    VXIO_ASSERT_NOTNULL(result);
    return *result;
}

template <typename T, typename BranchT>
bool SparseVoxelOctree<T, BranchT>::contains(const Vec3i32 &pos) const
{
    if (boundsTest(pos) != 0) {
        return false;
    }
    return findValue_unsafe(indexOf(pos)) != nullptr;
}

// STRUCTURE MANAGEMENT ================================================================================================

template <typename T, typename BranchT>
void SparseVoxelOctree<T, BranchT>::clear()
{
    root->clear();
}

template <typename T, typename BranchT>
void SparseVoxelOctree<T, BranchT>::reserve(u32 lim)
{
    while (lim >= unilateralCapacity()) {
        growOnce();
    }
}

template <typename T, typename BranchT>
void SparseVoxelOctree<T, BranchT>::growOnce()
{
    for (size_t i = 0; i < BRANCHING_FACTOR; ++i) {
        if (not root->has(i)) {
            continue;
        }
        auto parent = std::make_unique<branch_type>();
        parent->insert(~i & INDEX_DIGIT_MASK, root->extract(i));
        root->insert(i, std::move(parent));
    }

    depth += 1;
}

// FUNCTIONAL ==========================================================================================================

template <typename T, typename BT>
template <typename Consumer>
void SparseVoxelOctree<T, BT>::forEachNodeTopDown_impl(const Consumer &consumer)
{
    static_assert(isNodeConsumer<Consumer>);

    struct Pos {
        branch_type *branch;
        size_t index = 0;
    };

    std::stack<Pos> stack;
    stack.push({root.get(), 0});
    consumer(root.get(), SvoNodeType::BRANCH);

    while (not stack.empty()) {
        Pos &pos = stack.top();

        if (stack.size() == depth) {  // one layer above leafs
            for (; pos.index < BRANCHING_FACTOR; ++pos.index) {
                auto *child = voxelio::downcast<leaf_type *>(pos.branch->child(pos.index));
                if (child != nullptr) {
                    consumer(child, SvoNodeType::LEAF);
                }
            }
        }
        else {
            for (; pos.index < BRANCHING_FACTOR; ++pos.index) {
                auto *child = voxelio::downcast<branch_type *>(pos.branch->child(pos.index));
                if (child != nullptr) {
                    stack.push({child, 0});
                    consumer(child, SvoNodeType::BRANCH);
                    goto end_of_while;
                }
            }
        }

        do {
            stack.pop();
            if (stack.empty()) {
                return;
            }
            Pos &pos = stack.top();
            if (++pos.index < BRANCHING_FACTOR) {
                break;
            }
        } while (true);

    end_of_while:;
    }
}

template <typename T, typename BT>
template <typename Consumer>
void SparseVoxelOctree<T, BT>::forEachNodeBottomUp_impl(const Consumer &consumer)
{
    static_assert(isNodeConsumer<Consumer>);

    struct Pos {
        node_type *node;
        size_t index = 0;
    };

    std::stack<Pos> stack;
    stack.push({root.get(), 0});

    while (not stack.empty()) {
        Pos &topPos = stack.top();
        node_type *top = topPos.node;
        SvoNodeType type = stack.size() > depth ? SvoNodeType::LEAF : SvoNodeType::BRANCH;

        if (type == SvoNodeType::BRANCH) {
            for (; topPos.index < BRANCHING_FACTOR; ++topPos.index) {
                node_type *child = voxelio::downcast<branch_type *>(topPos.node)->child(topPos.index);
                if (child != nullptr) {
                    stack.push({child, 0});
                    goto end_of_while;
                }
            }
        }

        do {
            consumer(top, type);

            stack.pop();
            if (stack.empty()) {
                return;
            }
            Pos &pos = stack.top();
            if (++pos.index < BRANCHING_FACTOR) {
                break;
            }
            top = pos.node;
            type = SvoNodeType::BRANCH;
        } while (true);

    end_of_while:;
    }
}

// FUNCTIONAL EXTERNAL FUNCTIONS =======================================================================================

namespace detail {

template <bool EXPAND,
          typename Reduction,
          typename T,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, BT>::node_type *[],
                                                 size_t,
                                                 typename SparseVoxelOctree<T, BT>::branch_type &>,
                           int> = 0>
void reduceOrExpandNodes(SparseVoxelOctree<T, BT> &svo, const Reduction &reduction)
{
    using svo_type = SparseVoxelOctree<T, BT>;
    using node_type = typename svo_type::node_type;
    using branch_type = typename svo_type::branch_type;

    node_type *buffer[svo_type::BRANCHING_FACTOR];

    const auto consumer = [&buffer, &reduction](node_type *node, SvoNodeType type) -> void {
        if (type == SvoNodeType::LEAF) {
            return;
        }
        branch_type *oldBranch = voxelio::downcast<branch_type *>(node);
        size_t oldCount = 0;
        for (size_t i = 0; i < svo_type::BRANCHING_FACTOR; ++i) {
            if (oldBranch->has(i)) {
                buffer[oldCount++] = oldBranch->child(i);
            }
        }

        reduction(buffer, oldCount, *oldBranch);
    };
    if constexpr (EXPAND) {
        svo.forEachNodeTopDown(consumer);
    }
    else {
        svo.forEachNodeBottomUp(consumer);
    }
}

}  // namespace detail

template <typename Reduction,
          typename T,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, BT>::node_type *[],
                                                 size_t,
                                                 typename SparseVoxelOctree<T, BT>::branch_type &>,
                           int> = 0>
void reduceNodes(SparseVoxelOctree<T, BT> &svo, const Reduction &reduction)
{
    return detail::reduceOrExpandNodes<false>(svo, reduction);
}

template <typename Reduction,
          typename T,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, BT>::node_type *[],
                                                 size_t,
                                                 typename SparseVoxelOctree<T, BT>::branch_type &>,
                           int> = 0>
void expandNodes(SparseVoxelOctree<T, BT> &svo, const Reduction &reduction)
{
    return detail::reduceOrExpandNodes<true>(svo, reduction);
}

}  // namespace flvc

#endif  // FLVC_SVO_HPP
