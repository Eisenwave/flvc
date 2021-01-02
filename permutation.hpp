#ifndef FLVC_PERMUTATION_HPP
#define FLVC_PERMUTATION_HPP

#include "voxelio/src/assert.hpp"

#include <algorithm>
#include <vector>

namespace flvc {

/**
 * @brief A class representing a mathematical permutation.
 *
 * The internal permutation table and the tables from which the permutation is constructed are in INVERSE notation.
 * This means that each index in the permutation represents the input index from which the output is copied.
 * Example:
 *     (a, b, c) x (1, 2, 0) = (b, c, a)
 * This is contrary to the usual mathematical notation, where each index represents the index in the output onto which
 * the element is mapped. Converting between these notations can be done using inverse()
 *
 * In addition to storing the permutation in table-form itself, some information is computed upon construction
 * such as the start-indices of the cycles and whether this permutation is an identity permutation.
 *
 * Constructing a permutation from a table that does not actually represent a permutation is undefined behavior.
 * Error checking on debug builds prevents this from happening via assertion.
 */
struct Permutation {
public:
    static Permutation identity(size_t size)
    {
        // using std::iota would have been more concise but requires <numeric>
        Permutation result(size);
        for (size_t i = 0; i < size; ++i) {
            result[i] = i;
        }
        return result;
    }

private:
    /// The single vector which stores both the permutation AND the indices of the cycles starts.
    std::vector<size_t> perm;

    /// Constructs an empty permutation initialized to all-zeros.
    Permutation(size_t size) : perm(size) {}

public:
    /// Constructs a permutation from a table.
    /// No error handling is performed, it is assumed the the permutation is valid.
    Permutation(const size_t table[], size_t size) : perm{table, table + size}
    {
        integrityCheck();
    }

    /// Constructs a permutation from a table.
    /// No error handling is performed, it is assumed the the permutation is valid.
    explicit Permutation(std::vector<size_t> table) : perm{std::move(table)}
    {
        integrityCheck();
    }

    Permutation(std::initializer_list<size_t> table) : Permutation{std::vector<size_t>(table)} {}

    /// Constructs an empty permutation.
    /// This permutation represents a universal identity permutation.
    /// Applying it to any array results in no action at all.
    Permutation() : Permutation(0) {}

    Permutation(Permutation &&) = default;
    Permutation(Permutation const &) = default;

    Permutation &operator=(Permutation &&) = default;
    Permutation &operator=(Permutation const &) = default;

    /**
     * @brief Applies the permutation to a source array with the result being stored in a destination array.
     * @param dest the destination
     * @param src the source
     */
    template <typename T>
    void apply(T dest[], const T src[]) const
    {
        for (size_t i = 0; i < size(); ++i) {
            dest[i] = src[perm[i]];
        }
    }

    /// Modifies the current permutation p so that it represents first permuting by p, then by other.
    Permutation append(const Permutation &other) const;

    /// Modifies the current permutation p so that it represents first permuting by other, then by p.
    Permutation prepend(const Permutation &other) const;

    /**
     * @brief Returns a sub-permutation.
     * This is effectively a permutation where the lowest N elements in the input are ignored.
     *
     * Example:
     * (a, A, b, B, c, C, d, D) -> (a, b, c, d, A, B, C, D) using permutation (0, 2, 4, 6, 1, 3, 5, 7)
     * This becomes an short interleaving perm. when discarding the first four inputs (0, 1, 2, 3) or (a, A, b, B):
     * (c, C, d, D) -> (c, d, C, D)
     *
     * @param begin the number of input elements to ignore
     * @return the sub-permutation
     */
    Permutation subPerm(size_t begin) const;

    /**
     * @brief Returns a discarding permutation.
     * This is effectively a permutation where the lowest N elements in the output are ignored.
     *
     * Example:
     * (a, A, b, B, c, C, d, D) -> (a, b, c, d, A, B, C, D) using permutation (0, 2, 4, 6, 1, 3, 5, 7)
     * This becomes an identity permutation when discarding the first four outputs (0, 2, 4, 6) or (a, b, c, d):
     * (A, B, C, D) -> (A, B, C, D)
     *
     * @param begin the number of input elements to ignore
     * @return the sub-permutation
     */
    Permutation disPerm(size_t begin) const;

    /// Returns the inverse permutation.
    Permutation inverse() const;

    /**
     * @brief Returns the size of the permutation.
     * Permutations can be empty so the size may be zero.
     * @return the size of the permutation
     */
    size_t size() const
    {
        return perm.size();
    }

    const size_t *begin() const
    {
        return data();
    }

    const size_t *end()
    {
        return data() + size();
    }

    const size_t *data() const
    {
        return perm.data();
    }

    const size_t &operator[](size_t i) const
    {
        VXIO_DEBUG_ASSERT_LT(i, size());
        return perm[i];
    }

    bool operator==(const Permutation &other) const
    {
        return this->perm == other.perm;
    }

    bool operator!=(const Permutation &other) const
    {
        return this->perm != other.perm;
    }

private:
    size_t *data()
    {
        return perm.data();
    }

    size_t &operator[](size_t i)
    {
        VXIO_DEBUG_ASSERT_LT(i, size());
        return perm[i];
    }

    void integrityCheck() const
    {
        VXIO_DEBUG_ASSERTM(verifyIndicesInRange(), "Invalid permutation was constructed (indices out of range)");
        VXIO_DEBUG_ASSERTM(verifyIndicesUnique(), "Invalid permutation was constructed (duplicate indices)");
    }

    bool verifyIndicesInRange() const
    {
        for (size_t i = 0; i < size(); ++i) {
            if (i >= size()) return false;
        }
        return true;
    }

    bool verifyIndicesUnique() const
    {
        std::vector<bool> found(size());

        for (size_t i = 0; i < size(); ++i) {
            found[perm[i]] = true;
        }
        for (size_t i = 0; i < size(); ++i) {
            if (not found[i]) return false;
        }
        return true;
    }

};

inline Permutation Permutation::append(const Permutation &other) const
{
    VXIO_ASSERT_EQ(size(), other.size());

    Permutation result(size());
    for (size_t i = 0; i < size(); ++i) {
        result[i] = other[perm[i]];
    }

    result.integrityCheck();
    return result;
}

inline Permutation Permutation::prepend(const Permutation &other) const
{
    VXIO_ASSERT_EQ(size(), other.size());

    Permutation result(size());
    for (size_t i = 0; i < size(); ++i) {
        result[i] = perm[other[i]];
    }

    result.integrityCheck();
    return result;
}

inline Permutation Permutation::subPerm(size_t begin) const
{
    using isize = std::make_signed_t<size_t>;
    VXIO_DEBUG_ASSERT_LE(begin, size());

    std::vector<size_t> result(size() - begin);
    const isize ibegin = static_cast<isize>(begin);

    size_t o = 0;
    for (size_t i = 0; i < size(); ++i) {
        isize p = static_cast<isize>(perm[i]);
        p -= ibegin;
        if (p >= 0) {
            size_t pu = static_cast<size_t>(p);
            VXIO_DEBUG_ASSERT_LT(pu, size() - begin);
            result[o++] = pu;
        }
    }

    return Permutation{std::move(result)};
}

inline Permutation Permutation::disPerm(size_t begin) const
{
    using isize = std::make_signed_t<size_t>;
    VXIO_DEBUG_ASSERT_LE(begin, size());
    const isize ibegin = static_cast<isize>(begin);

    std::vector<size_t> low(perm.begin(), perm.begin() + ibegin);
    std::sort(low.begin(), low.end(), std::greater{});
    VXIO_DEBUG_ASSERT_LE(low.size(), begin);

    std::vector<size_t> result(perm.begin() + ibegin, perm.begin() + static_cast<isize>(size()));
    VXIO_DEBUG_ASSERT_LE(result.size(), size() - begin);

    for (size_t i = 0; i < begin; ++i) {
        size_t p = low[i];
        for (size_t j = 0; j < result.size(); ++j) {
            result[j] -= result[j] >= p;
        }
    }

    return Permutation{std::move(result)};
}

inline Permutation Permutation::inverse() const
{
    Permutation result(size());

    for (size_t i = 0; i < size(); ++i) {
        result[operator[](i)] = i;
    }

    return result;
}

}  // namespace flvc

#endif  // FLVC_PERMUTATION_HPP
