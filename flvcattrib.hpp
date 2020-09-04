#ifndef FLVC_ATTRIB_HPP
#define FLVC_ATTRIB_HPP
/*
 * flvcattrib.hpp
 * --------------
 * This header stores traits and utilities operating on the AttributeType enum.
 * It allows performing type-dependent arithmetic, even when AttributeType is known only at runtime.
 */

#include "flvcconst.hpp"

#include "voxelio/src/endian.hpp"
#include "voxelio/src/primitives.hpp"
#include "voxelio/src/wileave.hpp"

namespace flvc {

using namespace voxelio;

// internalType<AttributeType> =========================================================================================

namespace detail {

template <AttributeType type>
constexpr auto internalType_impl()
{
#define MATCH(attribType, internalType) \
    else if constexpr (type == AttributeType::attribType) return internalType {}

    if constexpr (false) {
        return void(0);
    }
    MATCH(BOOL, u8);
    MATCH(INT_8, i8);
    MATCH(INT_16, i16);
    MATCH(INT_32, i32);
    MATCH(INT_64, i64);
    MATCH(UINT_8, u8);
    MATCH(UINT_16, u16);
    MATCH(UINT_32, u32);
    MATCH(UINT_64, u64);
    MATCH(FLOAT_32, f32);
    MATCH(FLOAT_64, f64);
#undef MATCH
    else
    {
        VXIO_DEBUG_ASSERT_UNREACHABLE();
    }
}

}  // namespace detail

template <AttributeType type>
using internalType = decltype(detail::internalType_impl<type>());

static_assert(std::is_same_v<internalType<AttributeType::INT_32>, i32>);
static_assert(std::is_same_v<internalType<AttributeType::FLOAT_64>, f64>);

// safeMakeUnsigned<Arithmetic> ========================================================================================

namespace detail {

template <typename T>
auto safeMakeUnsigned_impl()
{
    if constexpr (std::is_same_v<T, f64>) {
        return uint64_t{0};
    }
    else if constexpr (std::is_same_v<T, f32>) {
        return uint32_t{0};
    }
    else {
        static_assert(std::is_integral_v<T>);
        return std::make_unsigned_t<T>{0};
    }
}

}  // namespace detail

/**
 * @brief Alternative to std::make_unsigned_t which converts any arithmetic type to an unsigned integer type of equal
 * size.
 * For example, double will be converted to uint64_t if double is a 64-bit type.
 */
template <typename Arithmetic, std::enable_if_t<std::is_arithmetic_v<Arithmetic>, int> = 0>
using safeMakeUnsigned = decltype(detail::safeMakeUnsigned_impl<Arithmetic>());

// ENCODE, DECODE, ILEAVE, DILEAVE =====================================================================================

namespace detail {

template <typename T, AttributeType type>
constexpr T decodeAttrib_impl(const u8 data[])
{
    using itype = internalType<type>;

    // signed/unsigned mismatch
    VXIO_DEBUG_ASSERT_EQ(std::is_signed_v<T>, isSigned(type));

    itype result = voxelio::decodeNative<itype>(data);

    if constexpr (isInteger(type)) {
        constexpr T maxRepresentable = std::numeric_limits<T>::max();
        VXIO_DEBUG_ASSERT_LE(result, maxRepresentable);
    }

    return static_cast<T>(result);
}

template <typename T, AttributeType type>
constexpr void encodeAttrib_impl(T value, u8 out[])
{
    using itype = internalType<type>;

    // signed/unsigned mismatch
    VXIO_DEBUG_ASSERT_EQ(std::is_signed_v<T>, isSigned(type));

    if constexpr (isInteger(type)) {
        constexpr itype maxRepresentable = std::numeric_limits<itype>::max();
        VXIO_DEBUG_ASSERT_LE(value, maxRepresentable);
    }

    itype result = static_cast<itype>(value);
    voxelio::encodeNative<itype>(result, out);
}

/**
 * @brief Interleaves an array of values of some given type and stores the result in an array of bytes.
 * The array of output bytes will be in little-endian byte- AND bit-order.
 */
template <typename T>
constexpr void ileaveToLittleBytes(const u8 in[], u8 out[], usize count)
{
    using DataType = safeMakeUnsigned<T>;

    constexpr usize maxByteCount = sizeof(DataType) * 8;
    constexpr usize max64count = divCeil(maxByteCount, sizeof(u64));

    VXIO_DEBUG_ASSERT_NE(count, 0);
    VXIO_DEBUG_ASSERT_LE(count, 8);

    const usize byteCount = count * sizeof(DataType);

    u64 outBuffer[max64count]{};

    if constexpr (not std::is_floating_point_v<T>) {
        DataType inBuffer[8]{};
        std::memcpy(inBuffer, in, byteCount);
        wide::ileave(inBuffer, outBuffer, count);
    }
    else {
        std::memcpy(outBuffer, in, byteCount);
    }

    if constexpr (Endian::NATIVE != Endian::LITTLE) {
        const usize u64count = divCeil(count, sizeof(u64));
        for (size_t i = 0; i < u64count; ++i) {
            outBuffer[i] = reverseBytes(outBuffer[i]);
        }
    }
    std::memcpy(out, outBuffer, byteCount);
}

template <typename T>
constexpr void dileaveLittleBytes(const u8 in[], u8 out[], usize count)
{
    using DataType = safeMakeUnsigned<T>;

    constexpr usize maxByteCount = sizeof(T) * 8;
    constexpr usize max64count = divCeil(maxByteCount, sizeof(u64));

    VXIO_DEBUG_ASSERT_NE(count, 0);
    VXIO_DEBUG_ASSERT_LE(count, 8);

    const usize byteCount = count * sizeof(T);

    u64 inBuffer[max64count]{};
    std::memcpy(inBuffer, in, byteCount);

    if constexpr (Endian::NATIVE != Endian::LITTLE) {
        const usize u64count = divCeil(count, sizeof(u64));
        for (size_t i = 0; i < u64count; ++i) {
            inBuffer[i] = reverseBytes(inBuffer[i]);
        }
    }

    if constexpr (not std::is_floating_point_v<T>) {
        DataType outBuffer[8]{};
        wide::dileave(inBuffer, outBuffer, count);
        std::memcpy(out, outBuffer, byteCount);
    }
    else {
        std::memcpy(out, inBuffer, byteCount);
    }
}

}  // namespace detail

// SWITCH TABLES FOR RUNTIME DECISIONS =================================================================================

template <typename T>
[[nodiscard]] constexpr T decodeAttrib(const u8 data[], AttributeType type)
{
#define CASE(E) \
    case AttributeType::E: return detail::decodeAttrib_impl<T, AttributeType::E>(data)
    switch (type) {
        CASE(BOOL);
        CASE(INT_8);
        CASE(INT_16);
        CASE(INT_32);
        CASE(INT_64);
        CASE(UINT_8);
        CASE(UINT_16);
        CASE(UINT_32);
        CASE(UINT_64);
        CASE(FLOAT_32);
        CASE(FLOAT_64);
    }
#undef CASE
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T>
constexpr void encodeAttrib(T value, u8 out[], AttributeType type)
{
#define CASE(E) \
    case AttributeType::E: return detail::encodeAttrib_impl<T, AttributeType::E>(value, out);
    switch (type) {
        CASE(BOOL);
        CASE(INT_8);
        CASE(INT_16);
        CASE(INT_32);
        CASE(INT_64);
        CASE(UINT_8);
        CASE(UINT_16);
        CASE(UINT_32);
        CASE(UINT_64);
        CASE(FLOAT_32);
        CASE(FLOAT_64);
    }
#undef CASE
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

constexpr void ileaveAttrib(const u8 in[], u8 out[], usize count, AttributeType type)
{
#define CASE(E) \
    case AttributeType::E: return detail::ileaveToLittleBytes<internalType<AttributeType::E>>(in, out, count);
    switch (type) {
        CASE(BOOL);
        CASE(INT_8);
        CASE(INT_16);
        CASE(INT_32);
        CASE(INT_64);
        CASE(UINT_8);
        CASE(UINT_16);
        CASE(UINT_32);
        CASE(UINT_64);
        CASE(FLOAT_32);
        CASE(FLOAT_64);
    }
#undef CASE
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

constexpr void dileaveAttrib(const u8 in[], u8 out[], usize count, AttributeType type)
{
#define CASE(E) \
    case AttributeType::E: return detail::dileaveLittleBytes<internalType<AttributeType::E>>(in, out, count);
    switch (type) {
        CASE(BOOL);
        CASE(INT_8);
        CASE(INT_16);
        CASE(INT_32);
        CASE(INT_64);
        CASE(UINT_8);
        CASE(UINT_16);
        CASE(UINT_32);
        CASE(UINT_64);
        CASE(FLOAT_32);
        CASE(FLOAT_64);
    }
#undef CASE
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

}  // namespace flvc

#endif  // FLVCATTRIB_HPP
