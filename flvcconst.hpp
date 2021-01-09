#ifndef FLVC_CONST_HPP
#define FLVC_CONST_HPP
/*
 * flvcconst.hpp
 * -------------
 * Defines constants and enumerations such as AttributeType and ResultCode which are used by FLVC.
 */

#include "voxelio/assert.hpp"
#include "voxelio/intlog.hpp"

#include <array>
#include <cstdint>

namespace flvc {

// ENUMS & CONSTANTS ===================================================================================================

enum class AttributeType : uint8_t {
    // clang-format off
    /// 8-bit unsigned integer where any nonzero value is interpreted as true
    BOOL     = 0x01,

    /// 8-bit signed integer
    INT_8    = 0x11,
    /// 16-bit signed integer
    INT_16   = 0x12,
    /// 32-bit signed integer
    INT_32   = 0x14,
    /// 64-bit signed integer
    INT_64   = 0x18,

    /// 8-bit unsigned integer
    UINT_8   = 0x21,
    /// 16-bit unsigned integer
    UINT_16  = 0x22,
    /// 32-bit unsigned integer
    UINT_32  = 0x24,
    /// 64-bit unsigned integer
    UINT_64  = 0x28,

    /// 32-bit floating point
    FLOAT_32 = 0x34,
    /// 64-bit floating point
    FLOAT_64 = 0x38,

    /// Q.8 fixed point number where raw ~0 is interpreted as 1.
    // UFRAC_8   = 0x41,
    /// Q.16 fixed point number where raw ~0 is interpreted as 1.
    // UFRAC_16  = 0x42,
    /// Q.32 fixed point number where raw ~0 is interpreted as 1.
    // UFRAC_32  = 0x44,
    /// Q.64 fixed point number where raw ~0 is interpreted as 1.
    // UFRAC_64  = 0x48
    // clang-format on
};

constexpr std::array<AttributeType, 11> ATTRIBUTE_TYPE_VALUES = {
    AttributeType::BOOL,
    AttributeType::INT_8,
    AttributeType::INT_16,
    AttributeType::INT_32,
    AttributeType::INT_64,
    AttributeType::UINT_8,
    AttributeType::UINT_16,
    AttributeType::UINT_32,
    AttributeType::UINT_64,
    AttributeType::FLOAT_32,
    AttributeType::FLOAT_64,
};

constexpr bool isValid(AttributeType type)
{
    auto rawType = static_cast<uint8_t>(type);
    return type == AttributeType::BOOL || (rawType >= 0x10 && rawType <= 0x38 && voxelio::isPow2(rawType & 0xfu));
}

static_assert(isValid(AttributeType::BOOL));
static_assert(isValid(AttributeType::UINT_64));
static_assert(isValid(AttributeType::FLOAT_64));
static_assert(not isValid(AttributeType{0x39}));
static_assert(not isValid(AttributeType{2}));
static_assert(not isValid(AttributeType{0x10}));

/**
 * @brief Returns the size of an attribute type in bytes.
 * @param type the type
 * @return the size of the type in bytes
 */
constexpr size_t sizeOf(AttributeType type)
{
    return static_cast<size_t>(type) & 0xf;
}

/// Returns true if the type is an integer. BOOL is also treated as an integer.
constexpr bool isInteger(AttributeType type)
{
    return static_cast<uint8_t>(type) < static_cast<uint8_t>(AttributeType::FLOAT_32);
}

/// Returns true if the type is signed. (signed integers, floats, ...)
constexpr bool isSigned(AttributeType type)
{
    return (static_cast<uint8_t>(type) & 0x10) != 0;
}

/// Returns true if the type is unsigned. (unsigned integers, bool, unsigned fractions, ...)
constexpr bool isUnsigned(AttributeType type)
{
    return (static_cast<uint8_t>(type) & 0x10) == 0;
}

/// Returns true if the type is a floating point type.
constexpr bool isFloat(AttributeType type)
{
    return static_cast<uint8_t>(type) >= static_cast<uint8_t>(AttributeType::FLOAT_32);
}

enum class ResultCode : uint8_t {
    /// Operation was performed successfully.
    OK,
    /// An attempt was made to define an attribute which has already been defined.
    MULTIPLE_DEFINITIONS,
    /// Voxels were inserted before defining at least a position attribute.
    INSERT_BEFORE_DEFINE,
    /// Voxels were inserted after already writing the SVO.
    INSERT_AFTER_WRITE,
    /// The SVO was written before defining at least a position attribute.
    WRITE_BEFORE_DEFINE,
    /// An attribute was defined after inserting attribute data.
    DEFINE_AFTER_INSERT,
    /// An attribute was defined after writing the SVO.
    DEFINE_AFTER_WRITE,
    /// Voxel data was read before the position attribute was extracted from the header.
    READ_BEFORE_DEFINE,
    /// An I/O error occured when writing the SVO.
    IO_ERROR,
    /// Unexpected EOF reached.
    EOF_ERROR,
    /// The encoder was used after entering a failed state (due to I/O error).
    USE_AFTER_FAIL,
    /// An identifier was given to an attribute which is longer than 255 characters and can't be stored.
    IDENTIFIER_TOO_LONG,
    /// Magic bytes don't match when reading.
    MAGIC_MISMATCH,
    /// The read version is not supported.
    VERSION_UNSUPPORTED,
    /// The read header has no attributes.
    HEADER_MISSING_ATTRIBUTES,
    /// The read header does not define a position attribute
    HEADER_MISSING_POSITION_ATTRIBUTE,
    /// A bool was read which is outside the range [0, 1].
    CORRUPTED_BOOL,
    /// An enum was read which is not a named constant.
    CORRUPTED_ENUM,
    /// The encoded SVO is deeper than expected.
    SVO_TOO_DEEP,
    /// The frame stack was popped without any node on it.
    EMPTY_STACK_POP,
    /// Invalid dimensions were read. For FLVC, this means that one dimension was zero while the others weren't.
    DEGENERATE_DIMENSIONS,
    /// The dimensions of the SVO are zero, but the empty flag is not set.
    EMPTYNESS_CONTRADICTION,
    /// A reserved value had an unexpected value.
    RESERVED_MISMATCH
};

constexpr const char *nameOf(ResultCode code)
{
#define ENUM_CASE(e) \
    case ResultCode::e: return #e
    switch (code) {
        ENUM_CASE(OK);
        ENUM_CASE(MULTIPLE_DEFINITIONS);
        ENUM_CASE(HEADER_MISSING_ATTRIBUTES);
        ENUM_CASE(HEADER_MISSING_POSITION_ATTRIBUTE);
        ENUM_CASE(INSERT_BEFORE_DEFINE);
        ENUM_CASE(INSERT_AFTER_WRITE);
        ENUM_CASE(WRITE_BEFORE_DEFINE);
        ENUM_CASE(DEFINE_AFTER_INSERT);
        ENUM_CASE(DEFINE_AFTER_WRITE);
        ENUM_CASE(IO_ERROR);
        ENUM_CASE(EOF_ERROR);
        ENUM_CASE(USE_AFTER_FAIL);
        ENUM_CASE(IDENTIFIER_TOO_LONG);
        ENUM_CASE(MAGIC_MISMATCH);
        ENUM_CASE(VERSION_UNSUPPORTED);
        ENUM_CASE(CORRUPTED_BOOL);
        ENUM_CASE(CORRUPTED_ENUM);
        ENUM_CASE(READ_BEFORE_DEFINE);
        ENUM_CASE(EMPTY_STACK_POP);
        ENUM_CASE(SVO_TOO_DEEP);
        ENUM_CASE(DEGENERATE_DIMENSIONS);
        ENUM_CASE(EMPTYNESS_CONTRADICTION);
        ENUM_CASE(RESERVED_MISMATCH);
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
#undef ENUM_CASE
}

}  // namespace flvc

#endif  // FLVCCONST_HPP
