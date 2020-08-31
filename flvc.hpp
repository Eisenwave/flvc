#ifndef FLVC_HPP
#define FLVC_HPP

#ifdef MVE_MAJOR
#include "core_structs/svo.hpp"
#else
#include "svo.hpp"
#endif

#include "voxelio/src/deflate.hpp"
#include "voxelio/src/endian.hpp"
#include "voxelio/src/streamfwd.hpp"
#include "voxelio/src/types.hpp"

#include <type_traits>
#include <vector>

namespace flvc {

// CONFIG ==============================================================================================================

// This flag can help with manual debugging of the output data in a hex editor.
// NEVER push this with a true value.
constexpr bool ANNOTATE_BINARY = false;

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

// ATTRIBUTE DEFINITIONS ===============================================================================================

struct AttributeDef {
    /// Convenience function for defining the position attribute.
    static AttributeDef position(AttributeType type)
    {
        VXIO_DEBUG_ASSERT(isInteger(type));
        return {"position", type, 3, 0};
    }

    /// Convenience function for defining the rgb attribute.
    static AttributeDef rgb(AttributeType type)
    {
        VXIO_DEBUG_ASSERT(isInteger(type) && isUnsigned(type));
        return {"rgb", type, 3, 0};
    }

    /// Convenience function for defining the argb attribute.
    static AttributeDef argb(AttributeType type)
    {
        VXIO_DEBUG_ASSERT(isInteger(type) && isUnsigned(type));
        return {"argb", type, 4, 0};
    }

    std::string identifier;
    AttributeType type;
    uint8_t cardinality;
    uint16_t modifiers = 0;

    AttributeDef(std::string identifier, AttributeType type, uint8_t cardinality = 1, uint16_t modifiers = 0)
        : identifier{std::move(identifier)}, type{type}, cardinality{cardinality}, modifiers{modifiers}
    {
    }

    AttributeDef(const AttributeDef &) = default;
    AttributeDef(AttributeDef &&) = default;

    size_t size() const
    {
        return sizeOf(type) * cardinality;
    }

    size_t elementSize() const
    {
        return sizeOf(type);
    }
};

// BASE ================================================================================================================

class CodecBase {
protected:
    /// Extends an attribute definition by additional internally used members.
    struct InternalAttributeDef : public AttributeDef {
        size_t inputOffset;
    };

    /// Type of encoded node. Can be either leaf, branch or voxel.
    enum class NodeType : signed char {
        LEAF = 0,
        BRANCH = 1,
        COMPLETE = -1,
        VOXEL = -2,
    };

    enum class Mode : unsigned char { ENCODE = 0, DECODE = 1 };

    /// The encoder's state. Transitions between states are expected to only be in one direction.
    enum State : unsigned  // not an enum class because arithmetic operators are used regularly
    {
        /// Default state. Nothing has been done.
        STATE_INITIAL = 0,
        /// The position attribute has been defined.
        STATE_POSITION_ATTRIBUTE_DEFINED = 1,
        /// Voxels have been inserted for writing. (Encoder-only)
        STATE_VOXELS_INSERTED = 2,
        /// Read or write has started. Indicates that the header was read/written and SVO data is being processed now.
        STATE_IO_STARTED = 3,
        /// An intermediate state between IO_STARTED and IO_DONE.
        STATE_DURING_IO = 4,
        /// Read or write has ended.
        STATE_IO_DONE = 5,
        /// Voxels have been fully extracted. (Decoder-only)
        STATE_VOXELS_EXTRACTED = 6,
        /// Failed state. Using the encoder/decoder in this state is illegal.
        STATE_FAILED = 7
    };

    static constexpr char annotationCharOf(NodeType type)
    {
#define ENUM_CASE(e) \
    case NodeType::e: return #e[0] + 'a' - 'A';
        switch (type) {
            ENUM_CASE(LEAF);
            ENUM_CASE(BRANCH);
            ENUM_CASE(COMPLETE);
            ENUM_CASE(VOXEL);
        }
        VXIO_DEBUG_ASSERT_UNREACHABLE();
#undef ENUM_CASE
    }

    static constexpr bool hasGeometry(NodeType type)
    {
        return static_cast<std::underlying_type_t<NodeType>>(type) >= 0;
    }

    static constexpr bool isComplete(NodeType type)
    {
        return static_cast<std::underlying_type_t<NodeType>>(type) < 0;
    }

    static constexpr NodeType nodeTypeOf(mve::SvoNodeType type) noexcept
    {
        return type == mve::SvoNodeType::BRANCH ? NodeType::BRANCH : NodeType::LEAF;
    }

    State state = STATE_INITIAL;
    Mode mode;

    size_t decodedAttribSize = 0;
    size_t encodedAttribSize = 1;
    size_t positionAttribIndex = 0;

    std::vector<InternalAttributeDef> attribDefs;
    std::vector<uint8_t> attribData;

    CodecBase(Mode mode) noexcept : mode{mode} {}

public:
    /**
     * @brief Defines an attribute.
     *
     * This step must be performed before inserting any data into the SVO.
     * The minimum definition required is one 3D-attribute exactly named "position".
     * This special attribute will be used for positioning inside the SVO.
     *
     * Calling this method after having inserted data into the SVO or having written data results in an error.
     *
     * @param def the attribute definition
     * @return the result code
     */
    [[nodiscard]] ResultCode defineAttribute(AttributeDef def) noexcept;

    /**
     * @brief Returns the current definition of the position attribute.
     * @return
     */
    const InternalAttributeDef &positionDefinition() const noexcept
    {
        VXIO_DEBUG_ASSERT_NE(attribDefs.size(), 0);
        VXIO_DEBUG_ASSERT_GE(state, STATE_POSITION_ATTRIBUTE_DEFINED);
        return attribDefs[positionAttribIndex];
    }

protected:
    /**
     * @brief Copies attribute definitions from another encoder/decoder.
     * This is a convenience method which simplifies defining attributes for a pair of encoders/decoders.
     * @param from the source of attribute definitions
     */
    void copyAttributeDefinitons(const CodecBase &from) noexcept
    {
        attribData = from.attribData;
        decodedAttribSize = from.decodedAttribSize;
        encodedAttribSize = from.encodedAttribSize;
        positionAttribIndex = from.positionAttribIndex;
        if (from.state >= STATE_POSITION_ATTRIBUTE_DEFINED && this->state == STATE_INITIAL) {
            this->state = STATE_POSITION_ATTRIBUTE_DEFINED;
        }
    }

    [[nodiscard]] size_t allocAttribute() noexcept;
    [[nodiscard]] size_t insertAttribute(const uint8_t attributeData[]) noexcept;

    [[nodiscard]] voxelio::Vec3i32 decodePosition(const uint8_t attributeData[]) noexcept;
    size_t encodePosition(voxelio::Vec3i32 pos, uint8_t attributeData[]) noexcept;
};

struct Header {
    /**
     * @brief The global offset of all voxels stored in the file.
     * This is the minimum of the volume.
     */
    voxelio::Vec3i32 offset;
    /// The dimensions of the voxel volume.
    voxelio::Vec3u32 size;
    /// True if the volume is empty.
    bool empty;
    /**
     * @brief The single-octant depth of the SVO.
     * This is the ceiled binary logarithm of the greatest volume dimension.
     * The smallest possible depth is 0, which is only the the case for a 1x1x1 volume.
     */
    unsigned svoDepth;
};

// ENCODER =============================================================================================================

class Encoder : public CodecBase {
private:
    using svo_type = mve::SparseVoxelOctree<size_t, 0, size_t>;
    using node_type = svo_type::node_type;
    using leaf_type = svo_type::leaf_type;
    using branch_type = svo_type::branch_type;

    using SvoNodeType = mve::SvoNodeType;

    voxelio::OutputStream &stream;
    voxelio::deflate::Deflator deflator;

    svo_type svo;

public:
    Encoder(voxelio::OutputStream &stream, voxelio::deflate::DeflateSettings settings = {})
        : CodecBase{Mode::ENCODE}, stream{stream}, deflator{stream, settings}
    {
    }

    /**
     * @brief Inserts a buffer of data into the SVO.
     * The layout of this data can be specified using defineAttribute().
     *
     * @param data the attribute data
     * @param voxelCount the number of voxels
     * @return the result code
     */
    [[nodiscard]] ResultCode insert(const uint8_t data[], size_t voxelCount);

    /**
     * @brief Writes the SVO data to the stream.
     * @return the result code
     */
    [[nodiscard]] ResultCode write();

private:
    bool writeHeader();
    void optimizeSvo();
    bool writeSvo();

    bool writeAttribData(size_t attribIndex, NodeType type);
    bool writeNode(node_type &node, SvoNodeType type);
};

// DECODER =============================================================================================================

class Decoder : public CodecBase {
private:
    using index_t = uint32_t;

    using SvoNodeType = mve::SvoNodeType;

    struct Frame {
        /// The data of the eight nodes on the current stack level.
        index_t nodes[8];
        /// The size of the attribute data before the frame was pushed.
        index_t baseSize;
        /// The node index on the current level [0, 8).
        uint8_t currIndex;
        /// True if this branch is complete. Complete branches only have attribute information and no geometry
        /// information.
        bool complete;

        index_t &currAttrib()
        {
            VXIO_DEBUG_ASSERT_LT(currIndex, 8);
            return nodes[currIndex];
        }
    };

    voxelio::InputStream &stream;
    voxelio::deflate::Inflator inflator;
    Header header;

    Frame frameStack[21] VXIO_IF_DEBUG({});
    uint64_t morton = 0;
    /// The index in the frame stack at which the next frame will be inserted.
    /// Only zero before root has been pushed and after it was popped.
    uint8_t depth = 0;

public:
    Decoder(voxelio::InputStream &stream, unsigned windowBits = voxelio::deflate::DEFAULT_WINDOW_BITS)
        : CodecBase{Mode::DECODE}, stream{stream}, inflator{stream, windowBits}
    {
        // pessimistic attribute size * pessimistic max frame stack depth * nodes per frame
        attribData.reserve(64 * 32 * 8);
    }

    /**
     * @brief Reads the FLVC header.
     * This makes attribute definitions available which can then be used to interpret the buffer data during
     * readVoxels().
     *
     * This method must be called before readVoxels().
     *
     * @return the result code
     */
    [[nodiscard]] ResultCode readHeader();

    /**
     * @brief Returns the read header information.
     * readHeader() must be called before getting the header, otherwise the result will be uninitialized.
     *
     * @return the read header
     */
    [[nodiscard]] const Header &getHeader()
    {
        return header;
    }

    /**
     * @brief Reads the next N voxels into the given buffer.
     * This method attempts to fill up as much of the buffer as possible.
     *
     * When the end of the stream is reached, outVoxelsRead will be set to zero.
     * Otherwise it will never be zero.
     *
     * readHeader() must be called before this method.
     *
     * @param out the output buffer
     * @param bufferSize the size of the buffer in bytes
     * @param outVoxelsRead the number of voxels actually read or an undefined value if an error occurs
     * @return the result code
     */
    [[nodiscard]] ResultCode readVoxels(uint8_t out[], size_t bufferSize, size_t &outVoxelsRead);

    bool eof()
    {
        return state >= STATE_IO_DONE;
    }

private:
    void decodeVoxelData(const uint8_t in[], uint8_t out[]) noexcept;

    /**
     * @brief Read the frame attribute data of the next frame.
     * This method requires the given frame to be initialized except for the attribute data.
     *
     * @param frame the almost fully initialized frame
     * @param parentMask the parent mask of the frame (nonzero)
     * @return the result code
     */
    [[nodiscard]] ResultCode readFrameAttributes(Frame &frame, uint8_t parentMask) noexcept;

    // FRAME AND POSITION MANAGEMENT ===================================================================================

    voxelio::Vec3i32 position() noexcept;

    /**
     * @brief Pushes one frame of nodes onto the internal node stack for decoding.
     * @param nodes the child nodes of the parent
     * @return true if the push was successful
     */
    bool pushFrame(Frame frame) noexcept;

    /**
     * @brief Pops one frame of nodes from the stack.
     * @return true if the pop was possible, false if the stack depth is zero
     */
    bool popFrame() noexcept;

    /**
     * @brief Changes the position within the current frame.
     * @param index the new index in [0, 8)
     * @return true if navigation was possible, false if the stack depth is zero
     */
    bool navFrame(uint8_t index) noexcept;

    Frame &parentFrame() noexcept
    {
        VXIO_DEBUG_ASSERT_GE(depth, 2);
        return frameStack[depth - 2];
    }

    Frame &topFrame() noexcept
    {
        VXIO_DEBUG_ASSERT_NE(depth, 0);
        return frameStack[depth - 1];
    }

    // HIGHER LEVEL SVO NAGIVATION =====================================================================================

    /**
     * @brief Pops a frame and goes sideways to the next branch in the parent.
     * This attempt of going sideways may result in finishing another frame, so this function is possibly recursive.
     * At the end of finishFrame(), the implicit iterator will be at the next valid parent position.
     *
     * @return true if the frame could be finished, false if this failed because there are no frames to pop
     */
    [[nodiscard]] bool finishFrame() noexcept;

    [[nodiscard]] bool goSideways() noexcept;

    [[nodiscard]] ResultCode readRoot() noexcept;

    [[nodiscard]] ResultCode goDeeper() noexcept;

    [[nodiscard]] ResultCode goToNextVoxel(bool &outEof) noexcept;
};

}  // namespace flvc

#endif  // SVOENCODE_HPP
