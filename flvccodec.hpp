#ifndef FLVC_CODEC_HPP
#define FLVC_CODEC_HPP

#include "svo.hpp"

#include "flvcconst.hpp"
#include "permutation.hpp"

#include "voxelio/src/deflate.hpp"
#include "voxelio/src/endian.hpp"
#include "voxelio/src/streamfwd.hpp"
#include "voxelio/src/types.hpp"

#include <type_traits>
#include <vector>

namespace flvc {

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

/// Extends an attribute definition by additional internally used members.
struct ExtendedAttributeDef : public AttributeDef {
    size_t inputOffset;
};

// DELTA KERNEL ========================================================================================================

struct DeltaKernel {
    static constexpr size_t CAPACITY = 8;

    uint8_t *children[CAPACITY]{};
    uint8_t *parent = nullptr;
    size_t size = 0;

    void pushChild(uint8_t *data)
    {
        VXIO_DEBUG_ASSERT_LT(size, CAPACITY);
        children[size++] = data;
    }

    void encode(const ExtendedAttributeDef defs[], size_t count, size_t posAttrib);
    void decode(const ExtendedAttributeDef defs[], size_t count, size_t posAttrib);

    void reset(uint8_t *parent)
    {
        this->parent = parent;
        size = 0;
    }
};

// BASE ================================================================================================================

class CodecBase {
public:
    using attrib_range_t = std::vector<ExtendedAttributeDef>;

protected:
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

    static constexpr NodeType nodeTypeOf(SvoNodeType type) noexcept
    {
        return type == SvoNodeType::BRANCH ? NodeType::BRANCH : NodeType::LEAF;
    }

    State state = STATE_INITIAL;
    Mode mode;

    size_t decodedAttribSize = 0;
    size_t encodedAttribSize = 1;
    size_t positionAttribIndex = 0;

    attrib_range_t attribDefs;
    std::vector<uint8_t> attribData;

    Permutation ileavePermsWithInternal[8]{};
    Permutation ileavePermsNoInternal[8]{};
    Permutation dileavePermsWithInternal[8]{};
    Permutation dileavePermsNoInternal[8]{};

    DeltaKernel deltaKernel;

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
    const ExtendedAttributeDef &positionDefinition() const noexcept
    {
        VXIO_DEBUG_ASSERT_NE(attribDefs.size(), 0);
        VXIO_DEBUG_ASSERT_GE(state, STATE_POSITION_ATTRIBUTE_DEFINED);
        return attribDefs[positionAttribIndex];
    }

    /**
     * @brief Returns a reference to a constant range which which iterates over the AttributeDef objects stored
     * by the encoder/decoder.
     * @return the range of attribute definitions
     */
    const attrib_range_t &attributeRange() const
    {
        return attribDefs;
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

    /**
     * @brief Generates (de-)interleaving permutations for every possible amount of child nodes.
     * This fills the ileavePermsWithInternal & co. arrays completely.
     */
    void genPermutations() noexcept;

    [[nodiscard]] size_t allocAttribute() noexcept;

    /**
     * @brief Inserts decoded attribute data (so still including the position data) into the attribute data vector.
     * This will crop out the position attribute and also insert one internal byte used for masks.
     * @param attributeData the input data in decoded format
     * @return the base index of the inserted data (aka. the size of the vector before insertion)
     */
    [[nodiscard]] size_t insertAttribute(const uint8_t attributeData[]) noexcept;

    [[nodiscard]] voxelio::Vec3i32 decodePosition(const uint8_t attributeData[]) noexcept;
    size_t encodePosition(voxelio::Vec3i32 pos, uint8_t attributeData[]) noexcept;

private:
    /**
     * @brief Generates the (de-)interleaving permutations for a given node count.
     * This fills the ileavePermsWithInternal & co. buffers at a single index.
     *
     * @param nodeCount the number of nodes to generate the permutation for; in range [1, 8]
     */
    void genPermutations(size_t nodeCount) noexcept;

    /**
     * @brief Generates a de-interleaving permutation for a given node count.
     * @param nodeCount the number of nodes to generate the permutation for
     */
    flvc::Permutation genDeinterleavingPermutation(size_t nodeCount) noexcept;
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

/**
 * @brief The FLVC encoder.
 * Allows for insertion of voxel data which is then written all at once using write().
 *
 * All data except for the header is compressed using zlib.
 * Compression settings can be provided in the constructor.
 */
class Encoder : public CodecBase {
private:
    using svo_type = SparseVoxelOctree<size_t, size_t>;
    using node_type = svo_type::node_type;
    using leaf_type = svo_type::leaf_type;
    using branch_type = svo_type::branch_type;

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
    /**
     * @brief Writes the header to the stream.
     * This requires a position attribute to be defined.
     * @return true if writing was successful
     */
    bool writeHeader();
    /**
     * @brief Writes the (deflated) SVO.
     * @return true if writing was successful
     */
    bool writeSvo();

    /**
     * @brief Performs various optimizations of the SVO.
     * Most notably, geometry of completely filled branches is trimmed and attributes are expressed as deltas to
     * their parents.
     */
    void optimizeSvo();
    void optimizeSvo_initAttribData(node_type *children[], size_t count, branch_type &parent);
    void optimizeSvo_initAttribDataForOneNode(node_type &node, SvoNodeType type);
    void optimizeSvo_reduceCompleteMasks(node_type *children[], size_t count, branch_type &parent);
    void optimizeSvo_computeAttributeDeltas(node_type *children[], size_t count, branch_type &parent);

    /**
     * @brief Writes the attribute data of one node.
     * @param attribIndex the attribute index
     * @param type the type of the node
     * @return true if writing was successful
     */
    bool writeAttribData(size_t attribIndex, NodeType type);

    bool writeNode(node_type &node, SvoNodeType type);
};

// DECODER =============================================================================================================

/**
 * @brief The FLVC decoder.
 */
class Decoder : public CodecBase {
private:
    using index_t = uint32_t;

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

    /**
     * @brief Returns true if the decoder is done or failed.
     * This does not have to be caused by a stream reaching the EOF, but rather by reading the last byte which is part
     * of the SVO.
     * @return true if the decoder is done or failed
     */
    bool done()
    {
        return state >= STATE_IO_DONE;
    }

    bool failed()
    {
        return state == STATE_FAILED;
    }

private:
    [[nodiscard]] ResultCode readHeader_constants() noexcept;
    [[nodiscard]] ResultCode readHeader_attributeDefinitions() noexcept;

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

    /**
     * @brief De-optimizes the top frame. Called right after reading frame attributes.
     * This method reverses all optimizations applied during encoding.
     *
     * The size of the frame is not changed by this operation, but attributes are de-interleaved, bit-de-interleaved,
     * converted from deltas to absolute values etc.
     *
     * @param parentMask the parent mask
     */
    void deoptimizeFrame(Frame &frame, uint8_t parentMask, uint8_t childCount, bool noGeometry) noexcept;

    void deoptimizeFrame_bitDeinterleave(Frame &frame, uint8_t childCount, bool noGeometry) noexcept;
    void deoptimizeFrame_byteInterleave(Frame &frame, uint8_t childCount, bool noGeometry) noexcept;
    void deoptimizeFrame_decodeDeltas(Frame &frame, uint8_t parentMask, bool noGeometry) noexcept;

    // FRAME AND POSITION MANAGEMENT ===================================================================================

    /**
     * @brief Returns the current voxel position.
     * This result is only meaningful after navigating to a voxel (e.g. using goToNextVoxel()).
     * @return the current 3D position
     */
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
