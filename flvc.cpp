#include "flvc.hpp"

#include "voxelio/src/deflate.hpp"
#include "voxelio/src/log.hpp"
#include "voxelio/src/stream.hpp"

namespace flvc {

using namespace voxelio;

namespace {

// STATIC UTILITY ======================================================================================================

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

namespace detail {

template <typename T, AttributeType type>
constexpr T decodeAttrib_impl(const uint8_t data[])
{
    using itype = internalType<type>;

    itype result = voxelio::decodeNative<itype>(data);
    return static_cast<T>(result);
}

template <typename T, AttributeType type>
constexpr void encodeAttrib_impl(T value, uint8_t out[])
{
    using itype = internalType<type>;

    itype result = static_cast<itype>(value);
    voxelio::encodeNative<itype>(result, out);
}

}  // namespace detail

template <typename T>
[[nodiscard]] constexpr T decodeAttrib(const uint8_t data[], AttributeType type)
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
constexpr void encodeAttrib(T value, uint8_t out[], AttributeType type)
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

}  // namespace

// ATTRIBUTE MANAGEMENT ================================================================================================

ResultCode CodecBase::defineAttribute(AttributeDef def) noexcept
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state >= STATE_VOXELS_INSERTED)
        return state == STATE_VOXELS_INSERTED ? ResultCode::DEFINE_AFTER_INSERT : ResultCode::DEFINE_AFTER_WRITE;

    if (def.identifier.length() > 255) {
        return ResultCode::IDENTIFIER_TOO_LONG;
    }

    size_t inputOffset = attribDefs.empty() ? 0 : attribDefs.back().inputOffset + attribDefs.back().size();

    if (def.identifier == "position") {
        if (state != STATE_INITIAL) {
            return ResultCode::MULTIPLE_DEFINITIONS;
        }
        positionAttribIndex = attribDefs.size();
        state = STATE_POSITION_ATTRIBUTE_DEFINED;
    }
    else {
        encodedAttribSize += def.size();
    }

    decodedAttribSize += def.size();

    attribDefs.push_back({std::move(def), inputOffset});

    return ResultCode::OK;
}

size_t CodecBase::allocAttribute() noexcept
{
    size_t allocSize = mode == Mode::ENCODE ? encodedAttribSize : decodedAttribSize;
    VXIO_DEBUG_ASSERT_NE(allocSize,
                         0);  // encoded will include at least one byte, decoded at least the bytes of positions
    size_t result = attribData.size();
    attribData.resize(attribData.size() + allocSize);
    return result;
}

Vec3i32 CodecBase::decodePosition(const u8 data[]) noexcept
{
    const InternalAttributeDef &posDef = positionDefinition();

    const size_t memberOffset = posDef.elementSize();
    const u8 *posDataBegin = data + posDef.inputOffset;

    i32 x = decodeAttrib<i32>(posDataBegin + memberOffset * 0, posDef.type);
    i32 y = decodeAttrib<i32>(posDataBegin + memberOffset * 1, posDef.type);
    i32 z = decodeAttrib<i32>(posDataBegin + memberOffset * 2, posDef.type);

    return {x, y, z};
}

usize CodecBase::encodePosition(Vec3i32 pos, u8 out[]) noexcept
{
    const InternalAttributeDef &posDef = positionDefinition();
    VXIO_DEBUG_ASSERT_EQ(posDef.cardinality, 3);

    const size_t memberOffset = posDef.elementSize();

    encodeAttrib<i32>(pos.x(), out + memberOffset * 0, posDef.type);
    encodeAttrib<i32>(pos.y(), out + memberOffset * 1, posDef.type);
    encodeAttrib<i32>(pos.z(), out + memberOffset * 2, posDef.type);

    return posDef.size();
}

size_t CodecBase::insertAttribute(const u8 data[]) noexcept
{
    VXIO_ASSERT_GE(state, STATE_POSITION_ATTRIBUTE_DEFINED);
    const InternalAttributeDef &posDef = attribDefs[positionAttribIndex];

    size_t result = attribData.size();
    // one internal byte to be used for complete branch optimization
    attribData.push_back(0);

    if (encodedAttribSize != 1) {
        // The input attribute gets sliced out.
        // For example, if we have input data AAAPPPPBB where P is position attribute data, we insert only AAA and BB.

        const u8 *posDataBegin = data + posDef.inputOffset;
        const u8 *posDataEnd = posDataBegin + posDef.size();
        const u8 *inputEnd = data + decodedAttribSize;

        attribData.insert(attribData.end(), data, posDataBegin);
        attribData.insert(attribData.end(), posDataEnd, inputEnd);
    }

    VXIO_DEBUG_ASSERT_EQ(attribData.size() - result, encodedAttribSize);
    return result;
}

// WRITING VOXEL DATA ==================================================================================================

ResultCode Encoder::insert(const u8 data[], size_t voxelCount)
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state < STATE_POSITION_ATTRIBUTE_DEFINED) return ResultCode::INSERT_BEFORE_DEFINE;
    if (state > STATE_VOXELS_INSERTED) return ResultCode::INSERT_AFTER_WRITE;

    state = STATE_VOXELS_INSERTED;

    // TODO optimize specially for geometry-only
    for (size_t i = 0; i < voxelCount; ++i) {
        Vec3i32 pos = decodePosition(data);
        size_t attribIndex = insertAttribute(data);
        svo.insert(pos, attribIndex);

        data += decodedAttribSize;
    }

    return ResultCode::OK;
}

// SVO DESERIALIZATION =================================================================================================

constexpr u64 MAGIC = 0xff11'33cc'666c'7663;
constexpr u8 VERSION_MAJOR = 0;
constexpr u8 VERSION_MINOR = 1;

ResultCode Encoder::write()
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state == STATE_IO_DONE) return ResultCode::OK;

    optimizeSvo();

    if (state < STATE_IO_STARTED) {
        state = STATE_IO_STARTED;
        if (not writeHeader() || stream.err()) {
            state = STATE_FAILED;
            return ResultCode::IO_ERROR;
        }
    }
    if (state == STATE_IO_STARTED) {
        state = STATE_IO_DONE;
        if (not writeSvo() || stream.err()) {
            state = STATE_FAILED;
            return ResultCode::IO_ERROR;
        }
    }

    return ResultCode::OK;
}

void Encoder::optimizeSvo()
{
    // create attribute data for all nodes, initialize the first byte to the mask
    // for leafs where the node is full, initialize to 0
    svo.forEachNodeTopDown([this](node_type *node, SvoNodeType type) -> void {
        VXIO_DEBUG_ASSERT(not node->empty());
        size_t resultIndex = allocAttribute();
        auto mask = static_cast<u8>(node->mask().to_ulong());
        if (type == SvoNodeType::LEAF && node->full()) {
            mask = 0;
        }
        VXIO_DEBUG_ASSERT_LT(resultIndex, attribData.size());
        attribData[resultIndex] = mask;
        node->value() = resultIndex;
    });

    // for complete nodes, mark the parent as complete as well by setting the parent's byte to 0
    // this happens recursively
    reduceNodes(svo, [this](node_type *children[], size_t count, branch_type &parent) -> void {
        if (not parent.full()) {
            return;
        }
        uint8_t combinedChildMask = 0;
        for (size_t i = 0; i < count; ++i) {
            size_t childAttribIndex = children[i]->value();
            VXIO_DEBUG_ASSERT_LT(childAttribIndex, attribData.size());
            combinedChildMask |= attribData[childAttribIndex];
        }
        if (combinedChildMask == 0) {
            size_t parentAttribIndex = parent.value();
            VXIO_DEBUG_ASSERT_LT(parentAttribIndex, attribData.size());
            attribData[parentAttribIndex] = 0;
        }
    });

    // retroactively fix the root node (commented out because not necessary for now)
    /* size_t rootAttribIndex = svo.rootNode().value();
    attribData[rootAttribIndex] = static_cast<u8>(svo.rootNode().mask().to_ulong()); */
}

#define DANNOTATE(str) \
    if constexpr (ANNOTATE_BINARY) stream.writeString("(" + std::string(str) + ")")

bool Encoder::writeHeader()
{
    constexpr u8 version[]{VERSION_MAJOR, VERSION_MINOR};

    stream.writeBig(MAGIC);
    DANNOTATE("version");
    stream.write(version, 2);

    DANNOTATE("definitions_size");
    stream.writeLittle(static_cast<u16>(attribDefs.size()));
    DANNOTATE("definitions");
    for (const AttributeDef &attr : attribDefs) {
        DANNOTATE("identifier::length");
        stream.writeU8(static_cast<u8>(attr.identifier.size()));
        DANNOTATE("identifier::data");
        stream.write(reinterpret_cast<const u8 *>(attr.identifier.data()), attr.identifier.size());
        DANNOTATE("type");
        stream.writeU8(static_cast<u8>(attr.type));
        DANNOTATE("cardinality");
        stream.writeU8(attr.cardinality);
        DANNOTATE("modifiers");
        stream.writeLittle<u16>(attr.modifiers);
    }

    const Vec3i32 min = svo.minIncl();
    const Vec3i32 max = svo.maxExcl();
    const Vec3u32 size = (max - min).cast<u32>();

    DANNOTATE("volume_offset");
    stream.writeLittle<3, i32>(min.data());
    DANNOTATE("volume_size");
    stream.writeLittle<3, u32>(size.data());
    DANNOTATE("volume_empty");
    stream.writeU8(svo.empty());
    DANNOTATE("reserved");
    stream.writeU8('|');

    DANNOTATE("content");
    return stream.good();
}

bool Encoder::writeAttribData(size_t attribIndex, NodeType type)
{
    const bool complete = isComplete(type);
    // Voxels don't store the one additional byte at the front which encodes the child mask.
    // Complete branches also don't store geometry information, but attribute information.
    // We need to skip the first (mask) byte and decrement the size.
    const u8 *begin = attribData.data() + attribIndex + complete;
    const size_t dataLength = encodedAttribSize - isComplete(type);

    if constexpr (ANNOTATE_BINARY) {
        DANNOTATE(std::string(1, annotationCharOf(type)));
        stream.write(begin, dataLength);
        return stream.good();
    }
    else {
        deflate::ResultCode result = deflator.deflate(begin, dataLength);
        if (result != deflate::ResultCode::OK) {
            VXIO_LOG(ERROR, "Deflate error: " + std::string(deflate::errorOf(result)));
            return false;
        }
        return true;
    }
}

bool Encoder::writeNode(node_type &node, SvoNodeType type)
{
    const bool atLeaf = type == SvoNodeType::LEAF;
    const bool complete = attribData[node.value()] == 0;

    // Because we are performing ADFS traversal, we are writing all the child nodes instead of the node itself.
    for (size_t i = 0; i < svo_type::BRANCHING_FACTOR; ++i) {
        if (not node.has(i)) {
            continue;
        }

        size_t childAttribIndex;
        NodeType childType;
        if (atLeaf) {
            auto &leaf = downcast<leaf_type &>(node);
            childAttribIndex = leaf.at(i);
            childType = NodeType::VOXEL;
        }
        else {
            auto &branch = downcast<branch_type &>(node);
            childAttribIndex = branch.child(i)->value();
            childType = complete ? NodeType::COMPLETE : nodeTypeOf(branch.child(i)->type());
        }

        if (not writeAttribData(childAttribIndex, childType)) {
            return false;
        }
    }
    return true;
}

bool Encoder::writeSvo()
{
    if (svo.empty()) {
        return true;
    }

    size_t rootAttribIndex = svo.rootNode().value();
    writeAttribData(rootAttribIndex, NodeType::BRANCH);

    const auto end = svo.depthFirstNodeRange().end();

    for (auto iter = svo.depthFirstNodeRange().begin(); iter != end; ++iter) {
        node_type *node = iter.node();
        if (not writeNode(*node, iter.nodeType())) {
            return false;
        }
    }

    if constexpr (ANNOTATE_BINARY) {
        return stream.good();
    }
    else {
        deflate::ResultCode deflResult = deflator.flush();
        return deflResult == deflate::ResultCode::OK && stream.good();
    }
}

// SVO DECODING ========================================================================================================

ResultCode Decoder::readHeader()
{
#define ENSURE_STREAM_GOOD()         \
    if (not stream.good()) {         \
        state = STATE_FAILED;        \
        return ResultCode::IO_ERROR; \
    }

    u64 magic = stream.readBig<u64>();
    ENSURE_STREAM_GOOD();
    if (magic != MAGIC) {
        return ResultCode::MAGIC_MISMATCH;
    }
    u8 majorAndMinor[2];

    stream.read(majorAndMinor, 2);
    ENSURE_STREAM_GOOD();
    if (majorAndMinor[0] != VERSION_MAJOR || majorAndMinor[1] != VERSION_MINOR) {
        return ResultCode::VERSION_UNSUPPORTED;
    }

    u8 identifierBuffer[256];
    auto *identifierChars = reinterpret_cast<const char *>(identifierBuffer);

    u16 definitionsSize = stream.readLittle<u16>();
    ENSURE_STREAM_GOOD();
    if (definitionsSize == 0) {
        state = STATE_FAILED;
        return ResultCode::HEADER_MISSING_ATTRIBUTES;
    }

    for (u16 i = 0; i < definitionsSize; ++i) {
        u8 identifierLength = stream.read();
        ENSURE_STREAM_GOOD();
        stream.read(identifierBuffer, identifierLength);

        u8 typeRaw = stream.read();
        u8 cardinality = stream.read();
        u16 modifiers = stream.readLittle<u16>();
        ENSURE_STREAM_GOOD();

        AttributeType type{typeRaw};
        if (not isValid(type)) {
            return ResultCode::CORRUPTED_ENUM;
        }

        std::string identifierStr(identifierChars, identifierLength);
        ResultCode defineResult = defineAttribute({identifierStr, type, cardinality, modifiers});
        if (defineResult != ResultCode::OK) {
            return defineResult;
        }
    }

    if (state < STATE_POSITION_ATTRIBUTE_DEFINED) {
        state = STATE_FAILED;
        return ResultCode::HEADER_MISSING_POSITION_ATTRIBUTE;
    }

    stream.readLittle<3, i32>(header.offset.data());
    stream.readLittle<3, u32>(header.size.data());
    u8 empty = stream.read();
    u8 reserved = stream.read();
    ENSURE_STREAM_GOOD();

    if (empty > 1) {
        return ResultCode::CORRUPTED_BOOL;
    }
    header.empty = empty;

    if (reserved != '|') {
        return ResultCode::RESERVED_MISMATCH;
    }

    u32 maxDim = std::max(std::max(header.size.x(), header.size.y()), header.size.z());
    header.svoDepth = log2ceil(maxDim);

    if (header.size.x() == 0 || header.size.y() == 0 || header.size.z() == 0) {
        if (maxDim != 0) {
            return ResultCode::DEGENERATE_DIMENSIONS;
        }
        if (not empty) {
            return ResultCode::EMPTYNESS_CONTRADICTION;
        }
    }

    state = STATE_IO_STARTED;
    return ResultCode::OK;
}

ResultCode Decoder::readVoxels(uint8_t out[], size_t bufferSize, size_t &outVoxelsRead)
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state < STATE_IO_STARTED) return ResultCode::READ_BEFORE_DEFINE;  // we need to read the header before voxels
    if (state == STATE_IO_DONE) {  // everything was already read, we can just exit
        outVoxelsRead = 0;
        return ResultCode::OK;
    }
    if (state == STATE_IO_STARTED) {  // the header was read, but not the root node
        state = STATE_DURING_IO;
        ResultCode result = readRoot();
        if (result != ResultCode::OK) {
            state = STATE_FAILED;
            return result;
        }
    }

    outVoxelsRead = 0;
    size_t voxelsToRead = bufferSize / decodedAttribSize;
    size_t byteOffset = 0;
    bool eof;

    for (; outVoxelsRead < voxelsToRead; ++outVoxelsRead, byteOffset += decodedAttribSize) {
        ResultCode result = goToNextVoxel(eof);
        if (result != ResultCode::OK) {
            return result;
        }
        if (eof) {
            state = STATE_IO_DONE;
            return ResultCode::OK;
        }

        index_t voxelAttrib = topFrame().currAttrib();
        const u8 *begin = attribData.data() + voxelAttrib;
        decodeVoxelData(begin, out + byteOffset);
    }

    return ResultCode::OK;
}

ResultCode Decoder::readRoot() noexcept
{
    VXIO_DEBUG_ASSERT_EQ(attribData.size(), 0);
    VXIO_DEBUG_ASSERT_EQ(depth, 0);
    VXIO_DEBUG_ASSERT_NE(header.svoDepth, 0);

    attribData.resize(encodedAttribSize);

    u8 *const begin = attribData.data();
    usize inflatedBytes;
    deflate::ResultCode inflateResult = inflator.inflate(begin, encodedAttribSize, inflatedBytes);
    if (inflateResult != deflate::ResultCode::OK) {
        return ResultCode::IO_ERROR;
    }
    if (inflatedBytes != encodedAttribSize) {
        VXIO_DEBUG_ASSERT(inflator.eof());
        return ResultCode::EOF_ERROR;
    }

    // the currIndex and nodes members will be initialized to all-zeros, which happens to be what we want in this case
    Frame rootFrame{};
    VXIO_IF_DEBUG(bool pushSuccess =) pushFrame(rootFrame);
    VXIO_DEBUG_ASSERT(pushSuccess);

    return ResultCode::OK;
}

void Decoder::decodeVoxelData(const u8 *in, u8 *out) noexcept
{
    Vec3i32 currPos = position();
    usize writeOffset = encodePosition(currPos, out);
    usize readOffset = 0;

    for (usize i = 0; i < attribDefs.size(); ++i) {
        if (i == positionAttribIndex) {
            continue;
        }
        usize attrSize = attribDefs[i].size();
        std::memcpy(out + writeOffset, in + readOffset, attrSize);
        readOffset += attrSize;
        writeOffset += attrSize;
    }

    VXIO_DEBUG_ASSERT_EQ(writeOffset, decodedAttribSize);
    VXIO_DEBUG_ASSERT_EQ(readOffset, encodedAttribSize - 1);
}

// FRAME STACK MANAGEMENT ==============================================================================================

bool Decoder::pushFrame(Frame frame) noexcept
{
    // uint8_t localIndex = depth == 0 ? 0 : topFrame().currIndex;
    VXIO_DEBUG_ASSERT_LT(frame.currIndex, 8);

    morton <<= 3;
    morton |= frame.currIndex;

    frameStack[depth++] = std::move(frame);

    return depth <= header.svoDepth + 1;
}

bool Decoder::popFrame() noexcept
{
    if (depth == 0) return false;
    index_t baseSize = frameStack[--depth].baseSize;
    attribData.resize(baseSize);
    morton >>= 3;
    return true;
}

bool Decoder::navFrame(u8 index) noexcept
{
    if (depth == 0) return false;
    Frame &top = topFrame();

    VXIO_DEBUG_ASSERT_NE(depth, 0);              // navigation in empty stack
    VXIO_DEBUG_ASSERT_LT(index, 8);              // index out of range
    VXIO_DEBUG_ASSERT_GT(index, top.currIndex);  // navigation in wrong direction or none at all

    top.currIndex = index;
    morton &= ~uint64_t{0b111};
    morton |= index;
    return true;
}

// IMPLICIT OCTREE NAVIGATION ==========================================================================================

Vec3i32 Decoder::position() noexcept
{
    Vec3u32 result;
    dileave3(morton, result.data());
    return result.cast<i32>() + header.offset;
}

ResultCode Decoder::readFrameAttributes(Frame &frame, u8 parentMask) noexcept
{
    VXIO_DEBUG_ASSERT_NE(parentMask, 0);

    const bool noGeometry = frame.complete || depth == header.svoDepth;
    const usize elementSize = encodedAttribSize - noGeometry;

    // It is possible for no bytes to be read, such as when no attribute data is stored for voxels.
    // In this case the contents of the frame are irrelevant, all that matters is the currIndex member in the frame
    // for navigation.
    if (elementSize == 0) {
        return ResultCode::OK;
    }

    const u8 childCount = popCount(parentMask);
    const usize bytesToRead = childCount * elementSize;
    VXIO_DEBUG_ASSERT_NE(childCount, 0);

    attribData.resize(frame.baseSize + bytesToRead);

    u8 *const begin = attribData.data() + frame.baseSize;
    usize inflatedBytes;
    deflate::ResultCode inflateResult = inflator.inflate(begin, bytesToRead, inflatedBytes);
    if (inflateResult != deflate::ResultCode::OK) {
        return ResultCode::IO_ERROR;
    }
    if (inflatedBytes != bytesToRead) {
        VXIO_DEBUG_ASSERT(inflator.eof());
        return ResultCode::EOF_ERROR;
    }

    index_t nodeOffset = 0;
    // could be turned into a do-while loop if zero-masks were handled specially
    for (usize i = frame.currIndex; i < 8; ++i) {
        if (parentMask & (1 << i)) {
            frame.nodes[i] = frame.baseSize + nodeOffset;
            nodeOffset += elementSize;
        }
    }
    VXIO_DEBUG_ASSERT_EQ(nodeOffset, bytesToRead);

    return ResultCode::OK;
}

ResultCode Decoder::goDeeper() noexcept
{
    VXIO_DEBUG_ASSERT_NE(depth, 0);
    if (depth > header.svoDepth) {
        // we are at the level of individual voxels now and can't go any deeper
        return goSideways() ? ResultCode::OK : ResultCode::EMPTY_STACK_POP;
    }
    Frame &top = topFrame();

    VXIO_DEBUG_ASSERT_LT(top.currAttrib(), attribData.size());
    const u8 rawParentMask = top.complete ? 0 : attribData[top.currAttrib()];
    const bool complete = rawParentMask == 0;
    const u8 parentMask = complete ? 0xff : rawParentMask;
    const u8 firstChildIndex = countTrailingZeros(parentMask);

    Frame frame VXIO_IF_DEBUG({});
    frame.complete = complete;
    frame.baseSize = static_cast<index_t>(attribData.size());
    frame.currIndex = firstChildIndex;                        // if mask is all zeros, currIndex will be zero
    VXIO_DEBUG_ASSERT_EQ(frame.baseSize, attribData.size());  // detect inalid type conversion

    ResultCode readResult = readFrameAttributes(frame, parentMask);
    if (readResult != ResultCode::OK) {
        return readResult;
    }

    if (not pushFrame(frame)) {
        return ResultCode::SVO_TOO_DEEP;
    }

    return ResultCode::OK;
}

bool Decoder::goSideways() noexcept
{
    VXIO_DEBUG_ASSERT_NE(depth, 0);

    Frame &top = topFrame();
    u8 topMask = 1;  // default is one for root node
    if (top.complete) {
        topMask = 0xff;
    }
    else if (depth != 1) {
        size_t parentAttribute = parentFrame().currAttrib();
        VXIO_DEBUG_ASSERT_LT(parentAttribute, attribData.size());
        topMask = attribData[parentAttribute];
    }

    VXIO_DEBUG_ASSERT_LT(top.currIndex, 8);
    // We use 2 << X as the start instead of 1 << X to implicitly perform one extra leftshift.
    // This is because we want to test the NEXT mask bit, not the current one.
    // Using index instead would not be safe because this may result in an 8-bit shift. (same as type width)
    u8 index = top.currIndex + 1;
    u8 start = static_cast<u8>(2u << top.currIndex);

    for (u8 testBit = start; testBit != 0; testBit <<= 1, ++index) {
        if (topMask & testBit) {
            return navFrame(index);
        }
    }

    VXIO_DEBUG_ASSERT_EQ(index, 8);
    return finishFrame();
}

ResultCode Decoder::goToNextVoxel(bool &outEof) noexcept
{
    do {
        ResultCode result = goDeeper();
        if (result != ResultCode::OK) {
            return result;
        }
        if (depth == 0) {
            outEof = true;
            VXIO_DEBUG_ASSERT_EQ(attribData.size(), 0);
            return ResultCode::OK;
        }
    } while (depth <= header.svoDepth);

    return ResultCode::OK;
}

bool Decoder::finishFrame() noexcept
{
    if (not popFrame()) {
        return false;
    }
    if (depth == 0) {
        return true;
    }

    return goSideways();
}

}  // namespace mve::flvc
