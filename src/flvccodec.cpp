#include "flvc/flvccodec.hpp"

#include "flvc/flvcattrib.hpp"
#include "flvc/flvcconfig.hpp"
#include "flvc/flvcconst.hpp"

#include "voxelio/deflate.hpp"
#include "voxelio/log.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/wileave.hpp"

namespace flvc {

using namespace voxelio;

// *********************************************************************************************************************
// DELTA KERNEL ********************************************************************************************************
// *********************************************************************************************************************

namespace {

/**
 * @brief Performs the actual delta-coding.
 * @param def the attribute definition
 * @param parent a pointer to the attribute data of the parent
 * @param children an array of pointers to the attribute data of the children
 * @param childCount the number of children
 * @param offset a constant offset for both parent and children; useful for vector attributes (cardinality >= 2)
 */
template <typename Int>
void doEncode(const AttributeDef &def, u8 *parent, u8 *children[], const usize childCount, const usize offset)
{
    Int min = ~Int{0};
    // Step 1: compute the minimum of all children
    // we use u64 to guarantee that the type is always representable
    for (usize child = 0; child < childCount; ++child) {
        u8 *childData = children[child] + offset;
        Int value = decodeAttrib<Int>(childData, def.type);
        min = std::min(value, min);
    }
    encodeAttrib<Int>(min, parent + offset, def.type);

    // Step 2: compute offset of children from minimum
    // we use u64 to guarantee that the type is always representable
    for (usize child = 0; child < childCount; ++child) {
        u8 *childData = children[child] + offset;
        Int value = decodeAttrib<Int>(childData, def.type);
        VXIO_DEBUG_ASSERT_GE(value, min);
        Int delta = value - min * (OPTIMIZATION_LEVEL > OptimizationLevel::NO_DELTA_CODING);
        encodeAttrib<Int>(delta, childData, def.type);
    }
}

/**
 * @brief Performs the actual delta-decoding.
 * @param def the attribute definition
 * @param parent a pointer to the attribute data of the parent
 * @param children an array of pointers to the attribute data of the children
 * @param childCount the number of children
 * @param offset a constant offset for both parent and children; useful for vector attributes (cardinality >= 2)
 */
template <typename Int>
void doDecode(const AttributeDef &def, u8 *parent, u8 *children[], const usize childCount, const usize offset)
{
    const Int min = decodeAttrib<Int>(parent + offset, def.type);

    // Decoding is considerably simpler because we only need to compute the offset from the minimum and
    // the minimum is already known in this case.
    for (usize child = 0; child < childCount; ++child) {
        u8 *childData = children[child] + offset;
        Int delta = decodeAttrib<Int>(childData, def.type);
        Int value = min * (OPTIMIZATION_LEVEL > OptimizationLevel::NO_DELTA_CODING) + delta;
        encodeAttrib(value, childData, def.type);
    }
}

}  // namespace

void DeltaKernel::encode(const ExtendedAttributeDef defs[], usize count, usize posAttrib)
{
    VXIO_DEBUG_ASSERT_NE(this->size, 0);
    VXIO_DEBUG_ASSERT_NE(count, 0);
    VXIO_DEBUG_ASSERT_NOTNULL(this->parent);
    VXIO_DEBUG_ASSERT_LT(posAttrib, count);

    for (usize a = 0, offset = 0; a < count; ++a) {
        if (a == posAttrib) {
            continue;
        }
        const AttributeDef &def = defs[a];
        const bool typeIsSigned = isSigned(def.type);

        for (usize k = 0; k < def.cardinality; ++k, offset += def.elementSize()) {
            if (typeIsSigned) {
                doEncode<i64>(def, parent, children, size, offset);
            }
            else {
                doEncode<u64>(def, parent, children, size, offset);
            }
        }
    }
}

void DeltaKernel::decode(const ExtendedAttributeDef defs[], usize count, usize posAttrib)
{
    VXIO_DEBUG_ASSERT_NE(this->size, 0);
    VXIO_DEBUG_ASSERT_NE(count, 0);
    VXIO_DEBUG_ASSERT_NOTNULL(this->parent);
    VXIO_DEBUG_ASSERT_LT(posAttrib, count);

    for (usize a = 0, offset = 0; a < count; ++a) {
        if (a == posAttrib) {
            continue;
        }
        const AttributeDef &def = defs[a];
        const bool typeIsSigned = isSigned(def.type);

        for (usize k = 0; k < def.cardinality; ++k, offset += def.elementSize()) {
            if (typeIsSigned) {
                doDecode<i64>(def, parent, children, size, offset);
            }
            else {
                doDecode<u64>(def, parent, children, size, offset);
            }
        }
    }
}

// *********************************************************************************************************************
// CODEC BASE **********************************************************************************************************
// *********************************************************************************************************************

// ATTRIBUTE MANAGEMENT ================================================================================================

ResultCode CodecBase::defineAttribute(AttributeDef def) noexcept
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state >= STATE_VOXELS_INSERTED)
        return state == STATE_VOXELS_INSERTED ? ResultCode::DEFINE_AFTER_INSERT : ResultCode::DEFINE_AFTER_WRITE;

    if (def.identifier.length() > 255) {
        return ResultCode::IDENTIFIER_TOO_LONG;
    }

    usize inputOffset = attribDefs.empty() ? 0 : attribDefs.back().inputOffset + attribDefs.back().size();

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

usize CodecBase::allocAttribute() noexcept
{
    usize allocSize = mode == Mode::ENCODE ? encodedAttribSize : decodedAttribSize;
    VXIO_DEBUG_ASSERT_NE(allocSize,
                         0);  // encoded will include at least one byte, decoded at least the bytes of positions
    usize result = attribData.size();
    attribData.resize(attribData.size() + allocSize);
    return result;
}

usize CodecBase::insertAttribute(const u8 data[]) noexcept
{
    VXIO_ASSERT_GE(state, STATE_POSITION_ATTRIBUTE_DEFINED);
    const ExtendedAttributeDef &posDef = attribDefs[positionAttribIndex];

    usize result = attribData.size();
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

// PERMUTATION GENERATION ==============================================================================================

void CodecBase::genPermutations() noexcept
{
    for (usize i = 0; i < 8; ++i) {
        genPermutations(i + 1);
    }
}

void CodecBase::genPermutations(const usize nodeCount) noexcept
{
    VXIO_DEBUG_ASSERT_NE(nodeCount, 0);
    VXIO_DEBUG_ASSERT_LE(nodeCount, 8);

    const usize index = nodeCount - 1;

    if (OPTIMIZATION_LEVEL <= OptimizationLevel::NO_ATTRIBUTE_DILEAVING || nodeCount == 1) {
        dileavePermsWithInternal[index] = Permutation::identity(encodedAttribSize * nodeCount);
        dileavePermsNoInternal[index] = Permutation::identity((encodedAttribSize - 1) * nodeCount);

        ileavePermsWithInternal[index] = dileavePermsWithInternal[index];
        ileavePermsNoInternal[index] = dileavePermsNoInternal[index];
    }
    else {
        flvc::Permutation perm = genDeinterleavingPermutation(nodeCount);

        dileavePermsNoInternal[index] = perm.disPerm(nodeCount);
        dileavePermsWithInternal[index] = std::move(perm);

        ileavePermsNoInternal[index] = dileavePermsNoInternal[index].inverse();
        ileavePermsWithInternal[index] = dileavePermsWithInternal[index].inverse();
    }
}

flvc::Permutation CodecBase::genDeinterleavingPermutation(const usize nodeCount) noexcept
{
    // There is always one additional implicitly defined mask attribute
    // We don't expose it in the attribute definitions, but must take it into account when interleaving.
    constexpr usize internalAttribSize = 1;
    constexpr usize internalAttribCardinality = 1;

    const usize defCountIncludingInternal = attribDefs.size() + 1;

    std::vector<usize> table(encodedAttribSize * nodeCount);

    usize byteIndex = 0;
    usize nodeIndex = 0;
    usize elemIndex = 0;
    usize defnIndex = 0;

    usize offset = 0;

    usize currElementSize = internalAttribSize;
    usize currCardinality = internalAttribCardinality;

    VXIO_DEBUG_ASSERT_GE(table.size(), 2);
    for (usize i = 0;;) {
        table[i] = offset;

        if (++i == table.size()) {
            break;
        }

        // 1. Byte of Element

        offset += 1;
        if (++byteIndex != currElementSize) {
            continue;
        }
        offset -= 1 * byteIndex;
        byteIndex = 0;

        // 2. Node Index

        offset += encodedAttribSize;
        if (++nodeIndex != nodeCount) {
            continue;
        }
        offset -= encodedAttribSize * nodeIndex;
        nodeIndex = 0;

        // 3. Vector Index of Definition

        offset += currElementSize;
        if (++elemIndex != currCardinality) {
            continue;
        }
        elemIndex = 0;

        // 4. Definition Index
        // We don't need to decrease and then increase the offset between these steps.
        // This is because the element is a sub-property of the definition.
        // The same can't be said for the byte index, which is not a sub-property of the node index.

        if (defnIndex++ == positionAttribIndex) {
            ++defnIndex;
        }
        if (defnIndex != defCountIncludingInternal) {
            const auto &def = attribDefs[defnIndex - 1];
            currElementSize = def.elementSize();
            currCardinality = def.cardinality;
            // currDefinitionSize = currElementSize * currCardinality;
            continue;
        }
        VXIO_DEBUG_ASSERT_UNREACHABLE();
    }

    return flvc::Permutation{table};
}

// POSITION ENCODING/DECODING ==========================================================================================

Vec3i32 CodecBase::decodePosition(const u8 data[]) noexcept
{
    const ExtendedAttributeDef &posDef = positionDefinition();

    const usize memberOffset = posDef.elementSize();
    const u8 *posDataBegin = data + posDef.inputOffset;

    i32 x = decodeAttrib<i32>(posDataBegin + memberOffset * 0, posDef.type);
    i32 y = decodeAttrib<i32>(posDataBegin + memberOffset * 1, posDef.type);
    i32 z = decodeAttrib<i32>(posDataBegin + memberOffset * 2, posDef.type);

    return {x, y, z};
}

usize CodecBase::encodePosition(Vec3i32 pos, u8 out[]) noexcept
{
    const ExtendedAttributeDef &posDef = positionDefinition();
    VXIO_DEBUG_ASSERT_EQ(posDef.cardinality, 3);

    const usize memberOffset = posDef.elementSize();

    encodeAttrib<i32>(pos.x(), out + memberOffset * 0, posDef.type);
    encodeAttrib<i32>(pos.y(), out + memberOffset * 1, posDef.type);
    encodeAttrib<i32>(pos.z(), out + memberOffset * 2, posDef.type);

    return posDef.size();
}

// *********************************************************************************************************************
// ENCODER *************************************************************************************************************
// *********************************************************************************************************************

// VOXEL DATA TO SVO INSERTION =========================================================================================

ResultCode Encoder::insert(const u8 data[], usize voxelCount)
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state < STATE_POSITION_ATTRIBUTE_DEFINED) return ResultCode::INSERT_BEFORE_DEFINE;
    if (state > STATE_VOXELS_INSERTED) return ResultCode::INSERT_AFTER_WRITE;

    if (state == STATE_POSITION_ATTRIBUTE_DEFINED) {
        // Upon the first insert() call, we compute the (de)interleaving permutations for our attributes and allocate
        // a small amount of bytes at the beginning of attribData.
        // This will be used as a temporary buffer for our de-interleaving operations.
        VXIO_ASSERT_EQ(attribData.size(), 0);
        usize deinterleavingBufferSize = encodedAttribSize * 8 * 2;
        attribData.resize(deinterleavingBufferSize);
        state = STATE_VOXELS_INSERTED;
    }

    // TODO optimize specially for geometry-only
    for (usize i = 0; i < voxelCount; ++i) {
        Vec3i32 pos = decodePosition(data);
        usize attribIndex = insertAttribute(data);
        svo.insert(pos, attribIndex);

        data += decodedAttribSize;
    }

    return ResultCode::OK;
}

// SVO OPTIMIZATION ====================================================================================================

void Encoder::optimizeSvo()
{
    optimizeSvo_initAttribDataForOneNode(svo.rootNode(), SvoNodeType::BRANCH);

    expandNodes(svo, [this](node_type *children[], usize count, branch_type &parent) -> void {
        optimizeSvo_initAttribData(children, count, parent);
    });

    // hint to the compiler that this stays constant for all invocations to optimize last-second decisions
    const usize encodedAttribSize = this->encodedAttribSize;

    reduceNodes(svo, [this, encodedAttribSize](node_type *children[], usize count, branch_type &parent) -> void {
        optimizeSvo_reduceCompleteMasks(children, count, parent);
        if (encodedAttribSize != 1) {
            optimizeSvo_computeAttributeDeltas(children, count, parent);
        }
    });

    // retroactively fix the root node (commented out because not necessary for now)
    /* usize rootAttribIndex = svo.rootNode().value();
    attribData[rootAttribIndex] = static_cast<u8>(svo.rootNode().mask().to_ulong()); */
}

void Encoder::optimizeSvo_initAttribData(node_type *children[], usize count, branch_type &)
{
    // create attribute data for all nodes, initialize the first byte to the mask
    // for leafs where the node is full, initialize to 0

    VXIO_DEBUG_ASSERT_NE(count, 0);
    const SvoNodeType commonChildType = children[0]->type();

    for (usize i = 0; i < count; ++i) {
        node_type *node = children[i];
        VXIO_DEBUG_ASSERT_NOTNULL(node);
        optimizeSvo_initAttribDataForOneNode(*node, commonChildType);
    }
}

void Encoder::optimizeSvo_initAttribDataForOneNode(node_type &node, SvoNodeType type)
{
    VXIO_DEBUG_ASSERT(not node.empty());
    usize resultIndex = allocAttribute();
    VXIO_DEBUG_ASSERT_LT(resultIndex, attribData.size());

    attribData[resultIndex] = type == SvoNodeType::LEAF && node.full() ? 0 : node.mask();
    node.value = resultIndex;
}

void Encoder::optimizeSvo_reduceCompleteMasks(node_type *children[], usize count, branch_type &parent)
{
    // for complete nodes, mark the parent as complete as well by setting the parent's byte to 0
    // this happens recursively
    VXIO_DEBUG_ASSERT_NE(count, 0);

    if (not parent.full()) {
        return;
    }
    u8 combinedChildMask = 0;
    for (usize i = 0; i < count; ++i) {
        usize childAttribIndex = children[i]->value;
        VXIO_DEBUG_ASSERT_LT(childAttribIndex, attribData.size());
        combinedChildMask |= attribData[childAttribIndex];
    }
    if (combinedChildMask == 0) {
        usize parentAttribIndex = parent.value;
        VXIO_DEBUG_ASSERT_LT(parentAttribIndex, attribData.size());
        attribData[parentAttribIndex] = 0;
    }
}

void Encoder::optimizeSvo_computeAttributeDeltas(node_type *children[], usize count, branch_type &parent)
{
    VXIO_DEBUG_ASSERT_NE(count, 0);
    const SvoNodeType commonChildType = children[0]->type();
    u8 *const begin = attribData.data();

    if (commonChildType == SvoNodeType::LEAF) {
        for (usize i = 0; i < count; ++i) {
            node_type &child = *children[i];
            leaf_type &leaf = downcast<leaf_type &>(child);

            deltaKernel.reset(begin + leaf.value + 1);

            for (usize j = 0; j < svo_type::BRANCHING_FACTOR; ++j) {
                if (not child.has(j)) {
                    continue;
                }
                usize voxelAttribOffset = leaf.at(j) + 1;
                deltaKernel.pushChild(begin + voxelAttribOffset);
            }

            deltaKernel.encode(attribDefs.data(), attribDefs.size(), positionAttribIndex);
        }
    }

    usize parentAttribOffset = parent.value + 1;
    deltaKernel.reset(attribData.data() + parentAttribOffset);

    for (usize i = 0; i < count; ++i) {
        node_type &child = *children[i];

        usize childAttribOffset = child.value + 1;
        deltaKernel.pushChild(attribData.data() + childAttribOffset);
    }

    deltaKernel.encode(attribDefs.data(), attribDefs.size(), positionAttribIndex);
}

// TOP-LEVEL AND HEADER WRITING ========================================================================================

static constexpr u64 MAGIC = 0xff11'33cc'666c'7663;
static constexpr u8 VERSION_MAJOR = 0;
static constexpr u8 VERSION_MINOR = 1;

ResultCode Encoder::write()
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state == STATE_IO_DONE) return ResultCode::OK;

    genPermutations();
    optimizeSvo();

    if (state < STATE_IO_STARTED) {
        if (not writeHeader()) {
            return ResultCode::IO_ERROR;
        }
    }
    if (state == STATE_IO_STARTED) {
        if (not writeSvo() || stream.err()) {
            state = STATE_FAILED;
            return ResultCode::IO_ERROR;
        }
        state = STATE_IO_DONE;
    }

    return ResultCode::OK;
}

#define FLVC_ANNOTATE(str) \
    if constexpr (ANNOTATE_BINARY) stream.writeString("(" + std::string(str) + ")")

bool Encoder::writeHeader()
{
    constexpr u8 version[]{VERSION_MAJOR, VERSION_MINOR};
    constexpr u8 pad[6] = "(def)";

    stream.writeBig(MAGIC);
    FLVC_ANNOTATE("version");
    stream.write(version, 2);

    const Vec3i32 min = svo.minIncl();
    const Vec3i32 max = svo.maxExcl();
    const Vec3u32 size = (max - min).cast<u32>();
    const bool empty = svo.empty();

    FLVC_ANNOTATE("volume_info::offset");
    stream.writeLittle<3, i32>(min.data());
    FLVC_ANNOTATE("volume_info::size");
    stream.writeLittle<3, u32>(size.data());
    FLVC_ANNOTATE("volume_info::empty");
    stream.writeU8(empty);

    // This padding makes the size of the constant-sized part of the header 40 (multiple of 8).
    // As a result we can examine this part as an array of 64-bit integers if we want to.
    FLVC_ANNOTATE("padding");
    stream.write(pad, 5);

    if constexpr (not ANNOTATE_BINARY) {
        VXIO_DEBUG_ASSERT_EQ(stream.position(), 40);
    }

    FLVC_ANNOTATE("definitions_size");
    stream.writeLittle(static_cast<u16>(attribDefs.size()));
    FLVC_ANNOTATE("definitions");
    for (const AttributeDef &attr : attribDefs) {
        FLVC_ANNOTATE("identifier::length");
        stream.writeU8(static_cast<u8>(attr.identifier.size()));
        FLVC_ANNOTATE("identifier::data");
        stream.write(reinterpret_cast<const u8 *>(attr.identifier.data()), attr.identifier.size());
        FLVC_ANNOTATE("type");
        stream.writeU8(static_cast<u8>(attr.type));
        FLVC_ANNOTATE("cardinality");
        stream.writeU8(attr.cardinality);
        FLVC_ANNOTATE("modifiers");
        stream.writeLittle<u16>(attr.modifiers);
    }

    FLVC_ANNOTATE("reserved");
    stream.writeU8('|');

    FLVC_ANNOTATE("content");
    const bool good = stream.good();

    state = good ? (empty ? STATE_IO_DONE : STATE_IO_STARTED) : STATE_FAILED;
    return good;
}

// SVO WRITING =========================================================================================================

bool Encoder::writeSvo()
{
    VXIO_ASSERT_NE(state, STATE_IO_DONE);

    usize rootAttribIndex = svo.rootNode().value;
    writeAttribData(rootAttribIndex, NodeType::BRANCH);

    bool writeSuccess = true;
    svo.forEachNodeTopDown([this, &writeSuccess](node_type *node, SvoNodeType type) -> void {
        writeSuccess &= writeSuccess && writeNode(*node, type);
    });

    if constexpr (OPTIMIZATION_LEVEL <= OptimizationLevel::NO_COMPRESSION) {
        return stream.good();
    }
    else {
        deflate::ResultCode deflResult = deflator.flush();
        return deflResult == deflate::ResultCode::OK && stream.good();
    }
}

bool Encoder::writeNode(node_type &node, SvoNodeType type)
{
    const usize firstIndex = node.firstIndex();
    VXIO_DEBUG_ASSERT_LT(firstIndex, 8);

    const bool atLeaf = type == SvoNodeType::LEAF;
    const bool complete = attribData[node.value] == 0;
    const bool noGeometry = complete || atLeaf;
    const usize childCount = node.count();

    // 1. COPY NODES INTO DE-INTERLEAVING BUFFER

    u8 *const begin = attribData.data();

    // Because we are performing ADFS traversal, we are writing all the child nodes instead of the node itself.
    // This method is somewhat complex, because the attribute data of the we want to write is scattered.
    // To solve this, we collect all the data at the start of our attribData buffer.

    /** The offset from the start of attribData after interleaving. This always includes the internal mask byte. */
    usize offset = 0;
    for (usize i = node.firstIndex(); i < svo_type::BRANCHING_FACTOR; ++i) {
        if (not node.has(i)) {
            continue;
        }

        usize childAttribIndex =
            atLeaf ? downcast<leaf_type &>(node).at(i) : downcast<branch_type &>(node).child(i)->value;

        std::memcpy(begin + offset, begin + childAttribIndex, encodedAttribSize);
        offset += encodedAttribSize;
    }
    VXIO_DEBUG_ASSERT_EQ(offset, childCount * encodedAttribSize);

    // 2. PERFORM DE-INTERLEAVING, BIT-INTERLEAVING

    flvc::Permutation &perm = dileavePermsWithInternal[childCount - 1];
    VXIO_DEBUG_ASSERT_EQ(perm.size(), offset);

    // the result of this permutation still includes the internal mask byte, which we will discard during write
    perm.apply(begin + offset, begin);

    const usize writeOffset = offset + (noGeometry * childCount);
    const usize writeSize = offset - (noGeometry * childCount);
    const u8 *writeBegin = begin + writeOffset;

    // always skip the internal mask byte for all children, which makes up the first N bytes
    usize ileaveOffset = offset + childCount;

    for (usize def = 0; def < attribDefs.size(); ++def) {
        if (def == positionAttribIndex) {
            continue;
        }
        const AttributeDef &attrDef = attribDefs[def];

        for (usize elem = 0; elem < attrDef.cardinality; ++elem) {
            u8 *inout = begin + ileaveOffset;
            if constexpr (OPTIMIZATION_LEVEL > OptimizationLevel::NO_BIT_INTERLEAVING) {
                ileaveAttrib(inout, inout, childCount, attrDef.type);
            }
            ileaveOffset += childCount * attrDef.elementSize();
        }
    }
    VXIO_DEBUG_ASSERT_EQ(ileaveOffset, offset + childCount * encodedAttribSize);

    // 3. WRITE TO STREAM / INFLATOR

    if constexpr (OPTIMIZATION_LEVEL <= OptimizationLevel::NO_COMPRESSION) {
        const NodeType type =
            atLeaf ? NodeType::VOXEL : complete ? NodeType::COMPLETE : nodeTypeOf(downcast<branch_type &>(node).type());
        const char annotationChar = annotationCharOf(type);

        FLVC_ANNOTATE(annotationChar + std::string{"*"} + stringify(childCount));
        stream.write(writeBegin, writeSize);
        return stream.good();
    }
    else {
        deflate::ResultCode result = deflator.deflate(writeBegin, writeSize);
        if (result != deflate::ResultCode::OK) {
            VXIO_LOG(ERROR, "Deflate error: " + std::string(deflate::errorOf(result)));
            return false;
        }
        return true;
    }
}

bool Encoder::writeAttribData(usize attribIndex, NodeType type)
{
    const bool complete = isComplete(type);
    // Voxels don't store the one additional byte at the front which encodes the child mask.
    // Complete branches also don't store geometry information, but attribute information.
    // We need to skip the first (mask) byte and decrement the size.
    const u8 *begin = attribData.data() + attribIndex + complete;
    const usize dataLength = encodedAttribSize - isComplete(type);

    if constexpr (OPTIMIZATION_LEVEL <= OptimizationLevel::NO_COMPRESSION) {
        FLVC_ANNOTATE(std::string(1, annotationCharOf(type)));
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

// *********************************************************************************************************************
// DECODER *************************************************************************************************************
// *********************************************************************************************************************

// HEADER DECODING =====================================================================================================

ResultCode Decoder::readHeader()
{
#define ENSURE_STREAM_GOOD()         \
    if (not stream.good()) {         \
        state = STATE_FAILED;        \
        return ResultCode::IO_ERROR; \
    }

    ResultCode constantsResult = readHeader_constants();
    if (constantsResult != ResultCode::OK) {
        state = STATE_FAILED;
        return constantsResult;
    }

    stream.readLittle<3, i32>(header.offset.data());
    stream.readLittle<3, u32>(header.size.data());
    u8 empty = stream.read();
    stream.seekRelative(5);  // five undefined padding bytes
    ENSURE_STREAM_GOOD();

    if (empty > 1) {
        return ResultCode::CORRUPTED_BOOL;
    }
    header.empty = empty;

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

    ResultCode attribDefResult = readHeader_attributeDefinitions();
    if (attribDefResult != ResultCode::OK) {
        state = STATE_FAILED;
        return attribDefResult;
    }

    u8 reserved = stream.read();
    ENSURE_STREAM_GOOD();
    if (reserved != '|') {
        state = STATE_FAILED;
        return ResultCode::RESERVED_MISMATCH;
    }

    state = STATE_IO_STARTED;
    return attribDefResult;
}

ResultCode Decoder::readHeader_constants() noexcept
{
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

    return ResultCode::OK;
}

ResultCode Decoder::readHeader_attributeDefinitions() noexcept
{
    u8 identifierBuffer[256];
    auto *identifierChars = reinterpret_cast<const char *>(identifierBuffer);

    u16 definitionsSize = stream.readLittle<u16>();
    ENSURE_STREAM_GOOD();
    if (definitionsSize == 0) {
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
        return ResultCode::HEADER_MISSING_POSITION_ATTRIBUTE;
    }

    return ResultCode::OK;
}

// CONTENT DECODING ====================================================================================================

ResultCode Decoder::readVoxels(u8 out[], usize bufferSize, usize &outVoxelsRead)
{
    if (state == STATE_FAILED) return ResultCode::USE_AFTER_FAIL;
    if (state < STATE_IO_STARTED) return ResultCode::READ_BEFORE_DEFINE;  // we need to read the header before voxels
    if (state == STATE_IO_DONE) {  // everything was already read, we can just exit
        outVoxelsRead = 0;
        return ResultCode::OK;
    }
    if (state == STATE_IO_STARTED) {  // the header was read, but not the root node
        state = STATE_DURING_IO;

        VXIO_DEBUG_ASSERT_EQ(attribData.size(), 0);
        const usize interleavingBufferSize = encodedAttribSize * 8u;
        attribData.resize(interleavingBufferSize);
        VXIO_ASSERT_EQ(interleavingBufferSize, attribData.size());

        genPermutations();

        ResultCode result = readRoot();
        if (result != ResultCode::OK) {
            state = STATE_FAILED;
            return result;
        }
    }

    outVoxelsRead = 0;
    usize voxelsToRead = bufferSize / decodedAttribSize;
    usize byteOffset = 0;
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

ResultCode Decoder::readRoot() noexcept
{
    VXIO_DEBUG_ASSERT_EQ(depth, 0);
    VXIO_DEBUG_ASSERT_NE(header.svoDepth, 0);

    index_t baseSize = static_cast<index_t>(attribData.size());
    attribData.resize(baseSize + encodedAttribSize);

    u8 *const begin = attribData.data() + baseSize;
    usize inflatedBytes;
    deflate::ResultCode inflateResult = inflator.inflate(begin, encodedAttribSize, inflatedBytes);
    if (inflateResult != deflate::ResultCode::OK) {
        return ResultCode::IO_ERROR;
    }
    if (inflatedBytes != encodedAttribSize) {
        VXIO_DEBUG_ASSERT(inflator.eof());
        return ResultCode::EOF_ERROR;
    }

    Frame rootFrame{};
    rootFrame.currIndex = 0;
    rootFrame.baseSize = baseSize;
    rootFrame.nodes[0] = baseSize;

    VXIO_IF_DEBUG(bool pushSuccess =) pushFrame(rootFrame);
    VXIO_DEBUG_ASSERT(pushSuccess);

    return ResultCode::OK;
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

    if (build::DEBUG || encodedAttribSize != 1) {
        deoptimizeFrame(frame, parentMask, childCount, noGeometry);
    }

    return ResultCode::OK;
}

// FRAME DEOPTIMIZATION ================================================================================================

void Decoder::deoptimizeFrame(Frame &frame, const u8 parentMask, const u8 childCount, const bool noGeometry) noexcept
{
    deoptimizeFrame_bitDeinterleave(frame, childCount, noGeometry);
    deoptimizeFrame_byteInterleave(frame, childCount, noGeometry);
    deoptimizeFrame_decodeDeltas(frame, parentMask, noGeometry);
}

void Decoder::deoptimizeFrame_bitDeinterleave(Frame &frame, const u8 childCount, const bool noGeometry) noexcept
{
    // This operation can take place directly in the frame, because bit-deinterleaving support in-place operations.

    VXIO_DEBUG_ASSERT_NE(childCount, 0);
    VXIO_DEBUG_ASSERT_LE(childCount, 8);

    u8 *const begin = attribData.data();
    usize ileaveOffset = frame.currAttrib() + childCount * not noGeometry;

    for (usize def = 0; def < attribDefs.size(); ++def) {
        if (def == positionAttribIndex) {
            continue;
        }
        const AttributeDef &attrDef = attribDefs[def];

        for (usize elem = 0; elem < attrDef.cardinality; ++elem) {
            u8 *inout = begin + ileaveOffset;
            if constexpr (OPTIMIZATION_LEVEL > OptimizationLevel::NO_BIT_INTERLEAVING) {
                dileaveAttrib(inout, inout, childCount, attrDef.type);
            }
            ileaveOffset += childCount * attrDef.elementSize();
        }
    }
}

void Decoder::deoptimizeFrame_byteInterleave(Frame &frame, const u8 childCount, const bool noGeometry) noexcept
{
    // When decoding, the data of the frame is always stored contiguously, so we can perform the interleaving at once
    // Before allocating our root attributes, we have also allocated a small amount of bytes to serve as an
    // interleaving buffer.

    VXIO_DEBUG_ASSERT_NE(childCount, 0);
    VXIO_DEBUG_ASSERT_LE(childCount, 8);

    Permutation &perm = (noGeometry ? ileavePermsNoInternal : ileavePermsWithInternal)[childCount - 1];
    index_t frameIndex = frame.currAttrib();
    u8 *begin = attribData.data();

    const usize expectedPermSize = childCount * (encodedAttribSize - noGeometry);
    VXIO_DEBUG_ASSERT_EQ(perm.size(), expectedPermSize);

    // We interleave from our frame into this buffer and then memcpy back.
    // This approach is significantly faster than permuting the frame in-place.

    // copy the frame data into our permutation buffer at the start while performing the permutation
    perm.apply(begin, begin + frameIndex);
    // copy the permuted data back into the frame
    std::memcpy(begin + frameIndex, begin, expectedPermSize);
}

void Decoder::deoptimizeFrame_decodeDeltas(Frame &frame, const u8 parentMask, const bool noGeometry) noexcept
{
    Frame &parent = topFrame();
    usize parentAttrib = parent.currAttrib() + not parent.complete;
    deltaKernel.reset(attribData.data() + parentAttrib);

    VXIO_DEBUG_ASSERT_LT(frame.currIndex, 8);
    const u8 start = static_cast<u8>(1u << frame.currIndex);
    for (u8 index = frame.currIndex, b = start; b != 0; ++index, b <<= 1) {
        if (parentMask & b) {
            index_t offset = frame.nodes[index] + not noGeometry;
            deltaKernel.pushChild(attribData.data() + offset);
        }
    }

    deltaKernel.decode(attribDefs.data(), attribDefs.size(), positionAttribIndex);
    VXIO_DEBUG_ASSERT_EQ(deltaKernel.size, popCount(parentMask));
}

// FRAME STACK MANAGEMENT ==============================================================================================

bool Decoder::pushFrame(Frame frame) noexcept
{
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
    morton &= ~u64{0b111};
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
        usize parentAttribute = parentFrame().currAttrib();
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
            usize interleavingBufferSize = encodedAttribSize * 8;
            VXIO_ASSERT_EQ(attribData.size(), interleavingBufferSize);
            attribData.clear();
            return ResultCode::OK;
        }
    } while (depth <= header.svoDepth);

    outEof = false;
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

}  // namespace flvc
