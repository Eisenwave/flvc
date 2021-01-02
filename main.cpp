#include "args.hpp"

#include "bits.hpp"
#include "filetype.hpp"
#include "log.hpp"
#include "stream.hpp"
#include "stringmanip.hpp"
#include "voxelio.hpp"

#include "flvccodec.hpp"

#include "format/binvox.hpp"
#include "format/cubeworld.hpp"
#include "format/qb.hpp"
#include "format/qef.hpp"
#include "format/vl32.hpp"
#include "format/vox.hpp"

#include <iostream>
#include <map>

namespace flvc {

using namespace voxelio;

namespace {

constexpr const char *HEADER = "Fast Lossless Voxel Compression.";
constexpr const char *FOOTER = "--------------------------------";

constexpr const char *INPUT_DESCR =
    "The input file from which voxelio reads. If none is specified, stdin is used instead.";
constexpr const char *OUTPUT_DESCR = "The output file which is written. If none is specified, stdout is used instead.";
constexpr const char *INPUT_FORMAT_DESCR = "The input format. Must be one of "
                                           "binvox, cub, qb, qef, vl32, vox";
constexpr const char *OUTPUT_FORMAT_DESCR = "The output format. Must be one of flvc, qef, vl32";
constexpr const char *LEVEL_DESCR = "The zlib compression level. Must be in range [0, 9]. Zero is no compression.";

}  // namespace

static const std::map<std::string, FileType> FORMAT_MAP{
    {"binvox", FileType::BINVOX},
    {"cub", FileType::CUBEWORLD_CUB},
    {"flvc", FileType::FLVC},
    {"qb", FileType::QUBICLE_BINARY},
    {"qef", FileType::QUBICLE_EXCHANGE},
    {"vl32", FileType::VL32},
    {"vox", FileType::MAGICA_VOX},
};

AbstractReader *makeReader(InputStream &stream, FileType type)
{
    switch (type) {
    case FileType::BINVOX: return new binvox::Reader{stream};
    case FileType::CUBEWORLD_CUB: return new cub::Reader{stream};
    case FileType::QUBICLE_BINARY: return new qb::Reader{stream};
    case FileType::QUBICLE_EXCHANGE: return new qef::Reader{stream};
    case FileType::MAGICA_VOX: return new vox::Reader{stream};
    case FileType::VL32: return new vl32::Reader{stream};
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

AbstractListWriter *makeWriter(OutputStream &stream, FileType type)
{
    switch (type) {
    case FileType::QUBICLE_EXCHANGE: return new qef::Writer{stream};
    case FileType::VL32: return new vl32::Writer{stream};
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

constexpr usize VOXEL_BUFFER_64_SIZE = 8 * 1024;
constexpr usize VOXEL_BUFFER_32_SIZE = VOXEL_BUFFER_64_SIZE * 2;

static Voxel64 VOXEL_BUFFER_64[VOXEL_BUFFER_64_SIZE];
static Voxel32 VOXEL_BUFFER_32[VOXEL_BUFFER_32_SIZE];
static u8 DECODE_BUFFER[256 * 1024];
static const AttributeDef DEF_POSITION = AttributeDef::position(AttributeType::INT_32);
static const AttributeDef DEF_COLOR = AttributeDef{"color", AttributeType::UINT_8, 4};

[[nodiscard]] int convert_flvc_voxelio(FileInputStream &in, FileOutputStream &out, FileType outFormat)
{
    flvc::Decoder decoder{in};
    std::unique_ptr<AbstractListWriter> writer{makeWriter(out, outFormat)};

    VXIO_LOG(INFO, "Decoding FLVC header ...");

    flvc::ResultCode headerCode = decoder.readHeader();
    if (headerCode != ResultCode::OK) {
        VXIO_LOG(ERROR, "Header read error: " + std::string(nameOf(headerCode)));
        return 1;
    }

    ByteArrayInputStream decodeStream{DECODE_BUFFER, sizeof(DECODE_BUFFER)};

    usize voxelIndex = 0;

    VXIO_LOG(INFO, "Decoding FLVC content ...");

    while (not decoder.done()) {
        usize voxelsRead;
        flvc::ResultCode readCode = decoder.readVoxels(DECODE_BUFFER, sizeof(DECODE_BUFFER), voxelsRead);
        if (readCode != ResultCode::OK) {
            VXIO_LOG(ERROR, "Read error: " + std::string(nameOf(readCode)));
            return 1;
        }
        decodeStream.clearErrors();
        decodeStream.seekAbsolute(0);

        for (usize i = 0; i < voxelsRead; ++i, ++voxelIndex) {
            if (voxelIndex == VOXEL_BUFFER_32_SIZE) {
                voxelio::ResultCode writeResult = writer->write(VOXEL_BUFFER_32, voxelIndex);
                if (not voxelio::isGood(writeResult)) {
                    VXIO_LOG(ERROR, "Write error: " + informativeNameOf(writeResult));
                    return 1;
                }
                voxelIndex = 0;
            }

            Voxel32 &voxel = VOXEL_BUFFER_32[voxelIndex];

            decodeStream.readNative<3, i32>(voxel.pos.data());
            decodeStream.read(reinterpret_cast<u8 *>(&voxel.argb), DEF_COLOR.cardinality);
        }
    }
    if (decoder.failed()) {
        VXIO_LOG(ERROR, "Decoder entered failed state!");
        return 1;
    }

    VXIO_LOG(INFO, "Flushing remaining " + stringify(voxelIndex) + " voxels ...");

    // flush any remaining voxels
    voxelio::ResultCode writeResult = writer->write(VOXEL_BUFFER_32, voxelIndex);
    if (not voxelio::isGood(writeResult)) {
        VXIO_LOG(ERROR, "Flush/Write error: " + informativeNameOf(writeResult));
        return 1;
    }

    VXIO_LOG(INFO, "Done!");
    return 0;
}

[[nodiscard]] int convert_voxelio_flvc(FileInputStream &in, FileType inFormat, FileOutputStream &out, unsigned level)
{
    std::unique_ptr<AbstractReader> reader{makeReader(in, inFormat)};
    ReadResult result;
    ByteArrayOutputStream attribStream;

    flvc::Encoder encoder{out, {level}};
    flvc::ResultCode code = encoder.defineAttribute(DEF_POSITION);
    code = encoder.defineAttribute(DEF_COLOR);
    if (code != flvc::ResultCode::OK) {
        VXIO_LOG(ERROR, "Definition error: " + std::string(nameOf(code)));
        return 1;
    }

    VXIO_LOG(INFO, "Reading file and passing voxels to encoder ...");

    do {
        result = reader->read(VOXEL_BUFFER_64, 8192);
        if (result.isBad()) {
            VXIO_LOG(ERROR, "Read error: " + informativeNameOf(result.type));
            return 1;
        }

        attribStream.clear();
        for (usize i = 0; i < result.voxelsRead; ++i) {
            Voxel64 &voxel = VOXEL_BUFFER_64[i];
            Vec3i32 pos32 = voxel.pos.cast<i32>();
            u32 *argb = &voxel.argb;
            u8 *argbBytes = reinterpret_cast<u8 *>(argb);

            attribStream.writeNative<3, i32>(pos32.data());
            attribStream.write(argbBytes, 4);
        }
        flvc::ResultCode insertResult = encoder.insert(attribStream.data(), result.voxelsRead);
        if (insertResult != flvc::ResultCode::OK) {
            VXIO_LOG(ERROR, "Insert error: " + std::string(nameOf(insertResult)));
            return 1;
        }

    } while (not result.isEnd());

    reader.reset();

    VXIO_LOG(INFO, "Optimizing SVO and writing ...");
    ResultCode writeResult = encoder.write();
    if (writeResult != flvc::ResultCode::OK) {
        VXIO_LOG(ERROR, "Write error: " + std::string(nameOf(writeResult)));
        return 1;
    }

    VXIO_LOG(INFO, "Done!");
    return 0;
}

[[nodiscard]] int convert(
    FileInputStream &in, FileType inFormat, FileOutputStream &out, FileType outFormat, unsigned level)
{
    if (inFormat == FileType::FLVC) {
        if (outFormat == FileType::FLVC) {
            VXIO_LOG(ERROR, "flvc -> flvc converting is not supported yet");
        }
        return convert_flvc_voxelio(in, out, outFormat);
    }
    else if (outFormat == FileType::FLVC) {
        return convert_voxelio_flvc(in, inFormat, out, level);
    }
    else {
        VXIO_LOG(ERROR, "voxelio -> voxelio converting is not supported yet");
        return 1;
    }
}

std::optional<FileType> parseFormat(args::ValueFlag<std::string> &flag)
{
    if (not flag.Matched()) {
        return std::nullopt;
    }
    std::string name = flag.Get();
    toLowerCase(name);
    auto iter = FORMAT_MAP.find(name);
    return iter != FORMAT_MAP.end() ? iter->second : std::optional<FileType>{};
}

[[nodiscard]] int main_impl(int argc, const char **argv)
{
    args::ArgumentParser parser(HEADER, FOOTER);

    args::Group group_general(parser, "General Options:");
    args::HelpFlag arg_help(group_general, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<unsigned> arg_comprLevel(group_general, "level", LEVEL_DESCR, {'l', "level"});

    args::Group group_input(parser, "Input file and input format:", args::Group::Validators::AtLeastOne);
    args::ValueFlag<std::string> arg_inputFile(group_input, "file", INPUT_DESCR, {'i', "input"});
    args::ValueFlag<std::string> arg_inputFormat(group_input, "format", INPUT_FORMAT_DESCR, {'I', "input-format"});

    args::Group group_output(parser, "Input file and input format:", args::Group::Validators::DontCare);
    args::ValueFlag<std::string> arg_outputFile(group_output, "file", OUTPUT_DESCR, {'o', "output"});
    args::ValueFlag<std::string> arg_outputFormat(group_output, "format", OUTPUT_FORMAT_DESCR, {'O', "output-format"});

    parser.ParseCLI(argc, argv);

    bool fail = false;
    if (not group_input.Matched()) {
        VXIO_LOG(ERROR, "Either the input file or the input format must be specified!\n\n");
        fail = true;
    }
    else if (not group_output.Matched()) {
        VXIO_LOG(ERROR, "Either the output file or the output format must be specified!\n\n");
        fail = true;
    }
    else if (parser.GetError() != args::Error::None) {
        fail = true;
    }
    if (fail) {
        parser.Help(std::cout);
        return 1;
    }

    std::optional<FileInputStream> in;
    if (arg_inputFile.Matched()) {
        in = openForRead(arg_inputFile.Get(), OpenMode::BINARY);
        if (in.has_value()) {
            VXIO_LOG(INFO, "Reading from \"" + arg_inputFile.Get() + '\"');
        }
        else {
            VXIO_LOG(ERROR, "Can't open \"" + arg_inputFile.Get() + "\" for reading");
            fail = true;
        }
    }
    else {
        in = FileInputStream{stdin};
        VXIO_LOG(INFO, "Reading from stdin");
    }

    std::optional<FileOutputStream> out;
    if (arg_outputFile.Matched()) {
        out = openForWrite(arg_outputFile.Get(), OpenMode::BINARY);
        if (out.has_value()) {
            VXIO_LOG(INFO, "Writing to \"" + arg_outputFile.Get() + '\"');
        }
        else {
            VXIO_LOG(ERROR, "Can't open \"" + arg_inputFile.Get() + "\" for reading");
            fail = true;
        }
    }
    else {
        out = FileOutputStream{stdout};
        VXIO_LOG(INFO, "Writing to stdout");
    }

    std::optional<FileType> inFormat;
    if (arg_inputFormat.Matched()) {
        inFormat = parseFormat(arg_inputFormat);
        if (not inFormat.has_value()) {
            VXIO_LOG(ERROR, "Provided input format \"" + arg_inputFormat.Get() + "\" is invalid");
            fail = true;
        }
    }
    else {
        VXIO_ASSERT(arg_inputFile.Matched());
        inFormat = detectFileType(arg_inputFile.Get());
        if (not inFormat.has_value()) {
            VXIO_LOG(ERROR, "Could not detect input format of \"" + arg_inputFile.Get() + '"');
            fail = true;
        }
    }

    std::optional<FileType> outFormat = parseFormat(arg_outputFormat);
    if (arg_outputFormat.Matched()) {
        outFormat = parseFormat(arg_outputFormat);
        if (not outFormat.has_value()) {
            VXIO_LOG(ERROR, "Provided output format \"" + arg_outputFormat.Get() + "\" is invalid");
            fail = true;
        }
    }
    else {
        VXIO_ASSERT(arg_outputFile.Matched());
        outFormat = detectFileType(arg_outputFile.Get());
        if (not outFormat.has_value()) {
            VXIO_LOG(WARNING,
                     "Could not detect output format of \"" + arg_outputFormat.Get() + "\", defaulting to FLVC");
            outFormat = FileType::FLVC;
        }
    }

    unsigned level = arg_comprLevel.Matched() ? arg_comprLevel.Get() : deflate::Defaults::DEFAULT_LEVEL;
    if (level >= 10) {
        VXIO_LOG(ERROR, "Compression level must be in range [0, 10] (was " + stringify(level) + ")");
        fail = true;
    }
    VXIO_LOG(INFO, "Compressing at level " + stringify(level) + (level == 6 ? " (default)" : std::string()))

    if (fail) {
        std::cout << '\n';
        parser.Help(std::cout);
        return 1;
    }

    VXIO_LOG(INFO, "Converting from " + std::string(nameOf(*inFormat)) + " to " + nameOf(*outFormat));
    return convert(*in, *inFormat, *out, *outFormat, level);
}

}  // namespace flvc

int main(int argc, const char **argv)
{
    return flvc::main_impl(argc, argv);
}
