#include "args.hpp"

#include "bits.hpp"
#include "filetype.hpp"
#include "stream.hpp"
#include "log.hpp"
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
constexpr const char *FOOTER = "Sample Text";

constexpr const char *INPUT_DESCR =
    "The input file from which voxelio reads. If none is specified, stdin is used instead.";
constexpr const char *OUTPUT_DESCR =
    "The output file which is written. Should end with a .flvc suffix. If none is specified, stdout is used instead.";
constexpr const char* INPUT_FORMAT_DESCR = "The input format. Must be one of "
                                           "binvox, cub, qb, qef, vl32, vox";
constexpr const char* OUTPUT_FORMAT_DESCR = "The output format. Currently only flvc is supported.";

}  // namespace

static const std::map<std::string, FileType> FORMAT_MAP {
    {"binvox", FileType::BINVOX},
    {"cub", FileType::CUBEWORLD_CUB},
    {"flvc", FileType::FLVC},
    {"qb", FileType::QUBICLE_BINARY},
     {"qef", FileType::QUBICLE_EXCHANGE},
     {"vl32", FileType::VL32},
     {"vox", FileType::VL32},
};

[[noreturn]] void convertFlvc(FileInputStream &, FileOutputStream &)
{
    // Not implemented yet.
    VXIO_ASSERT_UNREACHABLE();
}

AbstractReader* makeReader(InputStream &stream, FileType type)
{
    switch (type) {
    case FileType::BINVOX: return new binvox::Reader{stream};
    case FileType::CUBEWORLD_CUB: return new cub::Reader{stream};
    case FileType::QUBICLE_BINARY: return new qb::Reader{stream};
    case FileType::QUBICLE_EXCHANGE: return new qef::Reader{stream};
    case FileType::MAGICA_VOX: return new vox::Reader{stream};
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

void convert(FileInputStream &in, FileType inFormat, FileOutputStream &out, FileType outFormat) {
    VXIO_ASSERT_EQ(inFormat, outFormat);

    if (inFormat == FileType::FLVC) {
        convertFlvc(in, out);
    }

    std::unique_ptr<AbstractReader> reader{makeReader(in, inFormat)};


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

int main(int argc, const char **argv)
{
    args::ArgumentParser parser(HEADER, FOOTER);

    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});

    args::Group inputGroup(parser, "Input file and input format:", args::Group::Validators::AtLeastOne);
    args::ValueFlag<std::string> inputFile(inputGroup, "input file", INPUT_DESCR, {'i', "input"});
    args::ValueFlag<std::string> inputFormat(inputGroup, "input format", INPUT_FORMAT_DESCR, {'I', "input-format"});

    args::Group outputGroup(parser, "Input file and input format:", args::Group::Validators::DontCare);
    args::ValueFlag<std::string> outputFile(outputGroup, "output file", OUTPUT_DESCR, {'o', "output"});
    args::ValueFlag<std::string> outputFormat(
        outputGroup, "output format", OUTPUT_FORMAT_DESCR, {'O', "output-format"});

    parser.ParseCLI(argc, argv);

    bool fail = false;
    if (not inputGroup.Matched()) {
        VXIO_LOG(ERROR, "Either the input file or the input format must be specified!\n\n");
        fail = true;
    }
    else if (not outputGroup.Matched()) {
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
    if (inputFile.Matched()) {
        in = openForRead(inputFile.Get(), OpenMode::BINARY);
        VXIO_LOG(INFO, "Reading from \"" + inputFile.Get() + '\"');
    }
    else {
        in = FileInputStream{stdin};
        VXIO_LOG(INFO, "Reading from stdin");
    }

    std::optional<FileOutputStream> out;
    if (outputFile.Matched()) {
        out = openForWrite(outputFile.Get(), OpenMode::BINARY);
        VXIO_LOG(INFO, "Writing to \"" + outputFile.Get() + '\"');
    }
    else {
        out = FileOutputStream{stdout};
        VXIO_LOG(INFO, "Writing to stdout");
    }

    std::optional<FileType> inFormat;
    if (inputFormat.Matched()) {
        inFormat = parseFormat(inputFormat);
        if (not inFormat.has_value()) {
            VXIO_LOG(ERROR, "Provided input format \"" + inputFormat.Get() + "\" is invalid");
            fail = true;
        }
    }
    else {
        VXIO_ASSERT(inputFile.Matched());
        inFormat = detectFileType(inputFile.Get());
        if (not inFormat.has_value()) {
            VXIO_LOG(ERROR, "Could not detect format of \"" + inputFormat.Get() + "\" is invalid");
            fail = true;
        }
    }

    std::optional<FileType> outFormat = parseFormat(outputFormat);
    if (outputFormat.Matched()) {
        outFormat = parseFormat(outputFormat);
        if (outFormat != FileType::FLVC) {
            VXIO_LOG(ERROR, "Provided output format \"" + outputFormat.Get() + "\" is invalid (must be flvc)");
            fail = true;
        }
    }
    else {
        outFormat = FileType::FLVC;
    }

    if (fail) {
        std::cout << '\n';
        parser.Help(std::cout);
        return 1;
    }

    convert(*in, *inFormat, *out, *outFormat);

    return 0;
}

}
