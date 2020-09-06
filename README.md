# Free Lossless Voxel Compression (FLVC)

This repository contains the source code for the FLVC codec for compression of 3D models as well as a CLI program for converting to/from FLVC.

## Usage

Run the application with no options or with `-h` or `--help` to see a help menu explaining the usage:
```
    Free Lossless Voxel Compression.

  OPTIONS:

      General Options:
        -h, --help                        Display this help menu
        -l[level], --level=[level]        The zlib compression level. Must be in
                                          range [0, 9]. Zero is no compression.
      Input file and input format:
        -i[file], --input=[file]          The input file from which voxelio
                                          reads. If none is specified, stdin is
                                          used instead.
        -I[format],
        --input-format=[format]           The input format. Must be one of
                                          binvox, cub, qb, qef, vl32, vox
      Input file and input format:
        -o[file], --output=[file]         The output file which is written. If
                                          none is specified, stdout is used
                                          instead.
        -O[format],
        --output-format=[format]          The output format. Must be one of
                                          flvc, qef, vl32

```

For example:
```sh
flvc -i mymodel.vox -o mymodel.flvc # formats are automatically detected based on suffixes
```
