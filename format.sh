#!/bin/bash

SCRIPT_DIR="$(dirname $0)"

function format {
    clang-format-9 -style=file -i "$1"
}

for f in main.cpp svo_nodes.hpp svo.hpp; do
    if [[ -f "$f" && $(basename "$f") != 3rd* ]]; then
        (printf 'Formatting %s\n' "$f"; format "$f") &
    fi
done
