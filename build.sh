#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="build"
CLEAN=0

for arg in "$@"; do
    case "$arg" in
        clean|--clean)
            CLEAN=1
            ;;
        *)
            BUILD_DIR="$arg"
            ;;
    esac
done

HOST_ENV=(
    env -i
    HOME="$HOME"
    USER="${USER:-$(id -un)}"
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    CC=/usr/bin/gcc
    CXX=/usr/bin/g++
    AS=/usr/bin/as
)

if [ "$CLEAN" -eq 1 ]; then
    rm -rf "$BUILD_DIR"
fi

"${HOST_ENV[@]}" cmake -S . -B "$BUILD_DIR" \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++

cp test/WMM.COF build/WMM.COF

"${HOST_ENV[@]}" cmake --build "$BUILD_DIR" -j"$(nproc)"