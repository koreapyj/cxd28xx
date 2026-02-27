#!/bin/sh
# Build and install IFD handler if pcsc-lite headers available
cd "$(dirname "$0")/.." || exit 0
if pkg-config --exists libpcsclite 2>/dev/null; then
    make -C pcsc install || true
fi
