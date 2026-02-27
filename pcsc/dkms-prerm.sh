#!/bin/sh
cd "$(dirname "$0")/.." || exit 0
make -C pcsc uninstall 2>/dev/null || true
