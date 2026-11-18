#!/usr/bin/env bash
# Build and verify everything:
#   unit tests -> sanitizers -> SIL scenarios -> firmware -> PIL on QEMU -> static analysis -> plots
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

step() { printf '\n\033[1;34m== %s ==\033[0m\n' "$*"; }

step "Host build + unit tests"
cmake --preset host >/dev/null
cmake --build --preset host
ctest --preset host | tail -3

step "Unit tests with AddressSanitizer + UBSan"
cmake --preset host-asan >/dev/null
cmake --build --preset host-asan
ctest --preset host-asan | tail -3

step "Software-in-the-loop scenarios (host)"
./build/host/aeb_sil --csv out

step "Firmware build (bare-metal Cortex-M3)"
cmake --preset cortex-m3 >/dev/null
cmake --build --preset cortex-m3

step "Processor-in-the-loop on QEMU + SIL/PIL comparison"
./tools/compare_sil_pil.sh

step "Static analysis"
./tools/static_analysis.sh

if [[ -x .venv/bin/python ]]; then
  step "Plots"
  .venv/bin/python tools/plot_scenarios.py out | tail -3
  echo "(all plots in out/)"
fi

printf '\n\033[1;32mAll checks passed.\033[0m\n'
