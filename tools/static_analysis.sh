#!/usr/bin/env bash
# Static analysis of the production code (controller, plant model, firmware).
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

cppcheck --std=c++17 --language=c++ \
  --enable=warning,style,performance,portability \
  --inline-suppr --error-exitcode=1 --quiet \
  --suppress=missingIncludeSystem \
  -I include -I sim/include -I target \
  src sim/src target

echo "cppcheck: no findings."
