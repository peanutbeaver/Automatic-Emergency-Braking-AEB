#!/usr/bin/env bash
# Runs every scenario on the host (SIL) and on the emulated Cortex-M3 (PIL)
# and checks that both produce identical results (REQ-AEB-091).
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
sil_bin="$root/build/host/aeb_sil"
pil_elf="$root/build/cortex-m3/target/aeb_firmware.elf"

for f in "$sil_bin" "$pil_elf"; do
  [[ -f "$f" ]] || { echo "missing $f - build the 'host' and 'cortex-m3' presets first" >&2; exit 2; }
done

sil="$("$sil_bin" | grep -E '^(RESULT|SUMMARY)')" || true
pil="$(qemu-system-arm -M mps2-an385 -cpu cortex-m3 -nographic -monitor none \
        -semihosting-config enable=on,target=native -kernel "$pil_elf" 2>&1 \
        | grep -E '^(RESULT|SUMMARY)')" || true

if diff <(echo "$sil") <(echo "$pil"); then
  echo "SIL and PIL results are identical ($(echo "$sil" | grep -c '^RESULT') scenarios)."
else
  echo "SIL/PIL MISMATCH (lines above: < host, > Cortex-M3)" >&2
  exit 1
fi
