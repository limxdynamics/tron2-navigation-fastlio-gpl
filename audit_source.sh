#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fail() { printf 'GPL_SOURCE_AUDIT_FAIL: %s\n' "$*" >&2; exit 1; }

for command_name in file git sha256sum; do
  command -v "${command_name}" >/dev/null 2>&1 || fail "missing command: ${command_name}"
done

required=(
  LICENSE README.md RELEASE_INFO.md SOURCE_INVENTORY.sha256
  FAST_LIO/LICENSE FAST_LIO/NOTICE FAST_LIO/README.md FAST_LIO/package.xml
  FAST_LIO/config/rs_fairy.yaml FAST_LIO/launch/mapping.launch.py
  FAST_LIO/src/laserMapping.cpp
  FAST_LIO/include/ikd-Tree/LICENSE FAST_LIO/include/ikd-Tree/ikd_Tree.h
  FAST_LIO/include/ikd-Tree/ikd_Tree.cpp
  FAST_LIO_LOCALIZATION2/LICENSE FAST_LIO_LOCALIZATION2/NOTICE
  FAST_LIO_LOCALIZATION2/README.md FAST_LIO_LOCALIZATION2/package.xml
  FAST_LIO_LOCALIZATION2/include/ikd-Tree/LICENSE
  FAST_LIO_LOCALIZATION2/include/ikd-Tree/ikd_Tree.h
  FAST_LIO_LOCALIZATION2/include/ikd-Tree/ikd_Tree.cpp
)
for relative in "${required[@]}"; do
  [[ -f "${ROOT}/${relative}" ]] || fail "missing required source: ${relative}"
done

grep -Eq '<license>[[:space:]]*GPL-2.0-only[[:space:]]*</license>' \
  "${ROOT}/FAST_LIO/package.xml" || fail 'FAST_LIO license declaration is not GPL-2.0-only'
grep -Eq '<license>[[:space:]]*GPL-2.0-only[[:space:]]*</license>' \
  "${ROOT}/FAST_LIO_LOCALIZATION2/package.xml" \
  || fail 'localization license declaration is not GPL-2.0-only'
for notice in "${ROOT}/FAST_LIO/NOTICE" "${ROOT}/FAST_LIO_LOCALIZATION2/NOTICE"; do
  grep -Fq 'Baseline commit:' "${notice}" || fail "baseline missing from ${notice#"${ROOT}/"}"
  grep -Fq 'local modifications made in 2026' "${notice}" \
    || fail "modification notice missing from ${notice#"${ROOT}/"}"
done

[[ -f "${ROOT}/SOURCE_INVENTORY.sha256" ]] || fail 'inventory missing'
(
  cd "${ROOT}"
  sha256sum -c SOURCE_INVENTORY.sha256 >/dev/null
) || fail 'source inventory mismatch'
inventory_paths="$(mktemp)"
actual_paths="$(mktemp)"
trap 'rm -f "${inventory_paths}" "${actual_paths}"' EXIT
awk '{sub(/^\.\//, "", $2); print $2}' "${ROOT}/SOURCE_INVENTORY.sha256" \
  | LC_ALL=C sort >"${inventory_paths}"
find "${ROOT}" -type f ! -path '*/.git/*' ! -name SOURCE_INVENTORY.sha256 \
  -printf '%P\n' | LC_ALL=C sort >"${actual_paths}"
cmp -s "${inventory_paths}" "${actual_paths}" || fail 'inventory file set mismatch'

if find "${ROOT}" -type f ! -path '*/.git/*' \
  \( -name '*.o' -o -name '*.a' -o -name '*.so' -o -name '*.pyc' \
     -o -name '*.pcd' -o -name '*.bag' -o -name '*.db3' -o -name '*.mcap' \) \
  -print -quit | grep -q .; then
  fail 'generated binary, map, or recording found'
fi
if find "${ROOT}" -type f ! -path '*/.git/*' -size +25M -print -quit | grep -q .; then
  fail 'file larger than 25 MiB found'
fi
while IFS= read -r relative; do
  (( ${#relative} <= 180 )) || fail "path exceeds 180 characters: ${relative}"
done < <(find "${ROOT}" -mindepth 1 ! -path '*/.git/*' -printf '%P\n')

sensitive='(^|[^0-9])(10\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}|172\.(1[6-9]|2[0-9]|3[01])\.[0-9]{1,3}\.[0-9]{1,3}|192\.168\.[0-9]{1,3}\.[0-9]{1,3})([^0-9]|$)|WF_TRON[A-Za-z0-9_]*|guest@|/home/[A-Za-z0-9_.-]+(/|[^A-Za-z0-9_])|/var/limx'
if grep -RInI -E --exclude=audit_source.sh --exclude=SOURCE_INVENTORY.sha256 \
  --exclude-dir=.git -- "${sensitive}" "${ROOT}" | grep -q .; then
  fail 'private address, identity, or machine path found'
fi

echo 'GPL_SOURCE_AUDIT=PASS unit=FAST_LIO_AND_LOCALIZATION'
