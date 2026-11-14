#!/usr/bin/env bash
# Check that a packed cvcpkg bundle's shared libcvc loads from the closure its
# manifest declares.
#
#   bundle-check-closure.sh ARCHIVE
#
# `cvcpkg pack` copies the recipe's depends: into the bundle's required_deps,
# and a consumer's install resolves that list and nothing else. Nothing checked
# that the list covers what libcvc links. libcvc-cuda 3.7.13+cvc.2 was packed
# from a runtime list that had drifted from libcvc's: its libcvc.so needs
# yaml-cpp, ftxui, json-schema-validator, Jolt, curl and opus, its manifest
# declares none of them, and `import pycvc` fails in every prefix that resolves
# libcvc to it.
#
# This installs the declared required_deps from the catalog into an empty
# prefix, as a consumer's resolver would, unpacks the bundle over them and runs
# sdk-check-closure.sh on the result. A bundle that cannot load from what it
# declares then fails before it is published. The platform, arch, config and
# link come from the bundle's own manifest.
#
# Linux and macOS (the tar bundles). Run it on release bundles: the catalog has
# no debug build of several of libcvc's dependencies (eigen, for one), so a
# debug install of the declared list fails before there is anything to check.
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "usage: $0 ARCHIVE" >&2
  exit 2
fi
archive=$1
here=$(cd "$(dirname "$0")" && pwd)

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/bundle" "$work/prefix"

# GNU and BSD tar both detect the compression from the bytes, so a gzip
# bundle the server names .tar.zst unpacks too.
tar -xf "$archive" -C "$work/bundle"
manifest=$(find "$work/bundle/share/libcvc-deps" -mindepth 2 -maxdepth 2 -name manifest.yaml | head -1)
if [ -z "$manifest" ]; then
  echo "::error::$(basename "$archive") has no share/libcvc-deps/*/manifest.yaml" >&2
  exit 1
fi

# The manifest is cvcpkg's own YAML: `bundle:` holds the target keys at two
# spaces, `dependencies:` > `required:` lists `- name: X` entries.
field() { sed -n "/^bundle:/,/^[^ ]/s/^  $1: *//p" "$manifest" | head -1; }
name=$(field name)
platform=$(field platform)
arch=$(field arch)
config=$(field build_type)
link=$(field link)
deps=$(awk '
  /^dependencies:/ { in_deps = 1; next }
  in_deps && /^[^ ]/ { in_deps = 0 }
  in_deps && /^  required:/ { in_req = 1; next }
  in_req && /^  [^ -]/ { in_req = 0 }
  in_req && /^  - name: / { sub(/^  - name: */, ""); print }
' "$manifest")

echo "$name $(field version) ($platform/$arch/$config/$link) declares: $(tr '\n' ' ' <<<"$deps")"
if [ -n "$deps" ]; then
  # shellcheck disable=SC2086  # one argument per declared name
  cvcpkg install $deps --prefix "$work/prefix" \
    --platform "$platform" --arch "$arch" --config "$config" --link "$link"
fi
cp -a "$work/bundle/." "$work/prefix/"

bash "$here/sdk-check-closure.sh" "$work/prefix"
