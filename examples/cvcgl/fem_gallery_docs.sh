#!/usr/bin/env bash
# Renders the pictures in docs/FEM_GALLERY.md with fem_gallery.
#
#   examples/cvcgl/fem_gallery_docs.sh FEM_GALLERY HEAD_OFF CT_RAWIV [OUT_DIR]
#
#   FEM_GALLERY  the fem_gallery executable (a CVC_BUILD_EXAMPLES build with libigl)
#   HEAD_OFF     TexMol's LBIE_lib/head.off, one vertex per line (the original packs several
#                vertices on a line, which assimp rejects)
#   CT_RAWIV     VolRover's SimpleExample/vh4.rawiv
#   OUT_DIR      default docs/img/fem-gallery
#
# With python3 and Pillow, each picture is then cut to its content plus a margin, padded to 4:3
# and saved as an 85% JPEG (the docs' copies); without them the full-frame PNGs stay.
set -euo pipefail
if [ $# -lt 3 ]; then
  sed -n '2,13p' "$0"
  exit 2
fi
G=$1 HEAD=$2 CT=$3 OUT=${4:-docs/img/fem-gallery}
mkdir -p "$OUT"

run() {
  local name=$1
  shift
  echo "== $name"
  "$G" "$@" --png "$OUT/$name.png" | sed -n '/^fem_gallery: /{/: wrote /!p}' # LBIE chatters
}

B="--mesh stanford.bunny --up y --zoom 1.25"
H="--mesh $HEAD --up z --zoom 1.2"
V="--volume $CT --inside above --flip --zoom 1.1"

# Finite elements on LBIE tetrahedra (the solid inside an SDF at 128^3, or a CT isovalue)
run bunny_poisson $B --dim 128 --problem poisson --slice y
for s in 010 040 160; do
  run bunny_heat_$s $B --dim 128 --problem heat --steps $((10#$s)) --slice y
done
for m in 1 2 3; do
  run bunny_mode_$m $B --dim 128 --problem modes --mode $m --slice none --colormap turbo
done
run head_poisson $H --dim 128 --problem poisson --slice y
run head_heat_060 $H --dim 128 --problem heat --steps 60 --slice y
run ct_body_poisson $V --iso 45 --problem poisson --slice y
run ct_body_heat $V --iso 45 --problem heat --steps 60 --slice y
run ct_bone_heat $V --iso 80 --problem heat --steps 80 --slice none

# Geometry processing on the triangle surface itself
for m in 5 9; do
  run bunny_surfmode_$m $B --problem surface-modes --mode $m --colormap turbo
done
run head_curvature $H --smooth 2e-4 --problem curvature --colormap viridis
run head_geodesic $H --problem geodesic --colormap turbo
run head_surfmode_6 $H --problem surface-modes --mode 6 --colormap turbo

# LBIE's own elements, cut away
run lbie_bunny_hex_size $B --dim 64 --problem mesh --elements hex --slice y --ghost 0.12
run lbie_bunny_hex_quality $B --dim 64 --problem mesh --elements hex --slice y --color-by quality \
  --range 0,1
run lbie_bunny_tet_quality $B --dim 64 --problem mesh --slice y --color-by quality --range 0,70.5

python3 - "$OUT" <<'PY' || echo "(no python3 + Pillow: the PNGs stay)"
import pathlib, sys
from PIL import Image, ImageChops

for p in sorted(pathlib.Path(sys.argv[1]).glob("*.png")):
    im = Image.open(p).convert("RGB")
    bg = im.getpixel((0, 0))
    box = ImageChops.difference(im, Image.new("RGB", im.size, bg)).getbbox() or (0, 0, *im.size)
    pad = max(box[2] - box[0], box[3] - box[1]) // 12
    w, h = box[2] - box[0] + 2 * pad, box[3] - box[1] + 2 * pad
    w, h = max(w, h * 4 // 3), max(h, w * 3 // 4)
    tile = Image.new("RGB", (w, h), bg)
    tile.paste(im.crop(box), ((w - (box[2] - box[0])) // 2, (h - (box[3] - box[1])) // 2))
    tile.resize((800, 600), Image.LANCZOS).save(p.with_suffix(".jpg"), quality=85, optimize=True)
    p.unlink()
PY
