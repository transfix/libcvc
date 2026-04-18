# volutils — Unified Volume Utility CLI

`volutils` consolidates the ~60 separate VolUtils programs from VolumeRover into a single command-line tool with subcommands.

## Building

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target volutils
```

Optional dependencies (enabled by default):

| Feature | CMake option | Packages |
|---------|-------------|----------|
| ImageMagick (image I/O) | `CVC_ENABLE_IMAGEMAGICK` | `libmagick++-dev` (Debian/Ubuntu), `imagemagick` (Homebrew) |
| FFTW (filtered back-projection) | `CVC_ENABLE_FFTW` | `libfftw3-dev` (Debian/Ubuntu), `fftw` (Homebrew) |

## Usage

```
volutils <command> [options]
volutils <command> --help
```

## Commands

### Metadata & Statistics

#### `info` — Display volume file metadata
```bash
volutils info volume.rawiv
```

#### `stats` — Compute volume statistics (min, max, mean, std dev)
```bash
volutils stats volume.rawiv
volutils stats volume.rawiv --region 0,0,0,1,1,1
```

### Format Conversion

#### `convert` — Convert format or voxel type
```bash
volutils convert input.rawiv output.rawv
volutils convert input.rawiv output.rawiv --type float
```
Supported formats: RawIV, RawV, MRC, INR, Spider.

### Arithmetic

#### `add` — Add two volumes element-wise
```bash
volutils add -i a.rawiv b.rawiv -o sum.rawiv
```

#### `subtract` — Subtract two volumes element-wise
```bash
volutils subtract -i a.rawiv b.rawiv -o diff.rawiv
```

#### `scale` — Multiply volume by scalar
```bash
volutils scale -i input.rawiv -o output.rawiv --factor 2.5
```

#### `negate` — Negate all voxel values
```bash
volutils negate -i input.rawiv -o output.rawiv
```

### Normalization & Filtering

#### `normalize` — Remap voxel values to [min, max]
```bash
volutils normalize -i input.rawiv -o output.rawiv --min 0 --max 1
```

#### `clip` — Zero voxels above threshold
```bash
volutils clip -i input.rawiv -o output.rawiv --threshold 100
```

#### `mask` — Apply mask volume (zero where mask is nonzero)
```bash
volutils mask -i input.rawiv -o output.rawiv --mask mask.rawiv
```

### Spatial Operations

#### `downsample` — Reduce volume resolution
```bash
volutils downsample -i input.rawiv -o output.rawiv --factor 2
volutils downsample -i input.rawiv -o output.rawiv --fx 2 --fy 2 --fz 4
```

#### `rotate` — Rotate volume around Z-axis
```bash
volutils rotate -i input.rawiv -o output.rawiv --angle 45
volutils rotate -i input.rawiv -o output --count 36    # 36 evenly-spaced rotations
```

### Quality Metrics

#### `ssim` — Compute Structural Similarity Index (Wang et al. 2004)
```bash
volutils ssim -i reference.rawiv test.rawiv
volutils ssim -i reference.rawiv test.rawiv -o ssim_map.rawiv --window 11 --sigma 1.5
```

### Tomographic Projection

#### `project` — Forward ray projection
```bash
# angles.txt: one angle in degrees per line
volutils project -i volume.rawiv -o projections.rawiv --angles angles.txt --step 0.5
```

#### `backproject` — Filtered back-projection (FBP) reconstruction
Requires FFTW for the ramp filter (use `--no-filter` without FFTW).
```bash
volutils backproject -i projections.rawiv -o reconstructed.rawiv --angles angles.txt --dim 128
volutils backproject -i projections.rawiv -o reconstructed.rawiv --angles angles.txt --dim 128 --no-filter
```

### Image I/O (requires ImageMagick)

#### `vol2img` — Export volume slices as images
```bash
volutils vol2img -i volume.rawiv -d slices/
volutils vol2img -i volume.rawiv -d slices/ --format "slice_%05d.tiff"
```

#### `img2vol` — Import image stack as volume
```bash
volutils img2vol -i slice_000.png slice_001.png slice_002.png -o volume.rawiv
```

### Multi-variable / RGBA

#### `rgba-merge` — Merge 4 volumes into interleaved RGBA
```bash
volutils rgba-merge -i red.rawiv green.rawiv blue.rawiv alpha.rawiv -o rgba.rawiv
```

### Geometry

#### `bunny` — Output Stanford bunny geometry or SDF volume
```bash
volutils bunny mesh -o bunny.raw        # triangle mesh
volutils bunny mesh -o bunny.off        # OFF format
volutils bunny sdf -o bunny_sdf.rawiv   # signed distance field
volutils bunny sdf -o bunny_sdf.rawiv --dim 64
```

## Supported Volume Formats

| Format | Extension | Read | Write |
|--------|-----------|------|-------|
| RawIV  | `.rawiv`  | Yes  | Yes   |
| RawV   | `.rawv`   | Yes  | Yes   |
| MRC    | `.mrc`    | Yes  | Yes   |
| INR    | `.inr`    | Yes  | Yes   |
| Spider | `.spi`, `.vol` | Yes | Yes |

## Library API

All operations are available as C++ functions in `<cvc/volume_ops.h>` under the `cvc` namespace. See the header for full documentation.
