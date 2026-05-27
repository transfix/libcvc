# CVC Command-Line Interface Guide

The `cvc` command is a unified CLI that consolidates all of libcvc's
volume processing, geometry manipulation, meshing, distributed state,
and script execution into a single tool with subcommands.

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Command Reference](#command-reference)
- [Working with Volume Data](#working-with-volume-data)
  - [Inspecting Files](#inspecting-files)
  - [Format Conversion](#format-conversion)
  - [Volume Arithmetic](#volume-arithmetic)
  - [Volume Transforms](#volume-transforms)
  - [Comparing Volumes (SSIM)](#comparing-volumes-ssim)
  - [Image I/O](#image-io)
- [Working with Geometry](#working-with-geometry)
  - [Signed Distance Fields (SDF)](#signed-distance-fields-sdf)
  - [Isosurface Extraction](#isosurface-extraction)
  - [Volumetric Meshing](#volumetric-meshing)
  - [Layer Meshing](#layer-meshing)
- [End-to-End Pipelines](#end-to-end-pipelines)
- [State Tree](#state-tree)
- [Script Execution (state_exec)](#script-execution-state_exec)
- [Distributed State Server](#distributed-state-server)
  - [Standalone Server](#standalone-server)
  - [Forming a Cluster](#forming-a-cluster)
  - [Subtree Delegation](#subtree-delegation)
  - [Authentication and TLS](#authentication-and-tls)
  - [Cluster Health Monitoring](#cluster-health-monitoring)
- [Build Options](#build-options)

---

## Installation

`cvc` is built automatically as part of libcvc:

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
# The binary is at build/bin/cvc
```

To install system-wide:

```bash
sudo cmake --install .
# Now available as: cvc
```

## Quick Start

```bash
# Generate a test mesh (Stanford bunny)
cvc bunny -o bunny.off

# Inspect the geometry
cvc info bunny.off

# Compute a signed distance field
cvc sdf -i bunny.off -o bunny_sdf.rawiv -d 64,64,64

# Extract an isosurface at the zero level set
cvc iso -i bunny_sdf.rawiv -o reconstructed.off -v 0.0

# Run a state_exec expression
cvc exec -e '(+ 1 2 3)'
```

## Command Reference

Run `cvc --help` for the full list, or `cvc <command> --help` for
details on any command.

| Category | Command | Description |
|----------|---------|-------------|
| **File Info** | `info` | Display file metadata (volume or geometry) |
| | `stats` | Compute volume statistics (min, max, mean, std) |
| **Conversion** | `copy` | Copy/convert files (auto-detects type) |
| | `convert` | Convert volume format or voxel type |
| **Geometry** | `sdf` | Compute signed distance field from geometry |
| | `iso` | Extract isosurface from volume |
| | `tetrahedralize` | Extract tetrahedral mesh from volume |
| | `hexahedralize` | Extract hexahedral mesh from volume |
| | `tetrahedralize2` | Extract dual-tet (tet2) mesh from volume |
| | `layer-mesh` | Mesh the layer between two isosurfaces |
| **Vol Arithmetic** | `add` | Add two volumes element-wise |
| | `subtract` | Subtract two volumes element-wise |
| | `scale` | Multiply volume by scalar |
| | `normalize` | Remap voxel values to [min, max] |
| | `clip` | Zero voxels above threshold |
| | `negate` | Negate all voxel values |
| | `mask` | Apply mask volume |
| | `downsample` | Reduce volume resolution |
| **Vol Transform** | `rotate` | Rotate volume around Z-axis |
| **Analysis** | `ssim` | Compute SSIM between two volumes |
| **Projection** | `project` | Forward ray projection |
| | `backproject` | Filtered back-projection (FBP) |
| **Image I/O** | `vol2img` | Export volume slices as images |
| | `img2vol` | Import image stack as volume |
| | `rgba-merge` | Merge 4 volumes into RGBA |
| **Test Data** | `bunny` | Output Stanford bunny geometry or SDF |
| **State Server** | `serve` | Run headless CVC state server |
| | `cluster-status` | Display cluster health report |
| **State Exec** | `exec` | Run state_exec script |
| | `ps` | List state_exec processes |
| **State** | `state` | Get/set/list state tree values |

---

## Working with Volume Data

### Inspecting Files

```bash
# Volume metadata (dimensions, type, bounding box)
cvc info data.rawiv

# Geometry metadata (vertex and face counts)
cvc info mesh.off

# Detailed volume statistics
cvc stats data.rawiv
```

### Format Conversion

```bash
# Convert between volume formats
cvc copy input.rawiv output.mrc

# Change voxel type (e.g., float to unsigned char)
cvc convert -i input.rawiv -o output.rawiv -t uchar

# Supported voxel types: uchar, ushort, uint, float, double
```

### Volume Arithmetic

```bash
# Add two volumes element-wise
cvc add -i vol_a.rawiv vol_b.rawiv -o sum.rawiv

# Subtract: result = vol_a - vol_b
cvc subtract -i vol_a.rawiv vol_b.rawiv -o diff.rawiv

# Scale all voxels by a factor
cvc scale -i data.rawiv -o scaled.rawiv -s 2.5

# Normalize to a range
cvc normalize -i data.rawiv -o norm.rawiv --min 0 --max 255

# Clip voxels above a threshold to zero
cvc clip -i data.rawiv -o clipped.rawiv -t 200

# Negate all voxel values
cvc negate -i data.rawiv -o negated.rawiv

# Apply a binary mask
cvc mask -i data.rawiv -m mask.rawiv -o masked.rawiv

# Apply inverse mask (keep voxels where mask is zero)
cvc mask -i data.rawiv -m mask.rawiv -o masked.rawiv --inverse

# Downsample by a factor
cvc downsample -i data.rawiv -o small.rawiv -f 2
```

### Volume Transforms

```bash
# Rotate volume around the Z-axis
cvc rotate -i data.rawiv -o rotated.rawiv -a 45.0
```

### Comparing Volumes (SSIM)

```bash
# Compute structural similarity index
cvc ssim -i original.rawiv -j reconstructed.rawiv

# Save the SSIM error map
cvc ssim -i original.rawiv -j reconstructed.rawiv -o ssim_map.rawiv
```

### Image I/O

```bash
# Export volume slices as PNG images
cvc vol2img -i data.rawiv -o slices/ --dir z

# Import image stack as a volume
cvc img2vol -i slices/ -o volume.rawiv

# Merge 4 single-channel volumes into RGBA
cvc rgba-merge -r red.rawiv -g green.rawiv -b blue.rawiv -a alpha.rawiv -o rgba.rawiv
```

---

## Working with Geometry

### Signed Distance Fields (SDF)

Compute a signed distance field from triangle geometry. The SDF
encodes the distance from each grid point to the nearest surface,
with sign indicating inside (negative) vs. outside (positive).

```bash
# Basic SDF at 64³ resolution
cvc sdf -i mesh.off -o sdf.rawiv -d 64,64,64

# Use the v2 algorithm (distance transform, faster for smaller grids)
cvc sdf -i mesh.off -o sdf.rawiv -d 32,32,32 -a v2

# Use the v1 algorithm (octree-based, requires power-of-2 dims ≥ 16)
cvc sdf -i mesh.off -o sdf.rawiv -d 128,128,128 -a v1

# Flip surface normals (inverts inside/outside)
cvc sdf -i mesh.off -o sdf.rawiv -d 64,64,64 --flip-normals

# Specify a custom bounding box
cvc sdf -i mesh.off -o sdf.rawiv -d 64,64,64 \
    --bbox -1,-1,-1,1,1,1
```

**Algorithm comparison:**

| Algorithm | Flag | Min Dim | Best For |
|-----------|------|---------|----------|
| v1 (octree) | `-a v1` | 16 (power of 2) | Large grids (128³+), production |
| v2 (distance transform) | `-a v2` | Any | Small grids, quick previews |

### Isosurface Extraction

Extract a triangle mesh from a volume at a given isovalue:

```bash
# Basic isosurface extraction
cvc iso -i volume.rawiv -o surface.off -v 0.0

# Choose extraction method
cvc iso -i volume.rawiv -o surface.off -v 0.0 -m fastcontouring
cvc iso -i volume.rawiv -o surface.off -v 0.0 -m libisocontour

# With quality improvement (N iterations of smoothing)
cvc iso -i volume.rawiv -o surface.off -v 0.0 -q 5

# Choose normal computation method
cvc iso -i volume.rawiv -o surface.off -v 0.0 -n central-diff
cvc iso -i volume.rawiv -o surface.off -v 0.0 -n bspline-interp

# Interpolate property values from a second volume
cvc iso -i sdf.rawiv -o surface.off -v 0.0 -p property.rawiv
```

**Extraction methods:**

| Method | Flag | Description |
|--------|------|-------------|
| `duallib` | `-m duallib` | Dual contouring library (default) |
| `fastcontouring` | `-m fastcontouring` | Fast contouring |
| `libisocontour` | `-m libisocontour` | ISO contouring library |

**Normal types:**

| Method | Flag | Description |
|--------|------|-------------|
| `bspline-conv` | `-n bspline-conv` | B-spline convolution (default) |
| `central-diff` | `-n central-diff` | Central difference |
| `bspline-interp` | `-n bspline-interp` | B-spline interpolation |

### Volumetric Meshing

Generate tetrahedral or hexahedral volumetric meshes:

```bash
# Tetrahedral mesh
cvc tetrahedralize -i sdf.rawiv -o tet_mesh.off -v 0.0

# Hexahedral mesh
cvc hexahedralize -i sdf.rawiv -o hex_mesh.off -v 0.0

# Dual tetrahedral mesh (TETRA2 element type)
cvc tetrahedralize2 -i sdf.rawiv -o tet2_mesh.off -v 0.0

# With quality improvement
cvc tetrahedralize -i sdf.rawiv -o tet_mesh.off -v 0.0 \
    --improve geo-flow --iterations 10

# With property volume interpolation
cvc tetrahedralize -i sdf.rawiv -o tet_mesh.off -v 0.0 \
    -p property.rawiv
```

**Improvement methods:**

| Method | Flag | Description |
|--------|------|-------------|
| `none` | `--improve none` | No improvement (default) |
| `geo-flow` | `--improve geo-flow` | Geometric flow smoothing |
| `edge-contract` | `--improve edge-contract` | Edge contraction |
| `joe-liu` | `--improve joe-liu` | Joe-Liu method |
| `minimal-vol` | `--improve minimal-vol` | Minimal volume |
| `optimization` | `--improve optimization` | Optimization-based |

### Layer Meshing

Mesh the region between two isosurfaces — useful for shells, cortical
layers, or material boundaries:

```bash
# Mesh the shell between isovalue -0.1 (outer) and 0.1 (inner)
cvc layer-mesh -i sdf.rawiv -o shell.off \
    --isovalue-outer -0.1 --isovalue-inner 0.1

# With quality improvement
cvc layer-mesh -i sdf.rawiv -o shell.off \
    --isovalue-outer -0.5 --isovalue-inner 0.5 \
    --improve geo-flow --iterations 5
```

---

## End-to-End Pipelines

### Geometry → SDF → Isosurface Roundtrip

```bash
# 1. Start with a mesh
cvc bunny -o bunny.off
cvc info bunny.off

# 2. Compute SDF
cvc sdf -i bunny.off -o bunny_sdf.rawiv -d 64,64,64 -a v2
cvc info bunny_sdf.rawiv

# 3. Reconstruct the surface
cvc iso -i bunny_sdf.rawiv -o bunny_reconstructed.off -v 0.0
cvc info bunny_reconstructed.off

# 4. Compare: compute SDF of the reconstruction
cvc sdf -i bunny_reconstructed.off -o recon_sdf.rawiv -d 64,64,64 -a v2

# 5. Measure difference
cvc ssim -i bunny_sdf.rawiv -j recon_sdf.rawiv -o diff_map.rawiv
```

### Volume Processing Pipeline

```bash
# 1. Inspect input data
cvc info scan.mrc
cvc stats scan.mrc

# 2. Normalize
cvc normalize -i scan.mrc -o normalized.rawiv --min 0 --max 1

# 3. Downsample for preview
cvc downsample -i normalized.rawiv -o preview.rawiv -f 2

# 4. Extract isosurface
cvc iso -i normalized.rawiv -o surface.off -v 0.5

# 5. Generate tetrahedral mesh for simulation
cvc tetrahedralize -i normalized.rawiv -o tet_mesh.off -v 0.5 \
    --improve geo-flow --iterations 5
```

### Multi-Material Layer Meshing

```bash
# Compute SDF of a structure
cvc sdf -i structure.off -o sdf.rawiv -d 128,128,128

# Mesh the outer shell (between -0.2 and 0.0)
cvc layer-mesh -i sdf.rawiv -o outer_shell.off \
    --isovalue-outer -0.2 --isovalue-inner 0.0

# Mesh the inner core (between 0.0 and 0.5)
cvc layer-mesh -i sdf.rawiv -o inner_core.off \
    --isovalue-outer 0.0 --isovalue-inner 0.5
```

---

## State Tree

The CVC state tree is a hierarchical key-value store that underpins
the runtime configuration and data flow in libcvc and volrover3.

```bash
# Set a value
cvc state set scene.camera.x 1.5

# Get a value
cvc state get scene.camera.x

# List children of a node
cvc state list scene.camera

# Export the entire tree as JSON
cvc state json

# Export a subtree as JSON
cvc state json scene

# Delete a value
cvc state delete scene.camera.x
```

---

## Script Execution (state_exec)

`cvc exec` runs programs written in state_exec, a Scheme-like language
that can read and write the CVC state tree. Useful for scripting batch
operations, automating pipelines, and extending CVC at runtime.

### Inline Expressions

```bash
# Arithmetic
cvc exec -e '(+ 1 2 3)'          # → 6
cvc exec -e '(* 7 6)'            # → 42
cvc exec -e '(/ 355 113.0)'      # → 3.14159...

# Variables and functions
cvc exec -e '(let ((x 10) (y 20)) (+ x y))'   # → 30
cvc exec -e '(let ((sq (lambda (x) (* x x)))) (sq 9))'  # → 81

# Conditionals
cvc exec -e '(if (> 5 3) 99 0)'  # → 99

# Lists
cvc exec -e '(car (list 10 20 30))'   # → 10
cvc exec -e '(cdr (list 10 20 30))'   # → (20 30)
cvc exec -e '(length (list 1 2 3 4))' # → 4
```

### Script Files

```bash
# Write a script
cat > pipeline.sx << 'EOF'
(let ((pi 3.14159)
      (r  5))
  (* pi r r))
EOF

# Execute it
cvc exec -f pipeline.sx          # → 78.53975
```

### Resource Limits

```bash
# Limit execution steps (prevent runaway scripts)
cvc exec -e '(+ 1 2)' --max-steps 1000

# Limit execution time (in seconds)
cvc exec -e '(+ 1 2)' --max-time 5.0

# Name the process (for ps output)
cvc exec -e '(+ 1 2)' --name my-computation
```

### Process Listing

```bash
# List running state_exec processes
cvc ps
```

---

## Distributed State Server

The `serve` command runs a headless CVC state server that volrover3
instances — or other CVC clients — can connect to for shared state
replication. The server uses the `distributed_state_session` API which
supports multiple transport backends, clustering, authentication,
subtree delegation, and optional script execution.

### Standalone Server

```bash
# Start a server on a Unix socket (same-host usage)
cvc serve -l /tmp/cvc_state.sock -t ipc

# Start a server on a TCP port (networked, requires CVC_ENABLE_GRPC)
cvc serve -l 0.0.0.0:50051 -t grpc

# With a custom cluster and node identity
cvc serve -l 0.0.0.0:50051 -t grpc \
    --cluster-id my-lab \
    --node-id workstation-1

# Enable the script execution engine
cvc serve -l 0.0.0.0:50051 -t grpc --enable-exec

# Use persistent blob storage (for large volumes/geometries)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --blob-store-path /var/lib/cvc/blobs
```

### Forming a Cluster

Connect multiple servers into a peer cluster. Each node connects to
one or more seed peers; state mutations replicate automatically.

```bash
# Node 1 (first node — no seeds needed)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --cluster-id research-lab \
    --node-id node-1

# Node 2 (joins node-1)
cvc serve -l 0.0.0.0:50052 -t grpc \
    --cluster-id research-lab \
    --node-id node-2 \
    --seed 192.168.1.10:50051

# Node 3 (joins both for redundancy)
cvc serve -l 0.0.0.0:50053 -t grpc \
    --cluster-id research-lab \
    --node-id node-3 \
    --seed 192.168.1.10:50051 192.168.1.11:50052

# Enable conflict resolution (last-writer-wins)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --cluster-id research-lab \
    --node-id node-1 \
    --resolve-conflicts
```

**Sync modes** control how each node participates:

| Mode | Flag | Description |
|------|------|-------------|
| `read-write` | `--sync-mode read-write` | Bidirectional replication (default) |
| `read-only` | `--sync-mode read-only` | Mirror remote writes, reject local |
| `authoritative` | `--sync-mode authoritative` | This node owns writes |

### Subtree Delegation

Delegate ownership of a state subtree to a different cluster — useful
for multi-team setups where different groups own different parts of
the state tree.

```bash
# Delegate the "simulation" subtree to a compute cluster
cvc serve -l 0.0.0.0:50051 -t grpc \
    --cluster-id viz-cluster \
    --node-id viz-1 \
    --delegate simulation:compute-cluster:compute-1.internal:50051

# Delegate with a lease duration (3600 seconds = 1 hour)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --cluster-id viz-cluster \
    --node-id viz-1 \
    --delegate rendering.gpus:gpu-cluster:gpu-node-1:50051:3600

# Multiple delegations
cvc serve -l 0.0.0.0:50051 -t grpc \
    --cluster-id main \
    --node-id coordinator \
    --delegate sim:sim-cluster:sim-1:50051 \
    --delegate viz:viz-cluster:viz-1:50052
```

**Delegation format:** `path:cluster_id:endpoint[:lease_seconds]`

### Authentication and TLS

For production deployments, enable TLS encryption and bearer token
authentication:

```bash
# TLS only (encrypted, no client auth)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --tls-cert /etc/cvc/server.pem \
    --tls-key /etc/cvc/server-key.pem \
    --tls-ca /etc/cvc/ca.pem

# Mutual TLS (both sides authenticate)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --tls-cert /etc/cvc/server.pem \
    --tls-key /etc/cvc/server-key.pem \
    --tls-ca /etc/cvc/ca.pem \
    --tls-require-client-auth

# Bearer token authentication
cvc serve -l 0.0.0.0:50051 -t grpc \
    --auth-token my-secret-token

# TLS + auth (recommended for production)
cvc serve -l 0.0.0.0:50051 -t grpc \
    --tls-cert /etc/cvc/server.pem \
    --tls-key /etc/cvc/server-key.pem \
    --tls-ca /etc/cvc/ca.pem \
    --auth-token my-secret-token

# Enforcement policies
cvc serve -l 0.0.0.0:50051 -t grpc \
    --enforce-authority \
    --enforce-write-policy \
    --resolve-conflicts
```

### Cluster Health Monitoring

Connect to a running cluster and print a diagnostic report:

```bash
# Check cluster health
cvc cluster-status \
    -l /tmp/status_probe.sock -t ipc \
    --seed /tmp/cvc_state.sock

# Over gRPC
cvc cluster-status \
    -l 0.0.0.0:50099 -t grpc \
    --seed 192.168.1.10:50051 \
    --cluster-id research-lab

# With authentication
cvc cluster-status \
    -l 0.0.0.0:50099 -t grpc \
    --seed 192.168.1.10:50051 \
    --auth-token my-secret-token
```

The report includes peer status, delegation table, blob store
statistics, shard replication metrics, and conflict counts.

---

## Build Options

The commands available in `cvc` depend on build-time flags:

| Feature | CMake Flag | Default | Commands Affected |
|---------|-----------|---------|-------------------|
| SDF computation | `CVC_ENABLE_SDF` | ON | `sdf`, `bunny --volume` |
| Meshing (LBIE) | `CVC_ENABLE_MESHER` | ON | `iso`, `tetrahedralize`, `hexahedralize`, `tetrahedralize2`, `layer-mesh` |
| gRPC transport | `CVC_ENABLE_GRPC` | OFF | `serve -t grpc`, `cluster-status -t grpc` |
| Legacy XMLRPC | `CVC_USING_XMLRPC` | OFF | `server`, `client` (legacy) |

The `serve` command always supports IPC transport (`-t ipc`). To enable
gRPC for networked clustering, build with:

```bash
cmake .. -DCVC_ENABLE_GRPC=ON
```

The `exec`, `state`, and `ps` commands are always available.
