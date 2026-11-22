# Kuwajima 2019: a Reconstruct series from mouse hippocampus

The contours the contour tiler demo tiles
([contour_tiler.ari](../../../ariadne/documents/contour_tiler.ari), the `contour_lab` node).
They are part of a real serial-section electron microscopy reconstruction, traced in
Reconstruct.

| | |
|---|---|
| What | `VYLNH_20181017.ser` (the series file) and its section files `VYLNH_20181017.1` to `.72` |
| Tissue | mouse hippocampus, area CA1, stratum radiatum: tSEM serial sections, 50 to 57 nm thick (the section files' `thickness`) |
| Kept | the traces of five objects: the axons `ax006` and `ax010` (all 72 sections), `ax008` (sections 1 to 11, branching), and two mitochondria inside `ax010`, `ax010mito01` (21 to 32) and `ax010mito02` (17 to 26) |
| Units | microns |
| Licence | [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/) (public domain dedication) |

## Source

Kuwajima, Masaaki (2019). "3DEM dataset from a mouse hippocampus injected with rAAV to express
ChR2-GFP and mAPEX2". Texas Data Repository, V1.
[doi:10.18738/T8/QP43LB](https://doi.org/10.18738/T8/QP43LB).

The repository record gives the dataset's licence as CC0 1.0. It says the dataset's 3DEM series,
the aligned images and the trace files for Reconstruct, are downloadable from
<https://web.corral.tacc.utexas.edu/3DEM/3dem-1008>. These files were downloaded from that
folder's `3DEM_dataset/` on 2026-11-19. No images were taken.

The dataset is described in Kuwajima M, Ostrovskaya OI, Cao G, Weisberg SA, Harris KM and
Zemelman BV (2020), "Ultrastructure of light-activated axons following optogenetic stimulation
to produce late-phase long-term potentiation", PLoS ONE 15(1): e0226797,
[doi:10.1371/journal.pone.0226797](https://doi.org/10.1371/journal.pone.0226797). CC0 asks for
no attribution, but the repository's community norms ask for a citation, and this is it.

## What was changed

The series file is unchanged. Each section file keeps its `<Section>` element and the
`<Transform>` blocks that hold a kept object's contours. Everything else was left out: the
other objects' contours, the image blocks and their `domain1` contours, and the sections with
no kept contour (`.0`, the calibration grid, and `.73`). The kept text is the original's, apart
from line endings, which git stores as LF. The result is 216 KB, where the original trace files
are 1.3 MB.
