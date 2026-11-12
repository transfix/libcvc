# contour_stack samples

Small hand-made files in the legacy formats `cvc::read_contours` reads
(`inc/cvc/geometry/contours.h`), for `contour_stack_test` (the VolumeRover ports' T9 row). Each is
written the way the legacy program wrote its format; the expected values are in the test.

- `reconstruct/series.ser` and `series.1` .. `series.3`: a Reconstruct series.
  - Section 1: the image's transform with its `domain1` outline (not an object), and object `d01`
    under a translation (`xcoef` 2 1 0 0 0 0, `ycoef` -1 0 1 0 0 0).
  - Section 2: `d01` under a scale (`xcoef` 1 2 0 0 0 0, `ycoef` 0 0 2 0 0 0), the hidden object
    `spine`, and an open trace `ruler`.
  - Section 3: `d01` under a polynomial transform (`xcoef` 0 1 0 0.001 0 0, `ycoef` 0 0 1 0 0
    0.002). Its stored points are the transform of the world square (10, 10)-(20, 20).
- `tiling/slices.config` with `slice1.pts` .. `slice3.pts`: the Tiling program's format, with
  `DETZ`; slice 3 is an empty slice (one point giving its z). `uneven.config` with `uneven1.pts`
  .. `uneven3.pts`: no `DETZ`, slices at z = 0, 1 and 3.
- `ctr2suf/object.contour`: VolumeGridRover's ctr2suf export of one object on two planes: its
  vertices in sorted order, materials `1 2` on every edge.
