# point_classes samples

Small hand-made files in VolumeGridRover's formats that `cvc::read_point_classes` reads
(`inc/cvc/volume/point_classes.h`), for `point_classes_test` (the VolumeRover ports' T9 row).

- `legacy.vgr`: point classes as VolumeGridRover's `savePointClassesSlot` wrote them through
  QDomDocument: a DOCTYPE, attributes in no particular order, no radius, and the same class name
  for two timesteps. It also repeats one point, which reads once.
- `legacy.pts`: a point list as `loadPointClassesSlot` read it: per class, a count and that many
  object coordinates. With dims 11 x 11 x 11 and the box (0, 0, 0)-(10, 10, 10), the coordinates
  become voxel indices by truncation.
