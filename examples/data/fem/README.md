# FEM gallery data

The two datasets the FEM and mesh gallery uses besides the built-in `stanford.bunny`
([docs/FEM_GALLERY.md](../../../docs/FEM_GALLERY.md),
[fem_gallery.ari](../../ariadne/documents/fem_gallery.ari),
[fem_gallery_docs.sh](../../cvcgl/fem_gallery_docs.sh)).

| file | what | from |
|---|---|---|
| `head.off` | a scanned human head: a closed triangle surface, 10,530 vertices and 21,076 triangles, z up | TexMol's `src/LBIE_lib/head.off` |
| `vh4.rawiv` | a CT of a Visible Human torso: 128³ unsigned bytes, unit spacing, stored head down | VolRover's `src/SimpleExample/vh4.rawiv`, a Visible Human subset |

`head.off` is TexMol's file rewritten with one vertex per line. The original packs several
vertices on a line, which assimp rejects; the coordinates and the triangles are unchanged.
`vh4.rawiv` is VolRover's file unchanged (a big-endian RawIV header, then the voxels).

The gallery meshes the head's interior through its signed distance field, and the CT's body
(density above 45) and bone (above 80) directly, turned upside down (`--flip`, or the fe_lab
node's `source.flip`) to stand upright.
