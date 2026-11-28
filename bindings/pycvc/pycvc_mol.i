// pycvc_mol.i — molecules (inc/cvc/mol/): the TexMol ports' cvc::mol, read and written, viewed,
// grouped, measured and blurred, with their parameter tables so each takes keywords:
//
//   m = pycvc.read_molecule(app, "1grl.pdb")                # or hydrogens=False, model=0, ...
//   len(m.atoms), m.atoms[0].name, m.structures[0].type, m.bounds()
//   pycvc.write_molecule(app, m, "1grl.xyzr")               # .pdb .ent .pdb1 .pqr .xyzr .pts
//   v = pycvc.build_view(app, m, style="cartoon")           # the meshes cvcGL's molecule node draws
//   v.cartoon.num_triangles(), v.atoms_drawn, v.radii
//   t = pycvc.torsions(m)                                   # radians, NaN where undefined
//   g = pycvc.build_group_tree(m); pycvc.flatten_atoms(g)   # TexMol's PDBParser tree and walk
//   d = pycvc.blur(app, m, resolution=2.0, blobbiness=-0.5) # a volume
//
// An enum keyword takes its name as text (style="cartoon", color_by="chain", radii=
// "solvent_enlarged"), or the constant (pycvc.view_style_cartoon). The long ones (read and write,
// build_view, torsions, build_group_tree, blur) release the GIL; pycvc_jobs.i has them as resumable
// jobs. A file that can't be read or written raises IOError, parameters that aren't valid
// ValueError. The steppers and the stream readers are not wrapped: a file name, or a molecule
// built in Python, is how Python passes one.
%{
#include <cvc/mol/blur.h>
#include <cvc/mol/groups.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/torsions.h>
#include <cvc/mol/view.h>
%}

namespace std {
  %template(MolAtomVector) vector<cvc::mol::atom>;
  %template(MolStructureVector) vector<cvc::mol::secondary_structure>;
  %template(MolBondVector) vector<cvc::mol::bond>;
  %template(MolTraceVector) vector<cvc::mol::trace_residue>;
  %template(MolTorsionsVector) vector<cvc::mol::residue_torsions>;
  %template(MolGroupVector) vector<cvc::mol::group>;
}

// What Python has no use for or can't hold: the table's rows (string_view), the handler registry,
// the stream readers and writers, the steppers and jobs, the functions taking an app & (given
// below with the app pycvc.make_app() makes), and the arrays (given as lists below).
%ignore cvc::mol::residue_atom_info;
%ignore cvc::mol::residue_atom_table;
%ignore cvc::mol::find_residue_atom;
%ignore cvc::mol::molecule_file_error;
%ignore cvc::mol::molecule_file_io;
%ignore cvc::mol::molecule_handlers;
%ignore cvc::mol::molecule_read_extensions;
%ignore cvc::mol::molecule_write_extensions;
%ignore cvc::mol::read_molecule;
%ignore cvc::mol::write_molecule;
%ignore cvc::mol::read_pdb;
%ignore cvc::mol::read_pqr;
%ignore cvc::mol::read_xyzr;
%ignore cvc::mol::read_pts;
%ignore cvc::mol::write_pdb;
%ignore cvc::mol::write_pqr;
%ignore cvc::mol::write_xyzr;
%ignore cvc::mol::write_pts;
%ignore cvc::mol::blur_stepper;
%ignore cvc::mol::blur;
%ignore cvc::mol::blur_job;
%ignore cvc::mol::build_view_stepper;
%ignore cvc::mol::build_view;
%ignore cvc::mol::build_view_job;
%ignore cvc::mol::group_tree_stepper;
%ignore cvc::mol::group_tree_job;
%ignore cvc::mol::torsion_stepper;
%ignore cvc::mol::torsions_job;
%ignore cvc::mol::rgb8;
%ignore cvc::mol::element_color;
%ignore cvc::mol::chain_color;
%ignore cvc::mol::residue_color;
%ignore cvc::mol::structure_color;
%ignore cvc::mol::b_factor_color;
%ignore cvc::mol::atom_colors;
%ignore cvc::mol::dihedral;
%ignore cvc::mol::pick_sphere;
%ignore cvc::mol::nearest_drawn_atom;
%ignore cvc::mol::flatten_atoms;
%rename(flatten_atoms) pycvc_flatten_atoms; // a global flatten_atoms would meet cvc::mol's by ADL
%ignore cvc::mol::molecule::bounds;
%ignore cvc::mol::group::children;
%ignore cvc::mol::group::atoms;
%ignore cvc::mol::residue_torsions::nu;
%ignore cvc::mol::molecule_view::centers;
%ignore cvc::mol::molecule_view::radii;
%ignore cvc::mol::molecule_view::colors;
%ignore cvc::mol::molecule_view::spheres;
%ignore cvc::mol::molecule_view::sticks;
%ignore cvc::mol::molecule_view::cartoon;
%ignore cvc::mol::molecule_view::bounds;
%ignore cvc::mol::molecule_view::warnings;
// Lists reach Python as tuples, copies (the members' own getters hand out views).
%naturalvar cvc::mol::molecule::atoms;
%naturalvar cvc::mol::molecule::structures;
%naturalvar cvc::mol::molecule_view::sphere_atoms;
%naturalvar cvc::mol::molecule_view::bonds;
%naturalvar cvc::mol::molecule_view::cartoon_atoms;
%naturalvar cvc::mol::group_tree::groups;
%naturalvar cvc::mol::blur_params::dims;
%naturalvar cvc::mol::view_params::b_range;

// pycvc.view_params is cvc::lod's.
%rename(molecule_view_params) cvc::mol::view_params;

%include "cvc/mol/molecule.h"
%include "cvc/mol/blur.h"
%include "cvc/mol/view.h"
%include "cvc/mol/groups.h"
%include "cvc/mol/torsions.h"
PYCVC_PARAM_TABLE(molecule_read_params, cvc::mol::molecule_read_params)
PYCVC_PARAM_TABLE(blur_params, cvc::mol::blur_params)
PYCVC_PARAM_TABLE(molecule_view_params, cvc::mol::view_params)

%extend cvc::mol::molecule {
  std::vector<double> bounds_list(bool with_radii = false) const {
    const std::array<double, 6> b = $self->bounds(with_radii);
    return std::vector<double>(b.begin(), b.end());
  }
  %pythoncode %{
    def bounds(self, with_radii=False):
        """The atoms' box, xmin ymin zmin xmax ymax zmax: of their centres, or with `with_radii`
        of their spheres. All zero for no atoms."""
        return self.bounds_list(with_radii)

    def __repr__(self):
        return "<molecule %s: %d atoms, %d structures>" % (self.id or "-", len(self.atoms),
                                                           len(self.structures))
  %}
}

%extend cvc::mol::group {
  std::vector<int> children_list() const {
    return std::vector<int>($self->children.begin(), $self->children.end());
  }
  std::vector<int> atoms_list() const {
    return std::vector<int>($self->atoms.begin(), $self->atoms.end());
  }
  %pythoncode %{
    children = property(children_list, doc="Indices into the tree's groups, in order.")
    atoms = property(atoms_list, doc="A residue's atoms: indices into the molecule's, in order.")
  %}
}

%extend cvc::mol::residue_torsions {
  std::vector<double> nu_list() const {
    return std::vector<double>($self->nu.begin(), $self->nu.end());
  }
  %pythoncode %{
    nu = property(nu_list, doc="A nucleotide's sugar ring torsions nu0 .. nu4 (radians).")
  %}
}

// The view's parts are copies Python owns.
%extend cvc::mol::molecule_view {
  cvc::geometry spheres_copy() const { return $self->spheres; }
  cvc::geometry sticks_copy() const { return $self->sticks; }
  cvc::geometry cartoon_copy() const { return $self->cartoon; }
  std::vector<double> centers_list() const {
    return std::vector<double>($self->centers.begin(), $self->centers.end());
  }
  std::vector<double> radii_list() const {
    return std::vector<double>($self->radii.begin(), $self->radii.end());
  }
  std::vector<int> colors_list() const {
    std::vector<int> out;
    out.reserve(3 * $self->colors.size());
    for (const cvc::mol::rgb8 &c : $self->colors)
      out.insert(out.end(), {int(c[0]), int(c[1]), int(c[2])});
    return out;
  }
  std::vector<double> bounds_list() const {
    return std::vector<double>($self->bounds.begin(), $self->bounds.end());
  }
  std::vector<std::string> warnings_list() const { return $self->warnings; }
  %pythoncode %{
    spheres = property(spheres_copy, doc="Triangles: the spheres tessellated (with sphere_mesh).")
    sticks = property(sticks_copy, doc="Triangles: the bonds' cylinders.")
    cartoon = property(cartoon_copy, doc="Triangles: the cartoon.")
    centers = property(centers_list, doc="The spheres' centres, a flat [x, y, z, ...] list.")
    radii = property(radii_list, doc="The spheres' radii.")
    colors = property(colors_list, doc="The spheres' colours, a flat [r, g, b, ...] list (0-255).")
    bounds = property(bounds_list, doc="The view's box, xmin ymin zmin xmax ymax zmax.")
    warnings = property(warnings_list)

    def __repr__(self):
        return "<molecule_view: %d atoms, %d bonds, %d residues>" % (
            self.atoms_drawn, len(self.bonds), self.residues)
  %}
}

// Files: IOError for one that can't be read or written, ValueError for parameters.
%define PYCVC_MOL_FILE(FN)
%exception FN {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::mol::molecule_file_error &e) {
    SWIG_exception(SWIG_IOError, e.what());
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%enddef
PYCVC_MOL_FILE(_read_molecule)
PYCVC_MOL_FILE(write_molecule)

// The long ones: the GIL released, bad parameters ValueError.
%define PYCVC_MOL_LONG(FN)
%exception FN {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%enddef
PYCVC_MOL_LONG(_build_view)
PYCVC_MOL_LONG(_blur)
PYCVC_MOL_LONG(cvc::mol::torsions)
PYCVC_MOL_LONG(cvc::mol::build_group_tree)
PYCVC_MOL_LONG(cvc::mol::find_bonds)
PYCVC_MOL_LONG(dihedral)

%inline %{
cvc::mol::molecule _read_molecule(std::shared_ptr<cvc::app> app, const std::string &path,
                                  const cvc::mol::molecule_read_params &p) {
  if (!app)
    throw std::invalid_argument("read_molecule: app is None");
  return cvc::mol::read_molecule(*app, path, p);
}
// pycvc.write_molecule(app, m, path): the format by the path's extension.
void write_molecule(std::shared_ptr<cvc::app> app, const cvc::mol::molecule &m,
                    const std::string &path) {
  if (!app)
    throw std::invalid_argument("write_molecule: app is None");
  cvc::mol::write_molecule(*app, m, path);
}
std::vector<std::string> molecule_read_extensions(std::shared_ptr<cvc::app> app) {
  return app ? cvc::mol::molecule_read_extensions(*app) : std::vector<std::string>();
}
std::vector<std::string> molecule_write_extensions(std::shared_ptr<cvc::app> app) {
  return app ? cvc::mol::molecule_write_extensions(*app) : std::vector<std::string>();
}
cvc::mol::molecule_view _build_view(std::shared_ptr<cvc::app> app, const cvc::mol::molecule &m,
                                    const cvc::mol::view_params &p) {
  if (!app)
    throw std::invalid_argument("build_view: app is None");
  return cvc::mol::build_view(*app, m, p);
}
cvc::volume _blur(std::shared_ptr<cvc::app> app, const cvc::mol::molecule &m,
                  const cvc::mol::blur_params &p) {
  if (!app)
    throw std::invalid_argument("blur: app is None");
  return cvc::mol::blur(*app, m, p);
}
// pycvc.flatten_atoms(tree): the atoms in TexMol's walk order (indices into the molecule's).
std::vector<int> pycvc_flatten_atoms(const cvc::mol::group_tree &t) {
  const std::vector<std::size_t> order = cvc::mol::flatten_atoms(t);
  return std::vector<int>(order.begin(), order.end());
}
// pycvc.dihedral(a, b, c, d): the torsion of four points, each [x, y, z] (radians, IUPAC's sign).
double dihedral(const std::vector<double> &a, const std::vector<double> &b,
                const std::vector<double> &c, const std::vector<double> &d) {
  if (a.size() != 3 || b.size() != 3 || c.size() != 3 || d.size() != 3)
    throw std::invalid_argument("dihedral: each point is [x, y, z]");
  return cvc::mol::dihedral({a[0], a[1], a[2]}, {b[0], b[1], b[2]}, {c[0], c[1], c[2]},
                            {d[0], d[1], d[2]});
}
%}

%pythoncode %{
def read_molecule(app, path, params=None, **keywords):
    """The molecule in a file (.pdb .ent .pdb1 .pqr .xyzr .pts): read_molecule(app, path,
    model=0, hydrogens=False, radii="solvent_enlarged", ...) or with a molecule_read_params."""
    return _read_molecule(app, path, _params_from(molecule_read_params, params, keywords,
                                                  "read_molecule"))


def build_view(app, molecule, params=None, **keywords):
    """What cvcGL's molecule node draws (cvc::mol::build_view): build_view(app, m,
    style="cartoon", color_by="structure", sphere_mesh=True, ...) or with a molecule_view_params."""
    return _build_view(app, molecule,
                       _params_from(molecule_view_params, params, keywords, "build_view"))


def blur(app, molecule, params=None, **keywords):
    """A molecule's density as a volume (cvc::mol::blur, TexMol's GOABlur): blur(app, m,
    resolution=2.0, blobbiness=-0.5) or with a blur_params."""
    return _blur(app, molecule, _params_from(blur_params, params, keywords, "blur"))
%}
