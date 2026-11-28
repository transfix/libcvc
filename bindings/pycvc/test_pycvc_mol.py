"""pycvc's molecules (pycvc_mol.i, pycvc_jobs.i): the TexMol ports' cvc::mol from Python (TexMol
plan M0 part 2, docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2): 1GRL read with keywords and its
facts, every writer's round trip, a file's IOError and a parameter's ValueError, a molecule built in
Python, the view, the group tree against TexMol's (tools/texmol-goldens), the torsions, the blur, the
GIL released during a call, and the jobs' steps, cancel and the app they keep alive."""

import gc
import math
import os
import tempfile
import threading

import pycvc

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.join(HERE, "..", "..")
GRL = os.path.join(REPO, "examples", "data", "molecules", "1grl.pdb")
GOLDENS = os.path.join(REPO, "src", "cvc", "tests", "data", "texmol-goldens")

app = pycvc.make_app()
grl = pycvc.read_molecule(app, GRL)
grl_atoms = grl.atoms  # a copy: read once


def raises(exc, fn, *args, **kw):
    try:
        fn(*args, **kw)
    except exc as e:
        return str(e)
    raise AssertionError("%s should raise %s" % (fn.__name__, exc.__name__))


def test_1grl_and_its_facts():
    assert grl.id == "1GRL" and grl.models == 0
    assert len(grl_atoms) == 4186
    assert sum(a.element == "H" for a in grl_atoms) == 757
    assert grl_atoms[0].residue == "VAL" and grl_atoms[-1].name == "H"
    assert all(a.chain == " " and a.model == 0 for a in grl_atoms[:50])
    helix = pycvc.secondary_structure.kind_helix
    sheet = pycvc.secondary_structure.kind_sheet
    kinds = [s.type for s in grl.structures]
    assert kinds.count(helix) == 19 and kinds.count(sheet) == 11
    first = grl.structures[0]
    assert (first.start_seq, first.end_seq) == (9, 30)
    assert first.contains(" ", 12) and not first.contains(" ", 31)
    assert grl.structure_of(next(a for a in grl_atoms if a.residue_seq == 20)) == 0
    b, br = grl.bounds(), grl.bounds(with_radii=True)
    assert len(b) == 6 and all(br[k] < b[k] and br[k + 3] > b[k + 3] for k in range(3))
    assert pycvc.chain_ids(grl) == " "
    assert "1GRL" in repr(grl) and "4186 atoms" in repr(grl)


def test_read_keywords_and_params():
    heavy = pycvc.read_molecule(app, GRL, hydrogens=False)
    assert len(heavy.atoms) == 4186 - 757
    p = pycvc.molecule_read_params(hydrogens=False)
    assert len(pycvc.read_molecule(app, GRL, p).atoms) == 4186 - 757
    # An enum by its name: the solvent-enlarged radii are larger, a hydrogen's 0.
    big = pycvc.read_molecule(app, GRL, radii="solvent_enlarged", hydrogens=False)
    assert sum(a.radius for a in big.atoms) > sum(a.radius for a in heavy.atoms)
    assert "model" in pycvc.molecule_read_params.fields()
    assert "hydrogens" in raises(TypeError, pycvc.read_molecule, app, GRL, no_such_field=1)
    raises(TypeError, pycvc.read_molecule, app, GRL, p, hydrogens=True)
    raises(ValueError, pycvc.read_molecule, app, GRL, radii="no_such_radii")


def test_every_writer_reads_back():
    assert list(pycvc.molecule_read_extensions(app)) == [".ent", ".pdb", ".pdb1", ".pqr", ".pts",
                                                         ".xyzr"]
    exts = pycvc.molecule_write_extensions(app)
    assert {".pdb", ".pqr", ".xyzr", ".pts"} <= set(exts), exts
    with tempfile.TemporaryDirectory() as d:
        for ext in (".pdb", ".pqr", ".xyzr", ".pts"):
            path = os.path.join(d, "grl" + ext)
            pycvc.write_molecule(app, grl, path)
            back = pycvc.read_molecule(app, path)
            atoms = back.atoms
            assert len(atoms) == len(grl_atoms), ext
            for a, b in zip(atoms[::97], grl_atoms[::97]):
                assert abs(a.x - b.x) < 1e-3 and abs(a.z - b.z) < 1e-3, ext
                assert abs(a.radius - b.radius) < 1e-3 or ext == ".pdb", ext
            if ext == ".pdb":
                assert back.id == "1GRL" and len(back.structures) == len(grl.structures)
                assert [a.name for a in atoms[:30]] == [a.name for a in grl_atoms[:30]]


def test_a_file_raises_ioerror():
    assert "can't open" in raises(IOError, pycvc.read_molecule, app, os.path.join(HERE, "none.pdb"))
    assert "no reader" in raises(IOError, pycvc.read_molecule, app, __file__)
    with tempfile.TemporaryDirectory() as d:
        assert "no writer" in raises(IOError, pycvc.write_molecule, app, grl,
                                     os.path.join(d, "grl.obj"))
    raises(ValueError, pycvc.read_molecule, None, GRL)


def atom(name, element, x, y=0.0, z=0.0, seq=1, residue="ALA"):
    a = pycvc.atom()
    a.serial, a.name, a.element, a.residue, a.residue_seq = seq * 10, name, element, residue, seq
    a.x, a.y, a.z, a.radius = x, y, z, 1.7
    return a


def test_a_molecule_built_in_python():
    m = pycvc.molecule()
    assert m.empty() and m.bounds() == (0.0,) * 6
    m.id = "MINE"
    m.atoms = [atom("C1", "C", 0.0), atom("C2", "C", 1.5), atom("C3", "C", 6.0)]
    assert len(m.atoms) == 3 and not m.empty()
    bonds = pycvc.find_bonds(m)
    assert [(b.a, b.b) for b in bonds] == [(0, 1)]
    assert len(pycvc.find_bonds(m, 5.0)) == 3  # a generous tolerance bonds all three
    assert abs(pycvc.covalent_radius("C") - 0.76) < 0.1 and pycvc.is_metal("Zn")
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "mine.pdb")
        pycvc.write_molecule(app, m, path)
        back = pycvc.read_molecule(app, path)
        assert back.id == "MINE" and [a.name for a in back.atoms] == ["C1", "C2", "C3"]


def test_the_view():
    v = pycvc.build_view(app, grl)  # ball and stick, by element
    assert v.atoms_drawn == 4186 and len(v.bonds) > 4000
    assert len(v.radii) == 4186 and len(v.centers) == 3 * 4186 and len(v.colors) == 3 * 4186
    assert len(v.sphere_atoms) == 4186 and all(0 <= c <= 255 for c in v.colors[:300])
    assert v.sticks.num_triangles() > 0 and v.spheres.num_triangles() == 0
    assert abs(v.radii[0] - 0.25 * grl_atoms[v.sphere_atoms[0]].radius) < 1e-5
    assert len(v.bounds) == 6 and v.chains == " " and v.warnings == ()
    assert "4186 atoms" in repr(v)
    mesh = pycvc.build_view(app, grl, sphere_mesh=True, hydrogens=False)
    assert mesh.atoms_drawn == 4186 - 757 and mesh.spheres.num_triangles() > 0
    cartoon = pycvc.build_view(app, grl, style="cartoon", color_by="structure")
    assert cartoon.cartoon.num_triangles() > 0 and len(cartoon.sphere_atoms) == 0
    assert cartoon.sticks.num_triangles() == 0
    assert cartoon.residues == len(pycvc.chain_trace(grl)) and len(cartoon.cartoon_atoms) > 0
    p = pycvc.molecule_view_params()
    p.set_text("style", "spacefill")
    assert p.style == pycvc.view_style_spacefill and p.get_text("style") == "spacefill"
    fill = pycvc.build_view(app, grl, p)
    assert abs(fill.radii[0] - grl_atoms[fill.sphere_atoms[0]].radius) < 1e-5
    b = pycvc.build_view(app, grl, color_by="b_factor", b_range=[10.0, 40.0])
    assert len(b.colors) == 3 * 4186
    assert "b_range" in raises(ValueError, pycvc.build_view, app, grl, b_range=[3.0])
    assert "atom_scale" in raises(ValueError, pycvc.build_view, app, grl, atom_scale=-1.0)
    raises(ValueError, pycvc.build_view, app, grl, style="no_such_style")


def tree_lines_of_texmol():
    """TexMol's own tree (parser_legacy), its residues' phi and psi left out as describe() does."""
    out = []
    with open(os.path.join(GOLDENS, "parser", "1grl.tree.txt")) as f:
        for line in f.read().splitlines():
            if "residue " in line:
                line = line.rsplit(" ", 2)[0]
            out.append(line)
    return out


def test_the_group_tree_is_texmols():
    t = pycvc.build_group_tree(grl)
    assert pycvc.describe(t).splitlines() == tree_lines_of_texmol()
    groups = t.groups
    assert groups[0].type == pycvc.group.kind_collection and groups[0].children == (1,)
    residues = [g for g in groups if g.type == pycvc.group.kind_residue]
    assert sum(len(g.atoms) for g in residues) == 4186
    assert residues[0].residue == "VAL" and residues[0].residue_seq == 6
    order = pycvc.flatten_atoms(t)
    assert sorted(order) == list(range(4186)) and list(order) != list(range(4186))
    with open(os.path.join(GOLDENS, "blur", "1grl.atoms.txt")) as f:
        legacy = [list(map(float, line.split()[:3])) for line in f.read().splitlines()]
    for i in range(0, 4186, 211):
        a = grl_atoms[order[i]]
        assert max(abs(a.x - legacy[i][0]), abs(a.y - legacy[i][1]),
                   abs(a.z - legacy[i][2])) < 1e-4, i


def test_the_torsions():
    rs = pycvc.torsions(grl)
    aa = pycvc.residue_torsions.kind_amino_acid
    # An amino acid has its N, CA and C: 1GRL's C-alpha-only stretch is "other".
    names = {}
    for a in grl_atoms:
        names.setdefault(a.residue_seq, set()).add(a.name)
    backbone = sum({"N", "CA", "C"} <= n for n in names.values())
    assert len(rs) == len(names) and sum(r.type == aa for r in rs) == backbone < len(rs)
    assert rs[0].residue == "VAL" and math.isnan(rs[0].phi)
    assert len(rs[0].nu) == 5 and all(math.isnan(v) for v in rs[0].nu)
    # Peptide bonds are trans where both neighbours are there: |omega| near pi.
    omegas = [r.omega for r in rs if not math.isnan(r.omega)]
    assert len(omegas) > 300
    assert sum(abs(abs(w) - math.pi) < 0.5 for w in omegas) > 0.95 * len(omegas)
    # A right angle, by IUPAC's sign.
    d = pycvc.dihedral([1, 0, 0], [0, 0, 0], [0, 0, 1], [0, 1, 1])
    assert abs(d - math.pi / 2) < 1e-12, d
    raises(ValueError, pycvc.dihedral, [1, 0], [0, 0, 0], [0, 0, 1], [0, 1, 1])


def test_the_blur():
    vol = pycvc.blur(app, grl, resolution=2.0, blobbiness=-0.5)
    dims = (vol.XDim(), vol.YDim(), vol.ZDim())
    assert all(n > 10 for n in dims), dims
    assert vol.max_value() > 0.5 and vol.min_value() >= 0.0
    fine = pycvc.blur(app, grl, pycvc.blur_params(resolution=1.5, blobbiness=-0.5))
    assert fine.XDim() > vol.XDim()
    raises(ValueError, pycvc.blur, app, grl, resolution=-1.0)


def test_the_gil_is_released():
    ticks = [0]
    stop = threading.Event()

    def count():
        while not stop.is_set():
            ticks[0] += 1

    t = threading.Thread(target=count)
    t.start()
    try:
        before = ticks[0]
        for _ in range(3):
            pycvc.build_view(app, grl, style="cartoon", subdivisions=16)
        during = ticks[0] - before
    finally:
        stop.set()
        t.join()
    assert during > 0


def test_the_jobs():
    job = pycvc.build_view_job(app, grl, style="cartoon")
    last, phases = 0.0, []
    while not job.step(64):
        assert job.progress() >= last
        last = job.progress()
        if not phases or phases[-1] != job.phase():
            phases.append(job.phase())
    assert "cartoon" in phases and job.phase() == "done"
    v = job.result()
    ref = pycvc.build_view(app, grl, style="cartoon")
    assert v.cartoon.num_triangles() == ref.cartoon.num_triangles()
    assert v.residues == ref.residues

    job = pycvc.blur_job(app, grl, resolution=2.0, blobbiness=-0.5)
    job.step(1)
    job.cancel()
    assert job.step(1) and job.cancelled() and not job.has_result()

    rs = pycvc.torsions_job(grl).run(budget=32)
    assert len(rs) == len(pycvc.torsions(grl))
    t = pycvc.group_tree_job(grl).run(budget=256)
    assert pycvc.describe(t) == pycvc.describe(pycvc.build_group_tree(grl))

    # A job keeps its app: none other holds it here.
    def job_of_its_own_app():
        own = pycvc.make_app()
        return pycvc.blur_job(own, grl, resolution=3.0, blobbiness=-0.5)

    job = job_of_its_own_app()
    gc.collect()
    vol = job.run()
    assert vol.XDim() > 5


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
