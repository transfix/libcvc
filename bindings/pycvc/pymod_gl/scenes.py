"""Load real-world scene geometry (terrain heightfields + glTF city meshes) into
a :class:`pycvc_gl.lab.Lab`.

These are generic loaders for a ``geometry_bundle`` export — a ``terrain.json``
heightfield plus a ``buildings.glb`` (glTF 2.0) city mesh, as produced by an
OpenStreetMap + SRTM scene generator (e.g. the Austin scene bundle the nav demos
load, ``scene-austin-south-small`` on cvcpkg). The glTF is loaded
NATIVELY via libcvc (``pycvc.load_model`` → ``cvc::model``, the Assimp-backed
mesh loader — no ``vtkGLTFReader``, no trimesh/pygltflib) and added as a single
native geometry node; the terrain becomes a draped surface mesh; a
``sampler`` lets you drape an agent onto the terrain. The terrain (and, in
``load_geometry_bundle``, the buildings) come from the C++ loader
``pycvc.load_city_bundle``, so Python draws and samples the same height field the
physics world collides with and the navigator plans over. (``building_occupancy``
below still rasterizes the mesh through VTK offscreen — a separate concern.)

ATTRIBUTION: bundles generated from OpenStreetMap are © OpenStreetMap
contributors and licensed under the Open Database License (ODbL,
https://openstreetmap.org/copyright); SRTM terrain is US public domain. If you
publish renders, credit OpenStreetMap. Do NOT ship the Esri ``satellite.png``
overlay some bundles carry — it is proprietary; these loaders never touch it.
"""

from __future__ import annotations

import os


def _bundle_dir(path: str) -> str:
    """A bundle directory from either the directory or its ``terrain.json``."""
    return os.path.dirname(path) if os.path.basename(path) == "terrain.json" else path


def load_terrain(path: str):
    """The bundle's terrain as a ``pycvc.height_field``, read by the C++ loader
    (``cvc::world::load_city_bundle``) that the renderer, the physics world and the
    navigator use, so Python sees the same samples and the same triangles.

    ``path`` is the bundle directory or its ``terrain.json``. The stored grid runs
    north first; the field has row 0 at ``min_y`` (flipped once, in C++), so
    ``field.height(x, y)`` agrees with the glTF's world frame. Replaces the old
    ``terrain_grid``, which parsed ``terrain.json`` in Python."""
    import pycvc

    return pycvc.load_city_bundle(_bundle_dir(path), buildings=False).terrain()


def terrain_sampler(path: str = "", field=None):
    """A height function ``h(x, y)`` over the bundle's terrain, for DRAPING an
    agent or path onto it. Clamped to the grid at the edges.

    It samples ``pycvc.height_field.height``: the plane of the mesh triangle under
    the point, the surface the renderer draws and the physics engine collides with
    (the old sampler was bilinear, which differs inside a cell). Pass ``field`` to
    sample a height field already loaded."""
    if field is None:
        field = load_terrain(path)
    min_x, min_y, max_x, max_y = field.bounds2d()

    def h(x: float, y: float) -> float:
        x = min_x if x < min_x else (max_x if x > max_x else x)
        y = min_y if y < min_y else (max_y if y > max_y else y)
        return field.height(x, y)

    return h


def add_terrain_json(lab, path: str, name: str = "terrain", color=(0.34, 0.40, 0.28)):
    """Add a bundle's terrain to ``lab`` as a draped surface (the height field's own
    mesh); returns a ``sampler`` ``h(x, y)`` for the SAME field (so agents drape onto
    it). ``path`` is the bundle directory or its ``terrain.json``."""
    field = load_terrain(path)
    lab.add_height_field(name, field, color=color)
    return terrain_sampler(field=field)


def add_gltf(
    lab, path: str, name: str, color=(0.74, 0.74, 0.78), opacity: float = 1.0, parent: str = ""
):
    """Load a glTF/GLB (or any Assimp-supported) mesh NATIVELY via libcvc and add it
    to ``lab`` as one named ``GeometryNode``. ``parent`` (default the root) makes it a
    CHILD of that node so it stays aligned to and moves with it (e.g. buildings under
    the terrain). Returns the live ``GeometryNode``.

    The whole file is flattened to a single ``cvc::geometry``
    (``pycvc.load_model(path).merged()``) and rendered with a UNIFORM
    ``color``/``opacity`` — the demo deliberately renders a single-color city, not
    per-glTF materials — matching the previous VTK path's look (ambient 0.45 /
    diffuse 0.7 / specular 0.05). Imports NO VTK glTF reader; the scene node itself
    still renders through cvcGL/VTK as usual."""
    # Native load: one flattened geometry (single-color, one-node city mesh).
    return _add_geometry_node(
        lab, name, lab._pycvc.load_model(path).merged(), color, opacity, parent
    )


def _add_geometry_node(lab, name: str, g, color, opacity: float = 1.0, parent: str = ""):
    """Add geometry ``g`` as one uniformly coloured ``GeometryNode`` (see
    :func:`add_gltf`), a child of ``parent`` when given."""
    # Add via the native scene path, honoring parent (child inherits its transform).
    scene = lab._scene
    if parent:
        node = scene.add_child_geometry(parent, name, g)
    else:
        node = scene.addGraphics(name, g)  # downcast to the GeometryNode proxy

    # Uniform single-color material through the actor property (per-vertex colors go
    # through VTK's LUT and mangle channels — see Lab.recolor). Ground-level views
    # leave many building faces facing away from the light; a strong ambient term
    # keeps them from going black so the city reads clearly.
    node.setUseSingleColor(True)
    node.setColor(*[float(c) for c in color])
    node.setOpacity(float(opacity))
    node.setAmbient(0.45)
    node.setDiffuse(0.7)
    node.setSpecular(0.05)
    return node


# ── grounded routing: keep a vehicle ON THE STREETS, out of the buildings ────
#
# The scene is real: a vehicle should DRIVE the streets, not fly over rooftops or
# clip through walls. We rasterize the city footprint into a boolean occupancy
# grid, then A*-route a grounded path through the free space (streets) between
# waypoints and drape it on the terrain. All pure-numpy/heapq — no extra deps.


def building_occupancy(
    glb_path: str,
    bounds2d,
    nx: int = 512,
    ny: int = 512,
    inflate_m: float = 10.0,
    cache: bool = True,
):
    """Rasterize ``buildings.glb`` into a SOLID boolean occupancy grid (``True`` =
    inside a building footprint) over ``bounds2d`` = ``(min_x, min_y, max_x, max_y)``.

    ``occ[r][c]``: ``r`` -> y (row 0 = ``min_y``), ``c`` -> x — the same bottom-up
    world frame the terrain mesh/sampler use. Footprints are grown by ``inflate_m``
    (a clearance halo) so a routed path keeps a vehicle's width off the walls.

    The mask is produced by rendering the city mesh **top-down orthographically**
    and thresholding: VTK's polygon rasterizer fills every triangle, so building
    INTERIORS are solid — unlike marking only vertex cells (which leaves interiors
    and sparse-wall gaps a path could thread straight through). Needs an offscreen
    GL context (present wherever the scene renders).

    The mask is cached next to ``glb_path`` (``<glb>.occ_<nx>x<ny>_i<inflate>.npy``)
    keyed by the grid params, so a demo loads it instantly on later runs instead of
    re-rasterizing (and can precompute it out-of-process to avoid opening a second
    GL context inside a host app). Pass ``cache=False`` to always recompute.
    """
    import os

    import numpy as np

    cache_path = "%s.occ_%dx%d_i%d.npy" % (glb_path, nx, ny, int(round(inflate_m)))
    if (
        cache
        and os.path.exists(cache_path)
        and os.path.getmtime(cache_path) >= os.path.getmtime(glb_path)
    ):
        return np.load(cache_path)

    # Register VTK's OpenGL2 render factory — in a raw Python interpreter (unlike the
    # C++ VTK_MODULE_INIT path) vtkRenderWindow() has no GL override until this import
    # runs, and offscreen Render() otherwise crashes.
    import vtkmodules.vtkRenderingOpenGL2  # noqa: F401
    from vtkmodules.vtkIOGeometry import vtkGLTFReader
    from vtkmodules.vtkFiltersGeometry import vtkCompositeDataGeometryFilter
    from vtkmodules.vtkRenderingCore import (
        vtkActor,
        vtkPolyDataMapper,
        vtkRenderer,
        vtkRenderWindow,
        vtkWindowToImageFilter,
    )
    from vtkmodules.util.numpy_support import vtk_to_numpy

    min_x, min_y, max_x, max_y = bounds2d
    reader = vtkGLTFReader()
    reader.SetFileName(glb_path)
    reader.Update()
    gf = vtkCompositeDataGeometryFilter()
    gf.SetInputConnection(reader.GetOutputPort())
    gf.Update()

    mapper = vtkPolyDataMapper()
    mapper.SetInputData(gf.GetOutput())
    mapper.ScalarVisibilityOff()
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1.0, 1.0, 1.0)
    actor.GetProperty().LightingOff()  # flat white footprint on black — a clean mask

    ren = vtkRenderer()
    ren.SetBackground(0.0, 0.0, 0.0)
    ren.AddActor(actor)
    win = vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.AddRenderer(ren)
    win.SetSize(nx, ny)

    cam = ren.GetActiveCamera()
    cam.ParallelProjectionOn()  # orthographic: pixel<->world is a pure linear map
    cx, cy = 0.5 * (min_x + max_x), 0.5 * (min_y + max_y)
    cam.SetFocalPoint(cx, cy, 0.0)
    cam.SetPosition(cx, cy, 10000.0)  # straight above, looking down -Z
    cam.SetViewUp(0.0, 1.0, 0.0)  # world +Y = image up (north-up map)
    cam.SetParallelScale(0.5 * (max_y - min_y))  # half the world height fills the viewport
    cam.SetClippingRange(1.0, 20000.0)
    win.Render()

    w2i = vtkWindowToImageFilter()
    w2i.SetInput(win)
    w2i.Update()
    img = w2i.GetOutput()
    arr = vtk_to_numpy(img.GetPointData().GetScalars()).reshape(ny, nx, -1)
    # VTK image origin is bottom-left: row 0 = bottom = min_y, col 0 = left = min_x —
    # exactly our occ[r->y][c->x] convention, no flip needed.
    occ = arr[:, :, 0] > 127
    win.Finalize()

    cell = min((max_x - min_x) / nx, (max_y - min_y) / ny)
    k = int(round(inflate_m / cell)) if cell > 0 else 0
    for _ in range(k):  # 4-connected binary dilation, k steps ~= inflate_m of clearance
        d = occ.copy()
        d[1:, :] |= occ[:-1, :]
        d[:-1, :] |= occ[1:, :]
        d[:, 1:] |= occ[:, :-1]
        d[:, :-1] |= occ[:, 1:]
        occ = d

    if cache:
        try:
            np.save(cache_path, occ)
        except OSError:  # read-only bundle dir — just skip caching
            pass
    return occ


def _nearest_free(occ, rc):
    """Nearest ``(r, c)`` with ``occ`` False, spiralling out from ``rc``."""
    ny, nx = occ.shape
    r0, c0 = rc
    if not occ[r0, c0]:
        return rc
    for rad in range(1, max(nx, ny)):
        for dr in range(-rad, rad + 1):
            edge = range(-rad, rad + 1) if abs(dr) == rad else (-rad, rad)
            for dc in edge:
                r, c = r0 + dr, c0 + dc
                if 0 <= r < ny and 0 <= c < nx and not occ[r, c]:
                    return (r, c)
    return rc


def _line_of_sight(occ, a, b):
    """True if the straight cell line ``a``->``b`` crosses no blocked cell (Bresenham)."""
    r0, c0 = a
    r1, c1 = b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        if occ[r, c]:
            return False
        if (r, c) == (r1, c1):
            return True
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def _astar(occ, start, goal):
    """8-connected grid A* over free cells; returns a list of ``(r, c)`` or ``None``."""
    import heapq

    ny, nx = occ.shape

    def h(a):
        return ((a[0] - goal[0]) ** 2 + (a[1] - goal[1]) ** 2) ** 0.5

    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    openq = [(h(start), 0.0, start)]
    came = {}
    gcost = {start: 0.0}
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur == goal:
            break
        if gc > gcost.get(cur, float("inf")):
            continue
        for dr, dc in nbrs:
            nr, ncl = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < ny and 0 <= ncl < nx) or occ[nr, ncl]:
                continue
            if dr and dc and (occ[cur[0] + dr, cur[1]] or occ[cur[0], cur[1] + dc]):
                continue  # don't cut a diagonal across a blocked building corner
            step = 1.41421356 if dr and dc else 1.0
            ng = gc + step
            if ng < gcost.get((nr, ncl), float("inf")):
                gcost[(nr, ncl)] = ng
                came[(nr, ncl)] = cur
                heapq.heappush(openq, (ng + h((nr, ncl)), ng, (nr, ncl)))
    if goal not in came and goal != start:
        return None
    path = [goal]
    c = goal
    while c != start:
        c = came[c]
        path.append(c)
    path.reverse()
    return path


def _simplify(occ, path):
    """String-pull a grid path: keep only corners needed to preserve line-of-sight,
    so the route is straight street runs instead of a staircase."""
    if len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_of_sight(occ, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan_ground_route(occ, bounds2d, waypoints_xy, close_loop: bool = True):
    """Plan a GROUNDED route (world ``(x, y)`` list) that visits ``waypoints_xy`` in
    order through the free space of ``occ`` (streets), optionally closing the loop.

    Each waypoint is snapped to the nearest free cell; legs are A*-routed and
    line-of-sight simplified. Drape the result on the terrain to drive it. Returns
    ``[]`` if no leg is routable.
    """
    ny, nx = occ.shape
    min_x, min_y, max_x, max_y = bounds2d

    def to_cell(x, y):
        c = int(round((x - min_x) / (max_x - min_x) * (nx - 1)))
        r = int(round((y - min_y) / (max_y - min_y) * (ny - 1)))
        return (max(0, min(ny - 1, r)), max(0, min(nx - 1, c)))

    def to_world(rc):
        r, c = rc
        return (min_x + c / (nx - 1) * (max_x - min_x), min_y + r / (ny - 1) * (max_y - min_y))

    wps = list(waypoints_xy)
    if close_loop and wps:
        wps = wps + [wps[0]]
    cells = [_nearest_free(occ, to_cell(x, y)) for x, y in wps]
    full = []
    for a, b in zip(cells, cells[1:]):
        seg = _astar(occ, a, b)
        if not seg:
            continue
        seg = _simplify(occ, seg)
        if full:
            seg = seg[1:]  # drop the duplicated junction cell
        full += seg
    return [to_world(rc) for rc in full]


def resample_polyline(points_xy, spacing: float):
    """Resample a 2-D polyline to roughly uniform ``spacing`` (world units) so a
    draped route animates at constant ground speed and the chase-cam heading is
    smooth. Returns a list of ``(x, y)``."""
    if len(points_xy) < 2:
        return list(points_xy)
    out = [tuple(points_xy[0])]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(points_xy, points_xy[1:]):
        seg = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        if seg <= 1e-9:
            continue
        d = carry
        while d < seg:
            t = d / seg
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
            d += spacing
        carry = d - seg
    out.append(tuple(points_xy[-1]))
    return out


def load_geometry_bundle(
    lab,
    bundle_dir: str,
    terrain_color=(0.34, 0.40, 0.28),
    building_color=(0.74, 0.74, 0.78),
    buildings: bool = True,
):
    """Load a ``geometry_bundle`` directory (``terrain.json`` + ``buildings.glb``)
    into ``lab``. Returns the terrain ``sampler`` ``h(x, y)`` for draping agents.

    ``buildings=False`` loads only the terrain (much lighter). The Esri
    ``satellite.png`` some bundles carry is intentionally NOT loaded (proprietary).
    """
    import pycvc

    # One load through the C++ loader: the terrain and the buildings exactly as
    # the physics world and the navigator read them (z up, not lifted).
    city = pycvc.load_city_bundle(bundle_dir, buildings=buildings)
    field = city.terrain()
    lab.add_height_field("terrain", field, color=terrain_color)
    if buildings and city.buildings_loaded:
        # buildings are CHILDREN of the terrain node so they stay aligned to it
        # (same world frame) and move with any terrain transform.
        _add_geometry_node(
            lab,
            "buildings",
            city.world().item_geometry("buildings"),
            building_color,
            parent="terrain",
        )
    return terrain_sampler(field=field)
