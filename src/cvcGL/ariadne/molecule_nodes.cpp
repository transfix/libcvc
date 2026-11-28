/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The `molecule` scene node type (cvc/gl/ariadne/molecule_nodes.h).

#include <boost/any.hpp>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/core/job_keys.h>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/core/sliced_task.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/MoleculeNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/molecule_nodes.h>
#include <cvc/gl/ariadne/node_data.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/view.h>
#include <cvc/state/followed_key.h>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;
using cvc::mol::molecule;
using cvc::mol::molecule_view;
using cvc::mol::view_params;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

void put(cvc::state &s, const std::string &v) {
  if (s.value() != v)
    s.value(v);
}

// A prop as the text its key takes: a scalar as written, a list as [a, b].
std::string prop_text(const ari::Value &v) {
  if (v.is_scalar())
    return v.scalar;
  if (v.is_seq()) {
    std::string s = "[";
    for (std::size_t i = 0; i < v.items.size(); ++i)
      s += (i ? ", " : "") + prop_text(v.items[i]);
    return s + "]";
  }
  return std::string();
}

// The molecule a payload holds: a shared_ptr<const molecule>, a shared_ptr<molecule> or a molecule.
std::shared_ptr<const molecule> molecule_payload(const boost::any &d) {
  if (const auto *c = boost::any_cast<std::shared_ptr<const molecule>>(&d))
    return *c;
  if (const auto *m = boost::any_cast<std::shared_ptr<molecule>>(&d))
    return *m;
  if (const auto *v = boost::any_cast<molecule>(&d))
    return std::make_shared<const molecule>(*v);
  return nullptr;
}

std::string num(double d) {
  char b[64];
  std::snprintf(b, sizeof b, "%.4g", d);
  return b;
}

// A view's parameters as one string: what tells a change.
std::string signature(const view_params &p) {
  std::string s;
  cvc::for_each_param<view_params>([&](const auto &f) {
    s += f.name;
    s += '=';
    s += cvc::param_detail::value_text(p.*(f.member));
    s += ';';
  });
  s += "hidden=" + p.hidden_chains + ";mesh=" + (p.sphere_mesh ? "1" : "0");
  return s;
}

// What the node follows, builds and draws, ticked on the frame thread.
struct live {
  cvc::app *app = nullptr;
  std::weak_ptr<MoleculeNode> node;
  std::string scope; // the document's: source.molecule is relative to it
  cvc::followed_key followed;
  std::shared_ptr<const molecule> mol;
  std::string chain_ids;
  std::string status_said;
  bool said = false;
  // The build: the job running (and its task), what it was asked for, and what is drawn.
  std::shared_ptr<cvc::job_keys> keys;
  std::shared_ptr<cvc::result_job<molecule_view>> job;
  int pid = -1;
  std::string owner; // the scheduler owner tag of its builds
  std::string running_sig, drawn_sig;
  bool running_bonds = false; // the build running finds bonds (ball_and_stick, sticks)
  std::shared_ptr<const molecule> running_mol, drawn_mol;
  bool fallback = false; // impostors failed here: tessellated spheres from now on
  std::string params_error;
  // frame: (first or always) frames the main view's camera on what it draws, at this key.
  std::string frame, frame_key;
  bool framed = false;

  void status(MoleculeNode &n, const std::string &s) {
    if (said && s == status_said)
      return;
    status_said = s;
    said = true;
    put(n.getState("source.status"), s);
  }

  // A new molecule: its chains as a list, a visible key each (kept for a chain the last one had).
  void adopt(MoleculeNode &n, std::shared_ptr<const molecule> m) {
    mol = std::move(m);
    n.getState("molecule").data(boost::any(mol));
    const std::string ids = cvc::mol::chain_ids(*mol);
    cvc::state &chains = n.getState("chains");
    std::vector<std::string> visible(ids.size(), "1");
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const std::size_t old = chain_ids.find(ids[i]);
      if (old != std::string::npos) {
        const std::string v = n.getState("chains." + std::to_string(old) + ".visible").value();
        if (!v.empty())
          visible[i] = v;
      }
    }
    std::vector<std::size_t> atoms(ids.size(), 0);
    for (const cvc::mol::atom &a : mol->atoms)
      ++atoms[ids.find(a.chain)];
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const std::string at = "chains." + std::to_string(i);
      put(n.getState(at + ".id"), ids[i] == ' ' ? std::string() : std::string(1, ids[i]));
      put(n.getState(at + ".label"),
          ids[i] == ' ' ? std::string("(none)") : std::string(1, ids[i]));
      put(n.getState(at + ".visible"), visible[i]);
      put(n.getState(at + ".atoms"), std::to_string(atoms[i]));
      const cvc::mol::rgb8 c = cvc::mol::chain_color(i);
      put(n.getState(at + ".color"),
          num(c[0] / 255.0) + "," + num(c[1] / 255.0) + "," + num(c[2] / 255.0));
    }
    // A shorter list than before: the rows past it go.
    for (std::size_t i = ids.size(); i < chain_ids.size(); ++i)
      if (cvc::state *s = chains.findThroughLinks(std::to_string(i)))
        s->reset();
    put(chains, std::to_string(ids.size()));
    chain_ids = ids;
    std::size_t helices = 0, sheets = 0;
    for (const cvc::mol::secondary_structure &s : mol->structures) {
      if (s.type == cvc::mol::secondary_structure::kind::helix)
        ++helices;
      else if (s.type == cvc::mol::secondary_structure::kind::sheet)
        ++sheets;
    }
    put(n.getState("info.atoms"), std::to_string(mol->atoms.size()));
    put(n.getState("info.helices"), std::to_string(helices));
    put(n.getState("info.sheets"), std::to_string(sheets));
    put(n.getState("info.id"), mol->id);
    put(n.getState("info.title"), mol->title);
  }

  void follow(MoleculeNode &n) {
    const std::string key = n.getState("source.molecule").value();
    const std::string path = ari::resolve_bind(scope, key);
    boost::any d;
    if (path.empty()) {
      status(n, "no molecule: source.molecule names none");
      return;
    }
    if (path == n.stateName("molecule")) {
      status(n, "error: source.molecule names this node's own molecule");
      return;
    }
    if (followed.changed(n.app().root(), path, d, cvc::any_pointee<const molecule, molecule>)) {
      std::shared_ptr<const molecule> m = molecule_payload(d);
      if (m) {
        if (m != mol)
          adopt(n, std::move(m));
        status(n, std::string());
      } else if (!d.empty()) {
        status(n, "error: " + key + " holds no molecule");
      }
    }
    if (!mol && status_said.empty()) // not there yet: a dataset loading, a program not run
      status(n, "waiting for a molecule at " + key);
  }

  // The parameters from the node's keys, the chains hidden from their visible keys.
  bool read_params(MoleculeNode &n, view_params &p, std::string &mode) {
    cvc::param_result<view_params> r = cvc::params_from_state<view_params>(n.getState());
    std::string err;
    for (const std::string &e : r.errors)
      err += (err.empty() ? "" : "; ") + e;
    if (err != params_error) {
      params_error = err;
      if (!err.empty())
        put(n.getState("status"), "error: " + err);
    }
    if (!err.empty())
      return false;
    p = r.value;
    for (std::size_t i = 0; i < chain_ids.size(); ++i) {
      const std::string v = n.getState("chains." + std::to_string(i) + ".visible").value();
      if (v == "0" || v == "false" || v == "no" || v == "off")
        p.hidden_chains.push_back(chain_ids[i]);
    }
    mode = n.getState("spheres").value();
    if (mode != "impostor" && mode != "mesh")
      mode = "auto";
    p.sphere_mesh = mode == "mesh" || (mode == "auto" && fallback);
    return true;
  }

  void start(MoleculeNode &n, const view_params &p, const std::string &sig) {
    if (pid >= 0)
      cvc::cancel_sliced_job(*app, pid); // a newer build replaces it; what is drawn stays
    job = cvc::mol::build_view_job(*app, mol, p);
    running_sig = sig;
    running_bonds =
        p.style == cvc::mol::view_style::ball_and_stick || p.style == cvc::mol::view_style::sticks;
    running_mol = mol;
    keys->started("building...");
    cvc::sliced_task_options o;
    o.kind = "molecule";
    o.subject = n.getName();
    o.owner = owner;
    std::string why;
    pid = cvc::schedule_sliced_job(*app, job, o, &why);
    if (pid < 0) {
      job.reset();
      keys->finished("error: the build could not start: " + why);
    } else {
      keys->pid(pid);
    }
  }

  // The running build: its progress, and once done what it built drawn.
  void poll(MoleculeNode &n) {
    if (!job)
      return;
    keys->progress(*job);
    if (!job->done())
      return;
    std::shared_ptr<cvc::result_job<molecule_view>> j = std::move(job);
    pid = -1;
    if (j->failed()) {
      keys->finished(*j);
      drawn_sig = running_sig; // not again until something changes
      drawn_mol = running_mol;
    } else if (j->has_result()) {
      auto v = std::make_shared<const molecule_view>(j->take_result());
      drawn_sig = running_sig;
      drawn_mol = running_mol;
      put(n.getState("info.drawn"), std::to_string(v->atoms_drawn));
      put(n.getState("info.bonds"), running_bonds ? std::to_string(v->bonds.size()) : "-");
      put(n.getState("info.residues"), std::to_string(v->residues));
      // The bounds as a camera's frame request takes them, so a program can frame any view.
      const bool drew = v->bounds[3] > v->bounds[0] || v->bounds[4] > v->bounds[1];
      std::string box;
      for (int k = 0; k < 6; ++k)
        box += (k ? " " : "") + cvc::param_detail::format_double(v->bounds[k]);
      put(n.getState("info.bounds"), drew ? box : std::string());
      if (drew && !frame_key.empty() && (frame == "always" || !framed)) {
        cvc::state::instance (*app)(frame_key).value(box);
        framed = true;
      }
      n.setView(running_mol, std::move(v));
      keys->finished(*j);
    } else {
      keys->finished(std::string("cancelled"));
    }
  }

  void tick() {
    const std::shared_ptr<MoleculeNode> n = node.lock();
    if (!n)
      return;
    follow(*n);
    poll(*n);
    // Impostors that would not build here: tessellated spheres in their place, said once.
    if (!fallback && n->impostorsFailed()) {
      fallback = true;
      put(n->getState("status"),
          "impostor spheres can't be drawn here; drawing tessellated spheres");
    }
    view_params p;
    std::string mode;
    if (!mol || !read_params(*n, p, mode))
      return;
    const bool mesh = p.sphere_mesh;
    n->setSpherePath(mesh ? MoleculeNode::SpherePath::Mesh : MoleculeNode::SpherePath::Impostor);
    const std::string sig = signature(p);
    const bool want = sig != drawn_sig || mol != drawn_mol;
    if (want && !(job && sig == running_sig && mol == running_mol))
      start(*n, p, sig);
    const bool spheres = n->view() && !n->view()->radii.empty();
    put(n->getState("info.spheres"), !spheres ? "" : (mesh ? "mesh" : "impostor"));
  }

  // Whether what it draws will still change: a build running, or a molecule not built yet.
  bool pending() const {
    if (!node.lock())
      return false;
    return job != nullptr || (mol && mol != drawn_mol && params_error.empty());
  }

  ~live() {
    if (pid >= 0 && app)
      cvc::cancel_sliced_job(*app, pid);
  }
};

std::shared_ptr<GraphicsNode> realize_molecule(SceneGraph &sg, const ari::SceneNode &n,
                                               GraphicsNode *parent, RealizedScene &out,
                                               std::vector<std::string> *warnings) {
  cvc::app &app = sg.appContext();
  const NodeDataSource source = node_data_source(sg, out, n, "molecule", "molecule");
  if (!source) {
    if (!n.source_file.empty())
      warn(warnings, "ari: scene node '" + n.id +
                         "' (molecule) reads no file: load it as app data (data: { " + n.id + ": " +
                         n.source_file + " }) and give source: { data: " + n.id + " }");
    else
      warn(warnings, "ari: scene node '" + n.id +
                         "' (molecule) has no source: give source: { data: <name> }, "
                         "{ state: <key> } or { node: <id> }");
  }

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id))
    sg.removeGraphics(n.id);
  std::shared_ptr<MoleculeNode> m = pr->addGraphicsChild<MoleculeNode>(n.id);
  if (!m)
    return nullptr;
  if (!parent)
    sg.registerGraphics(n.id, m);
  if (n.has_material)
    m->setLighting(n.ambient, n.diffuse, n.has_specular ? n.specular : 0.25,
                   n.has_specular ? n.specular_power : 24.0);

  // A prop seeds its key only while the key is empty (a restored session's value stays).
  const auto seed = [](cvc::state &key, const std::string &text) {
    if (key.value().empty() && !text.empty())
      key.value(text);
  };
  seed(m->getState("source.molecule"), source.path);
  cvc::for_each_param<view_params>([&](const auto &f) {
    if (const ari::Value *v = n.props.find(f.name))
      seed(m->getState(f.name), prop_text(*v));
  });
  const std::string spheres = n.props.str("spheres", "auto");
  if (spheres != "auto" && spheres != "impostor" && spheres != "mesh")
    warn(warnings, "ari: scene node '" + n.id + "': spheres: '" + spheres +
                       "' is not auto, impostor or mesh (auto is used)");
  seed(m->getState("spheres"), spheres);
  // The parameters' keys now, with their properties, so a component labels its widgets by them.
  cvc::param_result<view_params> r = cvc::params_from_state<view_params>(m->getState());
  for (const std::string &e : r.errors)
    warn(warnings, "ari: scene node '" + n.id + "' (molecule): " + e);

  // The parts are the node's for picks and views: a pick on its sticks or cartoon is a pick on
  // it, and its tags hold for them.
  m->setView(nullptr, nullptr); // makes the parts
  for (const std::shared_ptr<GeometryNode> &part : {m->spheresMesh(), m->sticks(), m->cartoon()})
    if (part)
      out.nodes.push_back({n.id, n.tags, part, true, n.on_pick});

  auto l = std::make_shared<live>();
  l->app = &app;
  l->node = m;
  l->scope = out.bind_prefix;
  l->owner = "molecule:" + m->stateName();
  l->frame = n.frame;
  if (!n.frame.empty()) // the main view's camera (CameraController::viewerStatePath)
    l->frame_key = ari::resolve_bind(out.bind_prefix, "viewers.main.camera.frame");
  l->keys = std::make_shared<cvc::job_keys>(cvc::state::instance(app), m->stateName());
  l->keys->idle();
  put(m->getState("status"), "idle");
  l->tick(); // a molecule already on the tree starts building now
  out.custom_ticks.push_back([l](vtkRenderer *) { l->tick(); });
  std::weak_ptr<live> wl = l;
  out.pending_work.push_back([wl]() {
    const std::shared_ptr<live> s = wl.lock();
    return s && s->pending();
  });
  wait_for_datasets(out, app, m, {source});
  return m;
}

} // namespace

void register_molecule_node_type(cvc::app &app) {
  NodeTypeOptions options;
  options.reads_data = true; // source: { data } is the molecule the node follows
  register_scene_node_type(app, "molecule", realize_molecule, options);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
