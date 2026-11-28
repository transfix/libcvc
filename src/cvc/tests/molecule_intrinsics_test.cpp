/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// cvc::mol in the DSL (cvc/mol/molecule_intrinsics.h), the T10 rows: mol.info at once;
// mol.torsions, mol.blur and mol.view in their three forms on the app's scheduler, cooperative and
// threaded, each giving what the C++ call gives; a bad parameter raising at the call;
// params.defaults; a -start job's keys and result; job.cancel part-way through; and the provider on
// an app, which another app does not see. Runs natively and under node (the ports lane).
#include <chrono>
#include <cmath>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/mol/blur.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/molecule_intrinsics.h>
#include <cvc/mol/torsions.h>
#include <cvc/mol/view.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/volume/volume.h>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>

namespace {
namespace se = cvc::state_exec;
namespace mol = cvc::mol;
using steady = std::chrono::steady_clock;

const std::string kRepo = CVC_REPO_DIR;

// A backend that clicks one button, to run an action in a document.
struct ClickBackend : cvc::ariadne::Backend {
  std::string click;
  cvc::ariadne::Capabilities capabilities() const override {
    cvc::ariadne::Capabilities c;
    c.windows = true;
    return c;
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const cvc::ariadne::Size &, float,
                    bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const cvc::ariadne::Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *) override {}
  void text_value(const char *, const std::string &) override {}
  void separator() override {}
  bool button(const char *l) override {
    if (click == l) {
      click.clear();
      return true;
    }
    return false;
  }
  bool menu_item_action(const char *) override { return false; }
  cvc::ariadne::BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  cvc::ariadne::BoolEdit checkbox(const char *, bool) override { return {}; }
  cvc::ariadne::IntEdit slider_int(const char *, const char *, int, int, int) override {
    return {};
  }
  cvc::ariadne::DoubleEdit slider_double(const char *, const char *, double, double, double,
                                         const char *) override {
    return {};
  }
  cvc::ariadne::IndexEdit combo(const char *, int, const std::vector<std::string> &) override {
    return {};
  }
  cvc::ariadne::StringEdit input_text(const char *, const char *, const std::string &,
                                      bool) override {
    return {};
  }
};

// Click Go in a one-button document on `app`, running `action`, and run frames until it has run.
void click_once(cvc::app &app, const std::string &action) {
  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button("Go", action)}));
  be.click = "Go";
  for (int i = 0; i < 20; ++i) {
    rt.render();
    rt.drain();
  }
}

class MoleculeIntrinsics : public ::testing::TestWithParam<cvc::sliced_threading> {
protected:
  void SetUp() override {
    se::async_scheduler &sched = app.exec_scheduler();
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.host = &app.exec_async_host();
    ictx.proc = se::make_process();
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    cvc::register_molecule_intrinsics(env, ictx, app);
    // The ops at this test's threading, over the defaults register_molecule_intrinsics gave.
    cvc::register_job_op(env, ictx, app, cvc::molecule_torsions_op(GetParam()));
    cvc::register_job_op(env, ictx, app, cvc::molecule_blur_op(GetParam()));
    cvc::register_job_op(env, ictx, app, cvc::molecule_view_op(GetParam()));
    const std::string f = kRepo + "/examples/data/molecules/1grl.pdb";
    std::ifstream in(f, std::ios::binary);
    m = std::make_shared<const mol::molecule>(mol::read_pdb(in, {}, f));
    app.root()("doc.mol").data(boost::any(m));
  }
  int run(const std::string &program) {
    se::execute_options eo;
    eo.env = env;
    return app.exec_scheduler().execute(program, eo);
  }
  template <class Pred> bool pump_until(Pred pred, double seconds = 120.0) {
    const auto t0 = steady::now();
    se::async_scheduler &sched = app.exec_scheduler();
    while (!pred()) {
      if (std::chrono::duration<double>(steady::now() - t0).count() > seconds)
        return false;
      sched.drain_ingress();
      sched.wake_awaiting();
      sched.sync_step();
      sched.sync_run(200000, 0.1);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
  }
  bool finished(int pid) {
    const auto info = app.exec_scheduler().get_process_info(pid);
    return !info || se::is_finished(info->status);
  }
  std::string exit_error(int pid) {
    const auto info = app.exec_scheduler().get_process_info(pid);
    return info ? info->exit_error.value_or(std::string()) : std::string("(reaped)");
  }
  std::string key(const std::string &k) { return app.root()(k).value(); }
  // What a program stored at `k` with state-data-set (a DSL value holding a data object), or a
  // job wrote as its result: the T inside, held either way.
  template <class T> std::shared_ptr<const T> data(const std::string &k) {
    boost::any a = app.root()(k).data();
    if (const se::value_t *v = boost::any_cast<se::value_t>(&a))
      if (const se::data_object_ptr *obj = std::get_if<se::data_object_ptr>(&v->v))
        a = (*obj)->payload;
    if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&a))
      return *p;
    if (const auto *t = boost::any_cast<T>(&a))
      return std::make_shared<const T>(*t);
    return nullptr;
  }

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
  std::shared_ptr<const mol::molecule> m;
};

} // namespace

TEST_P(MoleculeIntrinsics, InfoAtOnce) {
  const int pid = run("(let ((i (mol.info \"doc.mol\")))"
                      "  (state-set \"r.atoms\" (get-attr i \"atoms\"))"
                      "  (state-set \"r.chains\" (get-attr i \"chains\"))"
                      "  (state-set \"r.helices\" (get-attr i \"helices\"))"
                      "  (state-set \"r.id\" (get-attr i \"id\"))"
                      "  (state-set \"r.bounds\" (length (get-attr i \"bounds\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.atoms"), "4186");
  EXPECT_EQ(key("r.helices"), "19");
  EXPECT_EQ(key("r.id"), "1GRL");
  EXPECT_EQ(key("r.bounds"), "6");
  // Not a molecule: a TypeError-like failure naming the call.
  const int bad = run("(mol.info \"doc.nothing\")");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("mol.info"), std::string::npos) << exit_error(bad);
}

TEST_P(MoleculeIntrinsics, TorsionsWaitAndStart) {
  const std::vector<mol::residue_torsions> want = mol::torsions(*m);
  const int pid = run("(let ((ts (mol.torsions \"doc.mol\")))"
                      "  (state-set \"r.n\" (length ts))"
                      "  (state-set \"r.kind\" (get-attr (nth ts 1) \"kind\"))"
                      "  (state-set \"r.seq\" (get-attr (nth ts 1) \"seq\"))"
                      "  (state-set \"r.phi0\" (if (get-attr (nth ts 0) \"phi\") \"set\" \"nil\"))"
                      "  (state-set \"r.phi1\" (get-attr (nth ts 1) \"phi\")))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.n"), std::to_string(want.size()));
  EXPECT_EQ(key("r.kind"), "amino_acid");
  EXPECT_EQ(key("r.seq"), std::to_string(want[1].residue_seq));
  EXPECT_EQ(key("r.phi0"), "nil"); // the chain's first residue has no phi
  EXPECT_NEAR(std::stod(key("r.phi1")), want[1].phi, 1e-9);
  // -start: the keys under the target, and the result there once done.
  const int caller = run("(state-set \"r.pid\" (mol.torsions-start \"doc.mol\" \"job\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  EXPECT_FALSE(key("r.pid").empty());
  ASSERT_TRUE(pump_until([&] { return key("job.busy") == "0" && key("job.status") == "done"; }))
      << key("job.status");
  EXPECT_EQ(key("job.progress"), "100");
  const int reader = run("(state-set \"r.started\" (length (state-data-get \"job.result\")))");
  ASSERT_TRUE(pump_until([&] { return finished(reader); }));
  EXPECT_EQ(key("r.started"), std::to_string(want.size()));
}

TEST_P(MoleculeIntrinsics, BlurIsTheBlur) {
  mol::blur_params p;
  p.resolution = 2.0;
  p.blobbiness = -0.5;
  const cvc::volume want = mol::blur(app, *m, p);
  const int pid = run("(state-data-set \"r.vol\" (mol.blur \"doc.mol\" "
                      "(dict \"resolution\" 2.0 \"blobbiness\" -0.5)))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  const auto v = data<cvc::volume>("r.vol");
  ASSERT_TRUE(v);
  EXPECT_EQ(v->XDim(), want.XDim());
  EXPECT_EQ(v->YDim(), want.YDim());
  EXPECT_EQ(v->ZDim(), want.ZDim());
  EXPECT_EQ((*v)(5, 7, 9), want(5, 7, 9));
  // A bad parameter raises at the call.
  const int bad = run("(mol.blur \"doc.mol\" (dict \"blobbiness\" 2.0))");
  ASSERT_TRUE(pump_until([&] { return finished(bad); }));
  EXPECT_NE(exit_error(bad).find("blobbiness"), std::string::npos) << exit_error(bad);
  // The table's defaults.
  const int defs =
      run("(state-set \"r.b\" (get-attr (params.defaults \"mol.blur\") \"blobbiness\"))");
  ASSERT_TRUE(pump_until([&] { return finished(defs); }));
  EXPECT_NEAR(std::stod(key("r.b")), -2.3442, 1e-12);
}

TEST_P(MoleculeIntrinsics, ViewAsyncGivesTheMeshes) {
  mol::view_params p;
  p.style = mol::view_style::cartoon;
  const mol::molecule_view want = mol::build_view(app, m, p);
  const int pid = run("(let ((f (mol.view-async \"doc.mol\" (dict \"style\" \"cartoon\"))))"
                      "  (state-set \"r.between\" \"ran\")"
                      "  (let ((v (await f)))"
                      "    (state-set \"r.residues\" (get-attr v \"residues\"))"
                      "    (state-data-set \"r.cartoon\" (get-attr v \"cartoon\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(pid); }));
  EXPECT_EQ(exit_error(pid), "");
  EXPECT_EQ(key("r.between"), "ran");
  EXPECT_EQ(key("r.residues"), std::to_string(want.residues));
  const auto g = data<cvc::geometry>("r.cartoon");
  ASSERT_TRUE(g);
  EXPECT_EQ(g->tris().size(), want.cartoon.tris().size());
}

TEST_P(MoleculeIntrinsics, CancelPartWay) {
  // A big molecule: 1GRL eight times over, so the view's build spans many slices.
  mol::molecule big = *m;
  for (int k = 1; k < 8; ++k)
    for (mol::atom a : m->atoms) {
      a.x += 80.0 * k;
      a.model = 0;
      big.atoms.push_back(a);
    }
  app.root()("doc.big").data(boost::any(std::make_shared<const mol::molecule>(std::move(big))));
  const int caller = run("(state-set \"r.pid\" (mol.view-start \"doc.big\" "
                         "(dict \"sphere_mesh\" true \"sphere_detail\" 3) \"big\"))");
  ASSERT_TRUE(pump_until([&] { return finished(caller); }));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "1"; }));
  const int canceller = run("(state-set \"r.cancelled\" (job.cancel (int (state-get \"r.pid\"))))");
  ASSERT_TRUE(pump_until([&] { return finished(canceller); }));
  ASSERT_TRUE(pump_until([&] { return key("big.busy") == "0"; }));
  EXPECT_EQ(key("big.status"), "cancelled");
  EXPECT_FALSE(data<cvc::geometry>("big.result"));
}

// An Ariadne action reads a molecule on the tree through the app's own provider; another app's
// documents never got it.
TEST(MoleculeAriadne, AnActionReadsAMoleculeThroughTheAppsOwnProvider) {
  const std::string action =
      "(state-set \"doc.atoms\" (get-attr (mol.info \"doc.mol\") \"atoms\"))";
  mol::molecule three;
  three.atoms.resize(3);
  cvc::app with, without;
  for (cvc::app *a : {&with, &without})
    a->root()("doc.mol").data(boost::any(std::make_shared<const mol::molecule>(three)));
  cvc::install_molecule_intrinsics(with);
  cvc::install_molecule_intrinsics(with); // keyed: a second install replaces the first
  click_once(with, action);
  EXPECT_EQ(with.root()("doc.atoms").value(), "3");
  click_once(without, action);
  EXPECT_TRUE(without.root()("doc.atoms").value().empty());
  cvc::ariadne::clear_action_intrinsics(with);
}

INSTANTIATE_TEST_SUITE_P(Threading, MoleculeIntrinsics,
                         ::testing::Values(cvc::sliced_threading::cooperative,
                                           cvc::sliced_threading::threaded));
