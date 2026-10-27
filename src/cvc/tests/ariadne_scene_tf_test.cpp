/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The scene `transfer_function:` block as the loader parses it: the legacy raw form (points,
// window, auto_domain) unchanged, and the roadmap §10.1 model form (domain, opacity_cubed,
// resolution, preset, vinay, color, alpha, isocontours) into SceneTransferFunction::model, on all
// three volume renderers. Skipped when the build has no yaml-cpp (the loader is then a stub).

#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/tf/model.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

using namespace cvc::ariadne;

namespace {

#define SKIP_WITHOUT_YAML()                                                                        \
  do {                                                                                             \
    if (!have_yaml())                                                                              \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

bool has_warning(const LoadResult &r, const std::string &needle) {
  for (const std::string &w : r.warnings)
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

// One node of `type` whose `<block>.transfer_function:` is `tf` (YAML flow or block text indented
// for the slot).
std::string doc(const std::string &type, const std::string &tf) {
  return "meta: { min_libcvc: \"0.0.0\" }\n"
         "scene:\n"
         "  nodes:\n"
         "    - node: v\n"
         "      type: " +
         type +
         "\n"
         "      source: ball.rawiv\n"
         "      " +
         type +
         ":\n"
         "        transfer_function: " +
         tf + "\n";
}

const SceneTransferFunction *tf_of(const LoadResult &r) {
  if (r.scene.nodes.empty())
    return nullptr;
  const SceneNode &n = r.scene.nodes[0];
  if (n.type == "volren")
    return &n.volren.tf;
  if (n.type == "volslice")
    return &n.volslice.tf;
  return &n.volume.tf;
}

std::filesystem::path temp_dir() {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ariadne_scene_tf_test";
  std::filesystem::create_directories(dir);
  return dir;
}

} // namespace

TEST(AriadneSceneTf, LegacyFormUnchanged) {
  SKIP_WITHOUT_YAML();
  const LoadResult r = load_string(
      doc("volslice", "{ window: [0.1, 0.9], points: [{ value: 0.2, color: [1, 0, 0, 0] }, "
                      "{ value: 0.8, color: [0, 0, 1, 0.5] }] }"));
  ASSERT_TRUE(r.ok) << r.error;
  const SceneTransferFunction *tf = tf_of(r);
  ASSERT_NE(tf, nullptr);
  EXPECT_FALSE(tf->has_model);
  EXPECT_FALSE(tf->empty());
  EXPECT_TRUE(tf->has_window);
  EXPECT_FALSE(tf->auto_domain);
  EXPECT_DOUBLE_EQ(tf->window_min, 0.1);
  EXPECT_DOUBLE_EQ(tf->window_max, 0.9);
  ASSERT_EQ(tf->points.size(), 2u);
  EXPECT_DOUBLE_EQ(tf->points[1].value, 0.8);
  EXPECT_FLOAT_EQ(tf->points[1].color[3], 0.5f);
  EXPECT_FALSE(has_warning(r, "transfer_function"));

  const LoadResult none = load_string(doc("volume", "{ auto_domain: false }"));
  ASSERT_TRUE(none.ok) << none.error;
  ASSERT_NE(tf_of(none), nullptr);
  EXPECT_FALSE(tf_of(none)->has_model);
  EXPECT_TRUE(tf_of(none)->empty());
  EXPECT_FALSE(tf_of(none)->auto_domain);
}

TEST(AriadneSceneTf, ListNodeForms) {
  SKIP_WITHOUT_YAML();
  const LoadResult r =
      load_string(doc("volren", "{ color: [[0, 0, 0, 0], [0.5, 1, 0, 0], [1, 0, 1, 0]], "
                                "alpha: [[0, 0], [0.25, 0.75], [1, 1]] }"));
  ASSERT_TRUE(r.ok) << r.error;
  const SceneTransferFunction *tf = tf_of(r);
  ASSERT_NE(tf, nullptr);
  ASSERT_TRUE(tf->has_model);
  EXPECT_FALSE(tf->empty());
  const cvc::tf::model &m = tf->model;
  ASSERT_EQ(m.color.size(), 3u);
  EXPECT_DOUBLE_EQ(m.color[1].at, 0.5);
  EXPECT_FLOAT_EQ(m.color[1].r, 1.0f);
  EXPECT_FLOAT_EQ(m.color[2].g, 1.0f);
  ASSERT_EQ(m.alpha.size(), 3u);
  EXPECT_DOUBLE_EQ(m.alpha[1].at, 0.25);
  EXPECT_FLOAT_EQ(m.alpha[1].a, 0.75f);
  EXPECT_TRUE(m.domain.automatic);
  EXPECT_TRUE(m.isocontours.empty());
  EXPECT_FALSE(has_warning(r, "transfer_function"));
}

TEST(AriadneSceneTf, MapNodeForms) {
  SKIP_WITHOUT_YAML();
  const LoadResult r =
      load_string(doc("volume", "{ color: [{ at: 0, rgb: [0, 0, 1] }, { at: 1, rgb: [1, 1, 0] }], "
                                "alpha: [{ at: 0, a: 0.1 }, { at: 1, a: 0.9 }] }"));
  ASSERT_TRUE(r.ok) << r.error;
  const SceneTransferFunction *tf = tf_of(r);
  ASSERT_NE(tf, nullptr);
  ASSERT_TRUE(tf->has_model);
  ASSERT_EQ(tf->model.color.size(), 2u);
  EXPECT_FLOAT_EQ(tf->model.color[0].b, 1.0f);
  EXPECT_FLOAT_EQ(tf->model.color[1].g, 1.0f);
  ASSERT_EQ(tf->model.alpha.size(), 2u);
  EXPECT_FLOAT_EQ(tf->model.alpha[0].a, 0.1f);
  EXPECT_FLOAT_EQ(tf->model.alpha[1].a, 0.9f);
  EXPECT_FALSE(has_warning(r, "transfer_function"));
}

TEST(AriadneSceneTf, DomainForms) {
  SKIP_WITHOUT_YAML();
  {
    const LoadResult r = load_string(doc("volslice", "{ domain: [-2, 6] }"));
    ASSERT_TRUE(r.ok) << r.error;
    const cvc::tf::model &m = tf_of(r)->model;
    ASSERT_TRUE(tf_of(r)->has_model);
    EXPECT_FALSE(m.domain.automatic);
    EXPECT_DOUBLE_EQ(m.domain.min, -2.0);
    EXPECT_DOUBLE_EQ(m.domain.max, 6.0);
    // Only the domain was given: the colour and opacity are the default model's.
    EXPECT_EQ(m.color.size(), cvc::tf::model::default_model().color.size());
  }
  {
    const LoadResult r = load_string(doc("volslice", "{ domain: { min: 1, max: 3 } }"));
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_FALSE(tf_of(r)->model.domain.automatic);
    EXPECT_DOUBLE_EQ(tf_of(r)->model.domain.max, 3.0);
  }
  {
    const LoadResult r = load_string(doc("volslice", "{ domain: auto }"));
    ASSERT_TRUE(r.ok) << r.error;
    ASSERT_TRUE(tf_of(r)->has_model);
    EXPECT_TRUE(tf_of(r)->model.domain.automatic);
  }
  {
    const LoadResult r = load_string(doc("volslice", "{ domain: [5, 1] }"));
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(tf_of(r)->model.domain.automatic);
    EXPECT_TRUE(has_warning(r, "domain: max must exceed min"));
  }
  {
    const LoadResult r = load_string(doc("volslice", "{ domain: sideways }"));
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(has_warning(r, "domain: must be auto or [min, max]"));
  }
  {
    // The legacy window beside the model keys is the explicit domain.
    const LoadResult r =
        load_string(doc("volslice", "{ window: [0.2, 0.4], opacity_cubed: true }"));
    ASSERT_TRUE(r.ok) << r.error;
    const SceneTransferFunction *tf = tf_of(r);
    ASSERT_TRUE(tf->has_model);
    EXPECT_TRUE(tf->has_window);
    EXPECT_FALSE(tf->model.domain.automatic);
    EXPECT_DOUBLE_EQ(tf->model.domain.min, 0.2);
    EXPECT_DOUBLE_EQ(tf->model.domain.max, 0.4);
    EXPECT_TRUE(tf->model.opacity_cubed);
  }
}

TEST(AriadneSceneTf, FlagsAndResolution) {
  SKIP_WITHOUT_YAML();
  const LoadResult r =
      load_string(doc("volren", "{ opacity_cubed: true, resolution: 512, dims: 1 }"));
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_TRUE(tf_of(r)->has_model);
  EXPECT_TRUE(tf_of(r)->model.opacity_cubed);
  EXPECT_EQ(tf_of(r)->model.resolution, 512);
  EXPECT_FALSE(has_warning(r, "transfer_function"));

  const LoadResult bad = load_string(doc("volren", "{ resolution: 1, dims: 2 }"));
  ASSERT_TRUE(bad.ok) << bad.error;
  EXPECT_EQ(tf_of(bad)->model.resolution, cvc::tf::model::default_model().resolution);
  EXPECT_TRUE(has_warning(bad, "resolution:"));
  EXPECT_TRUE(has_warning(bad, "dims:"));
}

TEST(AriadneSceneTf, PresetThenListsOverride) {
  SKIP_WITHOUT_YAML();
  const LoadResult r = load_string(doc("volren", "{ preset: grayscale }"));
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_TRUE(tf_of(r)->has_model);
  EXPECT_TRUE(tf_of(r)->model == cvc::tf::model::preset("grayscale"));

  // A list given replaces the preset's list; the others stay the preset's.
  const LoadResult o = load_string(doc("volren", "{ preset: grayscale, alpha: [[0, 1], [1, 1]] }"));
  ASSERT_TRUE(o.ok) << o.error;
  const cvc::tf::model &m = tf_of(o)->model;
  EXPECT_EQ(m.color.size(), cvc::tf::model::preset("grayscale").color.size());
  ASSERT_EQ(m.alpha.size(), 2u);
  EXPECT_FLOAT_EQ(m.alpha[0].a, 1.0f);

  const LoadResult u = load_string(doc("volren", "{ preset: no_such_preset }"));
  ASSERT_TRUE(u.ok) << u.error;
  EXPECT_TRUE(has_warning(u, "unknown preset 'no_such_preset'"));
  EXPECT_TRUE(tf_of(u)->model == cvc::tf::model::default_model());
}

TEST(AriadneSceneTf, IsocontourForms) {
  SKIP_WITHOUT_YAML();
  const LoadResult r = load_string(
      doc("volren", "{ isocontours: [{ id: bone, at: 0.6, rgb: [1, 0.5, 0.25], opacity: 0.5 }, "
                    "0.3, { at: 0.8 }] }"));
  ASSERT_TRUE(r.ok) << r.error;
  const cvc::tf::model &m = tf_of(r)->model;
  ASSERT_EQ(m.isocontours.size(), 3u);
  // Sorted by position; the bare number and the id-less map get fresh unique ids.
  EXPECT_DOUBLE_EQ(m.isocontours[0].at, 0.3);
  EXPECT_FALSE(m.isocontours[0].has_rgb);
  EXPECT_FALSE(m.isocontours[0].has_opacity);
  EXPECT_FALSE(m.isocontours[0].id.empty());
  EXPECT_EQ(m.isocontours[1].id, "bone");
  EXPECT_DOUBLE_EQ(m.isocontours[1].at, 0.6);
  EXPECT_TRUE(m.isocontours[1].has_rgb);
  EXPECT_FLOAT_EQ(m.isocontours[1].g, 0.5f);
  EXPECT_TRUE(m.isocontours[1].has_opacity);
  EXPECT_FLOAT_EQ(m.isocontours[1].opacity, 0.5f);
  EXPECT_FALSE(m.isocontours[2].id.empty());
  EXPECT_NE(m.isocontours[0].id, m.isocontours[2].id);
  EXPECT_FALSE(has_warning(r, "transfer_function"));

  // An empty list means "no isocontours" (it replaces a preset's), not "unset".
  const LoadResult e = load_string(doc("volren", "{ preset: isolines, isocontours: [] }"));
  ASSERT_TRUE(e.ok) << e.error;
  EXPECT_TRUE(tf_of(e)->model.isocontours.empty());

  const LoadResult bad =
      load_string(doc("volren", "{ isocontours: [{ id: x }, nope, { at: 1.5 }] }"));
  ASSERT_TRUE(bad.ok) << bad.error;
  EXPECT_TRUE(has_warning(bad, "an isocontour is not a number"));
  EXPECT_TRUE(has_warning(bad, "outside the normalized [0, 1] axis"));
  ASSERT_EQ(tf_of(bad)->model.isocontours.size(), 1u);
  EXPECT_DOUBLE_EQ(tf_of(bad)->model.isocontours[0].at, 1.0); // clamped
}

TEST(AriadneSceneTf, VinayImportResolvesAgainstTheDocument) {
  SKIP_WITHOUT_YAML();
  const std::filesystem::path dir = temp_dir();
  const std::filesystem::path vinay = dir / "skull.vinay";
  {
    std::ofstream f(vinay, std::ios::binary);
    f << "Anthony and Vinay are Great.\n"
         "Alphamap\nNumber of nodes\n3\nPosition and opacity\n0 0\n1 1\n0.5 0.25\n"
         "ColorMap\nNumber of nodes\n2\nPosition and RGB\n0 1 0 0\n1 0 0 1\n"
         "IsocontourMap\nNumber of nodes\n1\nPosition\n0.4\n";
  }
  const std::filesystem::path ari = dir / "vinay_doc.ari";
  {
    std::ofstream f(ari, std::ios::binary);
    f << doc("volslice", "{ vinay: skull.vinay, domain: [0, 10] }");
  }
  const LoadResult r = load_file(ari.string());
  ASSERT_TRUE(r.ok) << r.error;
  const SceneTransferFunction *tf = tf_of(r);
  ASSERT_NE(tf, nullptr);
  ASSERT_TRUE(tf->has_model);
  const cvc::tf::model &m = tf->model;
  ASSERT_EQ(m.alpha.size(), 3u);
  EXPECT_DOUBLE_EQ(m.alpha[1].at, 0.5);
  EXPECT_FLOAT_EQ(m.alpha[1].a, 0.25f);
  ASSERT_EQ(m.color.size(), 2u);
  EXPECT_FLOAT_EQ(m.color[0].r, 1.0f);
  EXPECT_FLOAT_EQ(m.color[1].b, 1.0f);
  ASSERT_EQ(m.isocontours.size(), 1u);
  EXPECT_DOUBLE_EQ(m.isocontours[0].at, 0.4);
  // The keys beside it still apply.
  EXPECT_FALSE(m.domain.automatic);
  EXPECT_DOUBLE_EQ(m.domain.max, 10.0);
  EXPECT_FALSE(has_warning(r, "vinay"));
  // §12.5: the colour table file is watched for hot-reload with the document.
  bool watched = false;
  for (const std::string &s : r.sources)
    watched = watched || std::filesystem::path(s).filename() == "skull.vinay";
  EXPECT_TRUE(watched);

  // An unresolvable file warns and keeps the starting model.
  const std::filesystem::path missing = dir / "missing_vinay.ari";
  {
    std::ofstream f(missing, std::ios::binary);
    f << doc("volslice", "{ vinay: no_such_file.vinay }");
  }
  const LoadResult w = load_file(missing.string());
  ASSERT_TRUE(w.ok) << w.error;
  EXPECT_TRUE(has_warning(w, "vinay 'no_such_file.vinay' could not be resolved"));
  EXPECT_TRUE(tf_of(w)->has_model);
  EXPECT_TRUE(tf_of(w)->model == cvc::tf::model::default_model());
}

TEST(AriadneSceneTf, UnknownAndMisplacedKeysWarn) {
  SKIP_WITHOUT_YAML();
  const LoadResult r = load_string(doc("volren", "{ colour: [[0, 1, 1, 1]], table: x }"));
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "unknown key 'colour'"));
  EXPECT_TRUE(has_warning(r, "unknown key 'table'"));
  EXPECT_TRUE(has_warning(r, "scene node 'v'"));
  EXPECT_FALSE(tf_of(r)->has_model); // nothing known set the model form

  // Legacy points beside the model keys are ignored, and say so.
  const LoadResult mix = load_string(
      doc("volren", "{ points: [{ value: 0, color: [1, 1, 1, 1] }], alpha: [[0, 0], [1, 1]] }"));
  ASSERT_TRUE(mix.ok) << mix.error;
  EXPECT_TRUE(tf_of(mix)->has_model);
  EXPECT_TRUE(has_warning(mix, "legacy points: are ignored"));

  // A data value written where a normalized position belongs.
  const LoadResult range = load_string(doc("volren", "{ alpha: [[0, 0], [128, 1], [1, 1]] }"));
  ASSERT_TRUE(range.ok) << range.error;
  EXPECT_TRUE(has_warning(range, "alpha: node at 128"));
  EXPECT_EQ(tf_of(range)->model.alpha.size(), 2u);

  // A malformed node is skipped; a list with nothing usable keeps the starting list.
  const LoadResult junk = load_string(doc("volren", "{ color: [[0, 1], hello] }"));
  ASSERT_TRUE(junk.ok) << junk.error;
  EXPECT_TRUE(has_warning(junk, "a color: node is not"));
  EXPECT_TRUE(has_warning(junk, "color: has no usable node"));
  EXPECT_TRUE(tf_of(junk)->model == cvc::tf::model::default_model());

  const LoadResult scalar = load_string(doc("volren", "oops"));
  ASSERT_TRUE(scalar.ok) << scalar.error;
  EXPECT_TRUE(has_warning(scalar, "transfer_function: must be a map"));
}

TEST(AriadneSceneTf, EveryVolumeRenderer) {
  SKIP_WITHOUT_YAML();
  for (const char *type : {"volren", "volslice", "volume"}) {
    const LoadResult r = load_string(doc(type, "{ preset: hot, domain: [0, 255] }"));
    ASSERT_TRUE(r.ok) << r.error;
    const SceneTransferFunction *tf = tf_of(r);
    ASSERT_NE(tf, nullptr) << type;
    EXPECT_TRUE(tf->has_model) << type;
    EXPECT_FALSE(tf->model.domain.automatic) << type;
    EXPECT_DOUBLE_EQ(tf->model.domain.max, 255.0) << type;
  }
}
