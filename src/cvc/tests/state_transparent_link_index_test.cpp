/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 4c: transparent-link index and alias expansion.
//
// Verifies state_distributed_admin::transparent_link_index() and
// state_distributed_admin::transparent_link_aliases(). The index
// must only enumerate transparent links (opaque links are
// ignored). The alias expansion must:
//   - return [] when no transparent link covers the path;
//   - emit link-side alias for exact target matches;
//   - emit link-side alias preserving the suffix for descendants
//     of a target;
//   - respect dot-segment boundaries ("scenery" must NOT match a
//     link target of "scene");
//   - emit one alias per transparent link that covers the path;
//   - skip self-aliases at the link node itself;
//   - handle a link whose target is the application root ("").

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_distributed_admin.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

using cvc::state;
using cvc::state_distributed_admin;

namespace {

bool contains(const std::vector<std::string> &v, const std::string &s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

} // namespace

TEST(StateTransparentLinkIndex, EmptyTreeYieldsNoEntries) {
  cvc::app a;
  auto &root = state::instance(a);
  auto r = state_distributed_admin::transparent_link_index(root);
  EXPECT_EQ(r.links.size(), 0u);
  EXPECT_EQ(r.link_nodes_scanned, 0u);
}

TEST(StateTransparentLinkIndex, OpaqueLinkIsIgnored) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry").value(std::string("v"));
  root("scene.geometry").linkTo("data.world.geometry"); // default opaque

  auto r = state_distributed_admin::transparent_link_index(root);
  EXPECT_EQ(r.links.size(), 0u);
  EXPECT_EQ(r.link_nodes_scanned, 1u);
}

TEST(StateTransparentLinkIndex, TransparentLinkIsIndexed) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry").value(std::string("v"));
  root("scene.geometry")
      .linkTo("data.world.geometry", state::link_mode::transparent);

  auto r = state_distributed_admin::transparent_link_index(root);
  ASSERT_EQ(r.links.size(), 1u);
  EXPECT_EQ(r.links[0].link_path, "scene.geometry");
  EXPECT_EQ(r.links[0].target_path, "data.world.geometry");
  EXPECT_EQ(r.link_nodes_scanned, 1u);
}

TEST(StateTransparentLinkIndex, RootTargetCanonicalizesToEmpty) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene.alias").linkTo(".", state::link_mode::transparent);

  auto r = state_distributed_admin::transparent_link_index(root);
  ASSERT_EQ(r.links.size(), 1u);
  EXPECT_EQ(r.links[0].link_path, "scene.alias");
  EXPECT_EQ(r.links[0].target_path, std::string()); // canonical root
}

TEST(StateTransparentLinkIndex, MixedOpaqueAndTransparent) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.a").value(std::string("v"));
  root("data.world.b").value(std::string("v"));
  root("scene.a").linkTo("data.world.a"); // opaque
  root("scene.b").linkTo("data.world.b", state::link_mode::transparent);

  auto r = state_distributed_admin::transparent_link_index(root);
  ASSERT_EQ(r.links.size(), 1u);
  EXPECT_EQ(r.links[0].link_path, "scene.b");
  EXPECT_EQ(r.links[0].target_path, "data.world.b");
  EXPECT_EQ(r.link_nodes_scanned, 2u);
}

TEST(StateTransparentLinkAliases, NoTransparentLinkYieldsEmpty) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene.geometry").linkTo("data.world.geometry"); // opaque
  auto aliases =
      state_distributed_admin::transparent_link_aliases(root,
                                                       "data.world.geometry");
  EXPECT_EQ(aliases.size(), 0u);
}

TEST(StateTransparentLinkAliases, ExactTargetMatchEmitsLinkPath) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry").value(std::string("v"));
  root("scene.geometry")
      .linkTo("data.world.geometry", state::link_mode::transparent);

  auto aliases = state_distributed_admin::transparent_link_aliases(
      root, "data.world.geometry");
  ASSERT_EQ(aliases.size(), 1u);
  EXPECT_EQ(aliases[0], "scene.geometry");
}

TEST(StateTransparentLinkAliases, DescendantPathPreservesSuffix) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world").value(std::string("v"));
  root("scene").linkTo("data.world", state::link_mode::transparent);

  auto aliases = state_distributed_admin::transparent_link_aliases(
      root, "data.world.geometry.mesh");
  ASSERT_EQ(aliases.size(), 1u);
  EXPECT_EQ(aliases[0], "scene.geometry.mesh");
}

TEST(StateTransparentLinkAliases, DotBoundaryPreventsSpoof) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene").value(std::string("v"));
  root("alias").linkTo("scene", state::link_mode::transparent);

  // "scenery" must not match link target "scene".
  auto aliases =
      state_distributed_admin::transparent_link_aliases(root, "scenery");
  EXPECT_EQ(aliases.size(), 0u);
}

TEST(StateTransparentLinkAliases, MultipleLinksAllEmit) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry").value(std::string("v"));
  root("scene.a")
      .linkTo("data.world.geometry", state::link_mode::transparent);
  root("scene.b")
      .linkTo("data.world.geometry", state::link_mode::transparent);

  auto aliases = state_distributed_admin::transparent_link_aliases(
      root, "data.world.geometry");
  ASSERT_EQ(aliases.size(), 2u);
  EXPECT_TRUE(contains(aliases, "scene.a"));
  EXPECT_TRUE(contains(aliases, "scene.b"));
}

TEST(StateTransparentLinkAliases, SelfAliasIsSkipped) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("scene.geometry");
  n.linkTo("scene.geometry", state::link_mode::transparent); // self-loop

  auto aliases = state_distributed_admin::transparent_link_aliases(
      root, "scene.geometry");
  // Path equals link_path: the trivial self-alias must be skipped.
  EXPECT_EQ(aliases.size(), 0u);
}

TEST(StateTransparentLinkAliases, RootTargetAliasesEveryPath) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene.alias").linkTo(".", state::link_mode::transparent);

  auto aliases = state_distributed_admin::transparent_link_aliases(
      root, "data.world.geometry");
  ASSERT_EQ(aliases.size(), 1u);
  EXPECT_EQ(aliases[0], "scene.alias.data.world.geometry");
}

TEST(StateTransparentLinkAliases, MismatchedSiblingPrefixNotEmitted) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene").value(std::string("v"));
  root("alias").linkTo("scene", state::link_mode::transparent);

  auto aliases =
      state_distributed_admin::transparent_link_aliases(root, "other.scene");
  EXPECT_EQ(aliases.size(), 0u);
}
