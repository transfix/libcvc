/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Runtime's half of Kind::TfEditor (private to the Ariadne core): read the bound model, the
// overlays published beside it and the instance's view from cvc::state, hand them to the backend,
// and write the edits back — or, on a backend with no native editor, compose one from sliders,
// colour pickers and buttons. Kept out of ariadne.cpp so the walk stays a dispatcher.

#ifndef CVC_ARIADNE_TF_EDITOR_EMIT_H
#define CVC_ARIADNE_TF_EDITOR_EMIT_H

#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/tf/model.h>
#include <cvc/tf/overlays.h>
#include <functional>
#include <string>
#include <unordered_map>

namespace cvc {
class app;
namespace ariadne {

// Parse caches for one tf_editor INSTANCE (keyed by its backend key, "<resolved bind>#<id>", so two
// editors on one model keep their own preview): each string is re-parsed only when the state value
// changes.
struct TfEditorCache {
  std::string model_src;
  cvc::tf::model model;
  bool model_ok = false;   // `model` holds something to show (model_src's, or a fallback)
  bool src_parsed = false; // model_src parsed: `model` is the model the key holds
  std::string range_src, histogram_src, spectrum_src, tree_src;
  cvc::tf::overlays overlays;
  // Bumped whenever the parsed model / an overlay changes: the composed preview's signature uses
  // these instead of the (possibly large) source strings.
  unsigned model_gen = 0, overlay_gen = 0;
  std::string preview_sig;
};

struct TfEditorContext {
  cvc::app &app;
  Backend &backend;
  std::function<std::string(const std::string &)> resolve; // bind -> state path (current scope)
  std::function<void(const std::string &)> enqueue;        // fire an `on:` action
  std::function<void(const std::string &)> warn_once;      // a de-duplicated diagnostic
  std::unordered_map<std::string, TfEditorCache> &caches;
};

// Emit one Kind::TfEditor widget.
void emit_tf_editor(TfEditorContext &ctx, const Widget &w);

// The model a tf_editor seeds an empty bound key with: props `default` (a model string, or the
// same document as a YAML map), else props `preset` (a preset name), else
// cvc::tf::model::default_model(); then any of props color / alpha / isocontours / domain /
// opacity_cubed / resolution (the scene transfer_function: vocabulary) replace that part. Props
// keep no YAML type, so each scalar is typed by the model's schema (an isocontour id is a string,
// opacity_cubed takes the loader's flag() spellings, positions and colours are numbers), never by
// its text, and numbers are written locale-independently. A `default` or list that does not parse
// is skipped, with the reason in `*problem` when given.
cvc::tf::model tf_editor_seed(const Widget &w, std::string *problem = nullptr);

// The initial view (layer toggles) from props `layers:` (a list) / `show_*` flags.
cvc::tf::view tf_editor_initial_view(const Widget &w);

// The instance id the view persists under (tree.<id>.view.*): the widget id, else its label, else
// "tf_editor".
std::string tf_editor_instance_id(const Widget &w);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_TF_EDITOR_EMIT_H
