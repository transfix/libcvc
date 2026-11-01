/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_AVATAR_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_AVATAR_H__

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
namespace conference {

// ----------------
// Avatar looks: a server-supplied catalog, one choice per participant
// ----------------
// Every client draws an avatar the same way for every viewer, its own included: the look comes
// from the room tree, never from who is looking. The relay supplies what can be worn (the
// catalog); each participant picks from it; every renderer resolves a choice through the catalog,
// so a choice the catalog does not offer draws as the default.
//
//   conf.rooms.<room>.avatars.rev                       bumped by every catalog write (written
//   last) conf.rooms.<room>.avatars.colors.<id>.{present, order, label, auto, color.{r,g,b}}
//   conf.rooms.<room>.avatars.accessories.<id>.{present, order, label, parts.count,
//       parts.<i>.{shape, size.{x,y,z}, radius, pos.{x,y,z}, tint, color.{r,g,b}}}
//   conf.rooms.<room>.participants.<pid>.avatar.{color, accessory}   ids, written by <pid> itself
//
// AUTHORITY: the relay is the sole writer of avatars.* (a client may write only its own
// participant subtree; room_write_gate). A participant writes only its own avatar.* choice, which
// is cosmetic: the body the physics simulates is the same 0.5 x 0.5 x 1.7 m box for everyone.
// Ids must not contain cvc::state::SEPARATOR (std::invalid_argument otherwise).

// The body every avatar has, in metres: a box standing on its pose (base at the pose's z), its
// front +X in the avatar's frame. The physics body is this size whatever is worn.
constexpr float kAvatarWidth = 0.5f;
constexpr float kAvatarHeight = 1.7f;

// What an accessory may occupy, in the avatar's frame from its feet (x forward, z up): the body
// grown 0.15 m on every side and 0.3 m on top. A part whose box reaches outside is dropped when a
// look is resolved, so no catalog can draw a figure much bigger than the body it collides as.
constexpr float kAccessoryReach = 0.5f * kAvatarWidth + 0.15f;
constexpr float kAccessoryTop = kAvatarHeight + 0.3f;
// At most this many parts per accessory; the rest are dropped.
constexpr std::size_t kMaxAccessoryParts = 6;

// One piece of an accessory: a box (SIZE, edge lengths) or a sphere (RADIUS), centred at POS from
// the avatar's feet. TINT colours it: "body" (the avatar's colour), "visor" (the visor's), or
// "rgb" (COLOR).
struct avatar_part {
  std::string shape = "box"; // "box" or "sphere"
  float size[3] = {0.1f, 0.1f, 0.1f};
  float radius = 0.05f;
  float pos[3] = {0.0f, 0.0f, kAvatarHeight};
  std::string tint = "body";
  float color[3] = {1.0f, 1.0f, 1.0f};
};

// A body colour: a fixed COLOR, or AUTOMATIC -- a hue every client derives from the participant's
// id (auto_avatar_color), so people who never chose still look different from one another.
struct avatar_color_option {
  std::string id, label;
  bool automatic = false;
  float color[3] = {0.8f, 0.8f, 0.8f};
};

struct avatar_accessory_option {
  std::string id, label;
  std::vector<avatar_part> parts; // none: nothing worn
};

// What a room offers, in the order its pickers list it. The first of each list is the default.
struct avatar_catalog {
  std::vector<avatar_color_option> colors;
  std::vector<avatar_accessory_option> accessories;

  bool empty() const { return colors.empty() && accessories.empty(); }
  // The option with this id or, failing that, this label; null if none.
  const avatar_color_option *find_color(const std::string &id_or_label) const;
  const avatar_accessory_option *find_accessory(const std::string &id_or_label) const;
};

// conf.rooms.<room-id>.avatars
std::string room_avatars_path(const std::string &room_id);

// The catalog revision: 0 when none was ever written, then +1 per write.
std::uint64_t avatar_catalog_revision(cvc::app &app, const std::string &room_id);

// Write the whole catalog. Options present before but absent now are soft-removed (present = 0).
void write_avatar_catalog(cvc::app &app, const std::string &room_id, const avatar_catalog &c);

// Read the catalog (present options only, in their written order). Empty if none was written.
avatar_catalog read_avatar_catalog(cvc::app &app, const std::string &room_id);

// What the relay offers by default: an automatic colour (the default), ten named colours, and a
// few accessories inside the envelope -- none (the default), a cap, a top hat, an antenna, a
// crest, a halo and a backpack.
avatar_catalog default_avatar_catalog();

// Seed ROOM-ID with the default catalog if it has none yet. Idempotent. True if it seeded.
bool seed_avatar_catalog(cvc::app &app, const std::string &room_id);

// A participant's choice: option ids ("" = the default).
struct avatar_choice {
  std::string color, accessory;
};

// PID's choice, as written under its own participant subtree (both "" if never written).
avatar_choice read_avatar_choice(cvc::app &app, const std::string &room_id, const std::string &pid);

// Write PID's choice (its own subtree only: what a client calls for itself).
void write_avatar_choice(cvc::app &app, const std::string &room_id, const std::string &pid,
                         const avatar_choice &choice);

// The hue PID gets with the automatic colour: FNV-1a of the pid on the hue circle at fixed
// saturation and value, so every client derives the same colour from the same id.
void auto_avatar_color(const std::string &pid, float out[3]);

// What to draw: the body and visor colours and the accessory's parts, each part's colour
// resolved from its tint. KEY changes whenever anything drawn would, for a renderer to rebuild on.
struct avatar_look {
  float body[3] = {0.8f, 0.8f, 0.8f};
  float visor[3] = {0.08f, 0.09f, 0.12f};
  std::vector<avatar_part> parts; // tint already applied to color; inside the envelope
  std::string color_id, accessory_id;
  std::string key;
};

// Resolve CHOICE through CATALOG for PID: an id (or label) the catalog lacks falls back to its
// first option; an empty catalog gives the automatic colour and nothing worn. Parts outside the
// envelope (kAccessoryReach, kAccessoryTop) and past kMaxAccessoryParts are dropped.
avatar_look resolve_avatar_look(const avatar_catalog &catalog, const avatar_choice &choice,
                                const std::string &pid);

// Whether PART lies wholly inside the accessory envelope.
bool avatar_part_fits(const avatar_part &part);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_AVATAR_H__
