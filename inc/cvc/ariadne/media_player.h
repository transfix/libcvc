#ifndef CVC_ARIADNE_MEDIA_PLAYER_H
#define CVC_ARIADNE_MEDIA_PLAYER_H

// Ariadne media players (docs/ARIADNE.md §5.13; design: docs/roadmap/ARIADNE_MEDIA.md §4) — what
// plays a `video:` or `audio:` widget's source. The core (here, no dependencies) owns the widget,
// its controls, its state machine and its state keys; a HOST registers the thing that decodes and
// plays, behind the Player interface:
//   - cvcMedia's FFmpeg + SDL3 player (cvc/media/player.h: register_ariadne_player), which
//   ari-media
//     and arigl register;
//   - the browser's own <video> / <audio> elements in a wasm page.
// A host with neither (the lean ari) has no factory, and its players say `unsupported`, with an
// "Open" action that hands the URL to the user's browser where the host can.
//
// Over SSH, sound would come out of the REMOTE machine's speakers. So when the process runs in an
// SSH session (SSH_CONNECTION, SSH_CLIENT or SSH_TTY set) the default policy plays nothing: players
// say `remote` and offer "Open" instead. A host flag (`--media-play always|never|auto`) sets it.

#include <cvc/image/image.h>
#include <functional>
#include <memory>
#include <string>

namespace cvc {
namespace ariadne {
namespace media {

// A player's states, as the widget's state key shows them.
enum class PlayerState {
  Idle,       // nothing to play (no src)
  Loading,    // fetching / opening the source
  Ready,      // opened, not started (its first frame, or the poster, shows)
  Playing,    //
  Paused,     //
  Ended,      // played to the end (not looping)
  Failed,     // could not fetch, open or decode (error says why)
  Remote,     // not played here: an SSH session, or the host's policy (`--media-play never`)
  Unsupported // this host has no player (the lean ari)
};
const char *player_state_name(PlayerState s); // "idle", "loading", ... "unsupported"

// What a player is asked to play.
struct PlayerSource {
  std::string uri;
  std::string auth;  // a host credential NAME for an http(s) source ("" = none)
  std::string base;  // the directory a relative file:// URI resolves against
  bool video = true; // a video: widget (false: audio:, no picture wanted)
  int frame_w = 0;   // the picture's box in px (frames are scaled to fit it)
  int frame_h = 0;
  bool loop = false;
  bool muted = false;
};

// One source being played. Created by a PlayerFactory, driven from the frame thread: every method
// is called there and returns at once (a player decodes on its own thread, or the browser does).
class Player {
public:
  virtual ~Player() = default;
  virtual void play() = 0;
  virtual void pause() = 0;
  virtual void seek(double seconds) = 0;
  virtual void set_muted(bool muted) = 0;
  virtual PlayerState state() const = 0; // Loading, Ready, Playing, Paused, Ended or Failed
  virtual double position() const = 0;   // seconds
  virtual double duration() const = 0;   // seconds; 0 when not known (yet)
  virtual std::string error() const = 0; // why it Failed; never a token
  // The picture to show now (RGBA), when it is a newer one than `gen` (then `gen` moves to it);
  // false when there is none or it has not changed.
  virtual bool frame(cvc::image &out, long &gen) = 0;
};

using PlayerFactory = std::function<std::unique_ptr<Player>(const PlayerSource &src)>;

// Register the host's player (replacing any). Null removes it. Process-global; call at setup.
void register_player_factory(PlayerFactory factory);
bool have_player_factory();
// A new player for `src` from the registered factory; null when there is none.
std::unique_ptr<Player> make_player(const PlayerSource &src);

// In a browser build (ari.html, arigl-web): register the page's own <video> / <audio> elements as
// the player (media_browser.cpp), the policy and credentials decided in C++ before the element sees
// a URL, a token or bytes. Returns false, registering nothing, off the browser.
bool register_browser_player();

// Whether players may play here. Auto: yes, except in an SSH session (the sound would play on the
// remote machine); Always: yes; Never: no. Process-global.
enum class PlayPolicy { Auto, Always, Never };
void set_play_policy(PlayPolicy policy);
PlayPolicy play_policy();
bool parse_play_policy(const std::string &s, PlayPolicy &out); // "auto" | "always" | "never"
// Whether this process runs in an SSH session (SSH_CONNECTION, SSH_CLIENT or SSH_TTY is set).
bool in_ssh_session();
// Whether a player may play now, by the policy and the session.
bool may_play_here();

// "m:ss" (or "h:mm:ss"), for the controls' time line. Negative or unknown: "-:--".
std::string format_time(double seconds);

} // namespace media
} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_MEDIA_PLAYER_H
