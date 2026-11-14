// Ariadne media players: the host's factory and the play policy. See
// inc/cvc/ariadne/media_player.h.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/media_player.h>
#include <cvc/core/app.h>
#include <mutex>
#include <utility>

namespace cvc {
namespace ariadne {
namespace media {

namespace {

// An app's player factory and play policy.
struct media_registry {
  static constexpr const char *app_slot = "cvc.ariadne.media";
  std::mutex m;
  PlayerFactory factory;
  PlayPolicy policy = PlayPolicy::Auto;
};

bool set_env(const char *name) {
  const char *v = std::getenv(name);
  return v && *v;
}

} // namespace

const char *player_state_name(PlayerState s) {
  switch (s) {
  case PlayerState::Idle:
    return "idle";
  case PlayerState::Loading:
    return "loading";
  case PlayerState::Ready:
    return "ready";
  case PlayerState::Playing:
    return "playing";
  case PlayerState::Paused:
    return "paused";
  case PlayerState::Ended:
    return "ended";
  case PlayerState::Failed:
    return "failed";
  case PlayerState::Remote:
    return "remote";
  case PlayerState::Unsupported:
    return "unsupported";
  }
  return "idle";
}

void register_player_factory(cvc::app &app, PlayerFactory factory) {
  media_registry &r = app.slot<media_registry>();
  std::lock_guard<std::mutex> lock(r.m);
  r.factory = std::move(factory);
}

bool have_player_factory(cvc::app &app) {
  media_registry &r = app.slot<media_registry>();
  std::lock_guard<std::mutex> lock(r.m);
  return static_cast<bool>(r.factory);
}

std::unique_ptr<Player> make_player(cvc::app &app, const PlayerSource &src) {
  PlayerFactory f;
  {
    media_registry &r = app.slot<media_registry>();
    std::lock_guard<std::mutex> lock(r.m);
    f = r.factory;
  }
  if (!f)
    return nullptr;
  PlayerSource s = src;
  s.app = &app; // whose network and resolver fetch it
  return f(s);
}

void set_play_policy(cvc::app &app, PlayPolicy policy) {
  media_registry &r = app.slot<media_registry>();
  std::lock_guard<std::mutex> lock(r.m);
  r.policy = policy;
}

PlayPolicy play_policy(cvc::app &app) {
  media_registry &r = app.slot<media_registry>();
  std::lock_guard<std::mutex> lock(r.m);
  return r.policy;
}

bool parse_play_policy(const std::string &s, PlayPolicy &out) {
  if (s == "auto")
    out = PlayPolicy::Auto;
  else if (s == "always")
    out = PlayPolicy::Always;
  else if (s == "never")
    out = PlayPolicy::Never;
  else
    return false;
  return true;
}

bool in_ssh_session() {
#ifdef __EMSCRIPTEN__
  return false; // a browser plays on the machine it shows on
#else
  return set_env("SSH_CONNECTION") || set_env("SSH_CLIENT") || set_env("SSH_TTY");
#endif
}

bool may_play_here(cvc::app &app) {
  switch (play_policy(app)) {
  case PlayPolicy::Always:
    return true;
  case PlayPolicy::Never:
    return false;
  case PlayPolicy::Auto:
    break;
  }
  return !in_ssh_session();
}

std::string format_time(double seconds) {
  if (!(seconds >= 0.0) || !std::isfinite(seconds))
    return "-:--";
  const long t = static_cast<long>(seconds);
  char buf[32];
  if (t >= 3600)
    std::snprintf(buf, sizeof(buf), "%ld:%02ld:%02ld", t / 3600, (t / 60) % 60, t % 60);
  else
    std::snprintf(buf, sizeof(buf), "%ld:%02ld", t / 60, t % 60);
  return buf;
}

} // namespace media
} // namespace ariadne
} // namespace cvc
