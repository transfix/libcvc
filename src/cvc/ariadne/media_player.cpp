// Ariadne media players: the host's factory and the play policy. See
// inc/cvc/ariadne/media_player.h.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/media_player.h>
#include <mutex>
#include <utility>

namespace cvc {
namespace ariadne {
namespace media {

namespace {

std::mutex &registry_mutex() {
  static std::mutex m;
  return m;
}
PlayerFactory &factory_cell() {
  static PlayerFactory f;
  return f;
}
PlayPolicy &policy_cell() {
  static PlayPolicy p = PlayPolicy::Auto;
  return p;
}

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

void register_player_factory(PlayerFactory factory) {
  std::lock_guard<std::mutex> lock(registry_mutex());
  factory_cell() = std::move(factory);
}

bool have_player_factory() {
  std::lock_guard<std::mutex> lock(registry_mutex());
  return static_cast<bool>(factory_cell());
}

std::unique_ptr<Player> make_player(const PlayerSource &src) {
  PlayerFactory f;
  {
    std::lock_guard<std::mutex> lock(registry_mutex());
    f = factory_cell();
  }
  return f ? f(src) : nullptr;
}

void set_play_policy(PlayPolicy policy) {
  std::lock_guard<std::mutex> lock(registry_mutex());
  policy_cell() = policy;
}

PlayPolicy play_policy() {
  std::lock_guard<std::mutex> lock(registry_mutex());
  return policy_cell();
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

bool may_play_here() {
  switch (play_policy()) {
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
