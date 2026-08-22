// Settings — small state_objects for classes that are not themselves
// state_objects (see the header).

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/Settings.h>
#include <stdexcept>

namespace cvc {
namespace gl {

// ---- ShadowSettings --------------------------------------------------------
std::string ShadowSettings::sceneStatePath(const std::string &scenePrefix) {
  return scenePrefix + ".shadows";
}

ShadowSettings::ShadowSettings(cvc::app &ctx, const std::string &statePath,
                               std::function<void(Values)> apply)
    : cvc::state_object<ShadowSettings>(ctx, statePath), m_apply(std::move(apply)) {
  // Synchronous, on the calling thread: this drives VTK render passes.
  this->setInstanceThreading(false);
  seedState();
}

void ShadowSettings::seedState() {
  getState("enabled").value(m_v.enabled ? 1 : 0);
  getState("resolution").value(m_v.resolution);
  getState("interval").value(m_v.interval);
}

void ShadowSettings::set(Values v) {
  // Object -> state only. Do NOT invoke m_apply here: the caller is the object
  // reporting what it already did, and calling back into it re-applies stale
  // values (it re-enabled shadows immediately after setShadowsEnabled(false)).
  // m_apply exists for the other direction, from handleStateChanged.
  m_v = v;
  seedState();
}

ShadowSettings::Values ShadowSettings::get() const { return m_v; }

void ShadowSettings::handleStateChanged(const std::string &) {
  try {
    m_v.enabled = getState("enabled").value<int>() != 0;
    m_v.resolution = getState("resolution").value<int>();
    m_v.interval = getState("interval").value<int>();
  } catch (const std::exception &) {
    return; // partially-initialised state: leave the object alone
  }
  if (m_apply)
    m_apply(m_v);
}

// ---- UiSettings ------------------------------------------------------------
std::string UiSettings::viewerStatePath(const std::string &scenePrefix,
                                        const std::string &viewerName) {
  return scenePrefix + ".viewers." + viewerName + ".ui";
}

UiSettings::UiSettings(cvc::app &ctx, const std::string &statePath,
                       std::function<void(Values)> apply)
    : cvc::state_object<UiSettings>(ctx, statePath), m_apply(std::move(apply)) {
  this->setInstanceThreading(false);
  seedState();
}

void UiSettings::seedState() {
  getState("visible").value(m_v.visible ? 1 : 0);
  getState("scale").value(m_v.scale);
  getState("touch_mode").value(m_v.touchMode ? 1 : 0);
  getState("panels_open").value(m_v.panelsOpen ? 1 : 0);
  getState("toggle_button").value(m_v.toggleButton ? 1 : 0);
}

void UiSettings::set(Values v) {
  // Object -> state only; see the note on ShadowSettings::set.
  m_v = v;
  seedState();
}

UiSettings::Values UiSettings::get() const { return m_v; }

void UiSettings::handleStateChanged(const std::string &) {
  try {
    m_v.visible = getState("visible").value<int>() != 0;
    m_v.scale = getState("scale").value<double>();
    m_v.touchMode = getState("touch_mode").value<int>() != 0;
    m_v.panelsOpen = getState("panels_open").value<int>() != 0;
    m_v.toggleButton = getState("toggle_button").value<int>() != 0;
  } catch (const std::exception &) {
    return;
  }
  if (m_apply)
    m_apply(m_v);
}

} // namespace gl
} // namespace cvc
