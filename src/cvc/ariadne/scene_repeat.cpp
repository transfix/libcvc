// A scene node's `repeat:` (§5.4): what it counts by, and its copies — see scene.h.
#include <cctype>
#include <cvc/ariadne/scene.h>
#include <string_view>

namespace cvc {
namespace ariadne {

namespace {

constexpr std::string_view kIndexToken = "{i}";

std::string with_index(std::string s, const std::string &idx) {
  for (std::size_t pos = s.find(kIndexToken); pos != std::string::npos;
       pos = s.find(kIndexToken, pos + idx.size()))
    s.replace(pos, kIndexToken.size(), idx);
  return s;
}

// Every scalar under `v`, map keys left as they are.
void value_with_index(Value &v, const std::string &idx) {
  switch (v.kind) {
  case Value::Kind::Scalar:
    v.scalar = with_index(std::move(v.scalar), idx);
    break;
  case Value::Kind::Sequence:
    for (Value &e : v.items)
      value_with_index(e, idx);
    break;
  case Value::Kind::Map:
    for (Value::Entry &e : v.entries)
      value_with_index(e.second, idx);
    break;
  }
}

std::string trimmed(const std::string &s) {
  std::size_t b = 0, e = s.size();
  while (b < e && std::isspace(static_cast<unsigned char>(s[b])))
    ++b;
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
    --e;
  return s.substr(b, e - b);
}

} // namespace

SceneRepeatCount scene_repeat_count(const std::string &repeat) {
  SceneRepeatCount c;
  const std::string t = trimmed(repeat);
  if (t.empty())
    return c;
  if (t[0] == '(') {
    c.kind = SceneRepeatCount::Kind::expression;
    c.text = t;
    return c;
  }
  // A whole number, signed: digits only after the sign.
  std::size_t k = (t[0] == '-' || t[0] == '+') ? 1 : 0;
  bool digits = k < t.size();
  for (std::size_t j = k; j < t.size(); ++j)
    digits = digits && std::isdigit(static_cast<unsigned char>(t[j]));
  if (digits) {
    c.kind = SceneRepeatCount::Kind::literal;
    long long n = 0;
    for (std::size_t j = k; j < t.size() && n <= kSceneRepeatMax; ++j)
      n = n * 10 + (t[j] - '0');
    c.literal = t[0] == '-' ? 0 : static_cast<int>(n > kSceneRepeatMax ? kSceneRepeatMax : n);
    return c;
  }
  c.kind = SceneRepeatCount::Kind::key;
  c.text = t;
  return c;
}

SceneNode scene_node_instance(const SceneNode &n, int index) {
  const std::string idx = std::to_string(index);
  SceneNode out = n;
  out.repeat.clear();
  out.id = with_index(std::move(out.id), idx);
  out.visible_bind = with_index(std::move(out.visible_bind), idx);
  for (std::string &t : out.tags)
    t = with_index(std::move(t), idx);
  out.source_file = with_index(std::move(out.source_file), idx);
  out.source_asset = with_index(std::move(out.source_asset), idx);
  out.source_data = with_index(std::move(out.source_data), idx);
  out.source_item = with_index(std::move(out.source_item), idx);
  out.material_texture = with_index(std::move(out.material_texture), idx);
  for (SceneClipPlane &p : out.clip.planes)
    p.offset_bind = with_index(std::move(p.offset_bind), idx);
  value_with_index(out.props, idx);
  out.children.clear();
  out.children.reserve(n.children.size());
  for (const SceneNode &c : n.children)
    out.children.push_back(c.repeat.empty() ? scene_node_instance(c, index) : c);
  return out;
}

} // namespace ariadne
} // namespace cvc
