// wasm_engine.cpp — see the header. The wasm: block, a guest instance per document, its imports
// (the host's functions, and its objects by handle), and the activation that runs one export call
// on it.

#include "wasm_engine.h"

#include "guest_runtime.h"
#include "value_cbor.h"

#include <boost/any.hpp>
#include <charconv>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/value.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/data_job.h> // kDataNodeBytes
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/volume/volume.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

namespace se = cvc::state_exec;
namespace fs = std::filesystem;

constexpr std::uint64_t kGasPerStep = 1000;                    // ariadne.limits.wasm_gas_per_step
constexpr std::uint64_t kMaxMemory = std::uint64_t(256) << 20; // ariadne.limits.wasm_max_memory
constexpr std::uint64_t kSetupFuel = 100'000'000; // _initialize, ari_alloc: the guest's own setup
constexpr std::size_t kMaxHandles = 65536;        // live handles per instance

const char *const kKeys[] = {"module", "export", "hooks", "imports", "limits", "priority"};

std::uint64_t count_setting(cvc::state &root, const char *key, std::uint64_t fallback) {
  cvc::state *node = root.findDescendant(key);
  if (!node)
    return fallback;
  const std::string v = node->value();
  std::uint64_t n = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
  return !v.empty() && ec == std::errc() && end == v.data() + v.size() ? n : fallback;
}

// The groups a block's imports: names (§8.6), by the function's name.
const char *import_group(const std::string &f) {
  if (f.rfind("state-", 0) == 0)
    return f.find("set") != std::string::npos || f.find("delete") != std::string::npos
               ? "state.write"
               : "state.read";
  if (f.rfind("msg-", 0) == 0)
    return "msg";
  if (f.rfind("http-", 0) == 0 || f.rfind("ws-", 0) == 0 || f.rfind("net-", 0) == 0)
    return "net";
  return "call";
}

// The block's imports: groups; none without imports:, which narrows nothing.
std::optional<std::set<std::string>> import_groups(const Value *imports) {
  if (!imports)
    return std::nullopt;
  std::set<std::string> groups;
  const auto add = [&](const std::string &g) {
    static const std::set<std::string> known = {"state.read", "state.write", "msg",       "call",
                                                "net",        "data.read",   "data.write"};
    if (!known.count(g))
      throw std::runtime_error("wasm: imports: '" + g +
                               "' is no group (state.read, state.write, msg, call, net, "
                               "data.read, data.write)");
    groups.insert(g);
  };
  if (imports->is_seq())
    for (const Value &v : imports->items)
      add(v.as_string());
  else if (!imports->as_string().empty())
    add(imports->as_string());
  return groups;
}

// The groups as a narrowing of what call() reaches; null: no narrowing.
std::function<bool(const std::string &)>
narrowing(const std::optional<std::set<std::string>> &groups) {
  if (!groups)
    return nullptr;
  return [g = *groups](const std::string &f) { return g.count(import_group(f)) != 0; };
}

std::int64_t packed(std::uint32_t ptr, std::uint32_t len) {
  return static_cast<std::int64_t>((static_cast<std::uint64_t>(ptr) << 32) | len);
}

// The host objects an instance holds, by handle (§8.4.1): the generation in the high 32 bits, so a
// dropped handle names nothing even once its slot is reused, and the slot plus one in the low.
class HandleTable : public Handles {
public:
  std::uint64_t put(const se::data_object_ptr &obj) override {
    std::uint32_t i = 0;
    if (!free_.empty()) {
      i = free_.back();
      free_.pop_back();
    } else {
      if (slots_.size() >= kMaxHandles)
        throw std::invalid_argument("the guest holds " + std::to_string(kMaxHandles) +
                                    " handles, and drops some (handle_drop) before it takes more");
      i = static_cast<std::uint32_t>(slots_.size());
      slots_.emplace_back();
    }
    Slot &slot = slots_[i];
    slot.obj = obj;
    if (++slot.gen == 0)
      slot.gen = 1;
    return (static_cast<std::uint64_t>(slot.gen) << 32) | (i + 1u);
  }

  se::data_object_ptr get(std::uint64_t handle) const override {
    const Slot *slot = find(handle);
    return slot ? slot->obj : nullptr;
  }

  void drop(std::uint64_t handle) {
    if (const Slot *slot = find(handle)) {
      const auto i = static_cast<std::uint32_t>(slot - slots_.data());
      slots_[i].obj.reset();
      free_.push_back(i);
    }
  }

private:
  struct Slot {
    se::data_object_ptr obj;
    std::uint32_t gen = 0;
  };
  const Slot *find(std::uint64_t handle) const {
    const std::uint64_t i = handle & 0xffffffffu;
    if (i == 0 || i > slots_.size())
      return nullptr;
    const Slot &slot = slots_[i - 1];
    return slot.obj && slot.gen == static_cast<std::uint32_t>(handle >> 32) ? &slot : nullptr;
  }
  std::vector<Slot> slots_;
  std::vector<std::uint32_t> free_;
};

// One instance of a block's module, for one document. Its imports reach the activation running
// on it (one at a time: busy), through what they leave here.
struct Guest {
  std::unique_ptr<Instance> instance;
  HandleTable handles;     // the host objects it holds; they go with it
  bool data_read = true;   // imports: lets the data calls through
  std::uint64_t moved = 0; // bytes the data calls copied, for the activation's next charge
  std::string module;      // the module's name, for messages
  std::uint64_t gas_per_step = kGasPerStep;
  std::set<std::string> exports; // the guest's own exports, which its calls may not re-enter
  const void *busy = nullptr;    // the activation whose call is on the instance
  // What the call or yield import asked for, for the activation's next slice.
  struct Request {
    bool yield = false;
    std::string function;
    std::vector<se::value_t> args;
    std::shared_ptr<PendingImport> pending;
  };
  std::optional<Request> request;
  std::string reply;   // the last call's CBOR result, or its error's text, for reply()
  std::string failure; // fail()'s text
};

// The text of guest memory [ptr, ptr + len), or a trap's message.
bool read_guest(GuestMemory &mem, std::int64_t ptr, std::int64_t len, std::string &out,
                std::string &trap) {
  if (ptr < 0 || len < 0 ||
      !mem.span_ok(static_cast<std::uint64_t>(ptr), static_cast<std::uint64_t>(len), trap)) {
    if (trap.empty())
      trap = "an out-of-bounds pointer into the guest's memory";
    return false;
  }
  const std::span<std::uint8_t> b = mem.bytes();
  out.assign(reinterpret_cast<const char *>(b.data()) + ptr, static_cast<std::size_t>(len));
  return true;
}

// A data call's refusal: its text, "Class: message", waits for reply(), and the call returns minus
// its length.
std::int64_t refuse(Guest *g, std::string text) {
  g->reply = std::move(text);
  return -static_cast<std::int64_t>(g->reply.size());
}

// The host object `h` names, for data call `what`; null, with the refusal in `r`, when it names
// nothing or the block's imports: have no data.read.
se::data_object_ptr object(Guest *g, std::int64_t h, const char *what, std::int64_t &r) {
  if (!g->data_read) {
    r = refuse(g, std::string("PermissionError: ari.") + what +
                      ": the wasm: block's imports: have no data.read");
    return nullptr;
  }
  se::data_object_ptr o = g->handles.get(static_cast<std::uint64_t>(h));
  if (!o)
    r = refuse(g, std::string("ValueError: ari.") + what + ": handle " +
                      std::to_string(static_cast<std::uint64_t>(h)) +
                      " names nothing (it was dropped, or never given)");
  return o;
}

// The object a data_object holds as a T, the shapes data: blocks and the state tree park
// (shared_ptr<const T> or shared_ptr<T>); null when it holds something else.
template <class T> std::shared_ptr<const T> held(const se::data_object_ptr &o) {
  if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&o->payload))
    return *p;
  if (const auto *p = boost::any_cast<std::shared_ptr<T>>(&o->payload))
    return *p;
  return nullptr;
}

std::shared_ptr<const cvc::geometry> mesh_of(const se::data_object_ptr &o) {
  if (std::shared_ptr<const cvc::geometry> m = held<cvc::geometry>(o))
    return m;
  return held<cvc::height_field>(o); // a height field is a mesh too
}

// [dst, dst + n) in the guest's memory, for a data call's copy; false, with a trap, when it isn't.
std::uint8_t *room(GuestMemory &mem, std::int64_t dst, std::uint64_t n, std::string &trap) {
  if (dst < 0 || !mem.span_ok(static_cast<std::uint64_t>(dst), n, trap)) {
    if (trap.empty())
      trap = "an out-of-bounds pointer into the guest's memory";
    return nullptr;
  }
  return mem.bytes().data() + dst;
}

se::value_t bounds_list(const cvc::bounding_box &b) {
  return se::make_list({se::value_t(b.minx), se::value_t(b.miny), se::value_t(b.minz),
                        se::value_t(b.maxx), se::value_t(b.maxy), se::value_t(b.maxz)});
}

se::value_t count(std::uint64_t n) { return se::value_t(static_cast<std::int64_t>(n)); }

// data_info's map: what an object is, and its sizes, with none of its data.
se::value_t info_of(const se::data_object_ptr &o) {
  if (const auto vol = held<cvc::volume>(o))
    return se::make_dict(
        {{"type", se::value_t(std::string("volume"))},
         {"dims", se::make_list({count(vol->XDim()), count(vol->YDim()), count(vol->ZDim())})},
         {"voxel", se::value_t(std::string(vol->voxelTypeStr()))},
         {"voxel_bytes", count(vol->voxelSize())},
         {"bounds", bounds_list(vol->boundingBox())}});
  if (const auto hf = held<cvc::height_field>(o))
    return se::make_dict({{"type", se::value_t(std::string("heightfield"))},
                          {"rows", count(static_cast<std::uint64_t>(hf->rows()))},
                          {"cols", count(static_cast<std::uint64_t>(hf->cols()))},
                          {"x0", se::value_t(hf->x0())},
                          {"y0", se::value_t(hf->y0())},
                          {"dx", se::value_t(hf->dx())},
                          {"dy", se::value_t(hf->dy())}});
  if (const auto m = held<cvc::geometry>(o)) {
    std::vector<std::pair<std::string, se::value_t>> d = {
        {"type", se::value_t(std::string("mesh"))}, {"points", count(m->num_points())},
        {"normals", count(m->normals().size())},    {"tris", count(m->num_tris())},
        {"quads", count(m->num_quads())},           {"lines", count(m->num_lines())}};
    if (m->num_points() > 0)
      d.emplace_back("bounds", bounds_list(m->extents()));
    return se::make_dict(std::move(d));
  }
  return se::make_dict(
      {{"type", se::value_t(std::string("object"))}, {"name", se::value_t(o->type_name)}});
}

// An i32 argument that is a count or an offset: false when it is negative.
bool natural(std::int64_t v) { return v >= 0; }

std::vector<Import> data_imports(Guest *g) {
  std::vector<Import> out;

  Import drop; // handle_drop(h: i64): let a handle go; one that names nothing is ignored
  drop.name = "handle_drop";
  drop.params = 1;
  drop.wide = 1;
  drop.sync = [g](GuestMemory &, std::span<const std::int64_t> a, std::string &) -> std::int64_t {
    g->handles.drop(static_cast<std::uint64_t>(a[0]));
    return 0;
  };
  out.push_back(std::move(drop));

  Import info; // data_info(h: i64) -> i64: the CBOR map's length, for reply()
  info.name = "data_info";
  info.params = 1;
  info.wide = 1;
  info.returns = true;
  info.sync = [g](GuestMemory &, std::span<const std::int64_t> a, std::string &) -> std::int64_t {
    std::int64_t r = 0;
    const se::data_object_ptr o = object(g, a[0], "data_info", r);
    if (!o)
      return r;
    g->reply = to_cbor(info_of(o));
    return static_cast<std::int64_t>(g->reply.size());
  };
  out.push_back(std::move(info));

  // volume_read(h: i64, x, y, z, nx, ny, nz, lod, dst, len) -> i64: the box's voxels, every
  // 2^lod-th along each axis, x fastest, in the volume's own voxel type, into [dst, dst + len); the
  // bytes written.
  Import vread;
  vread.name = "volume_read";
  vread.params = 10;
  vread.wide = 1;
  vread.returns = true;
  vread.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::int64_t {
    std::int64_t r = 0;
    const se::data_object_ptr o = object(g, a[0], "volume_read", r);
    if (!o)
      return r;
    const auto vol = held<cvc::volume>(o);
    if (!vol)
      return refuse(g, "TypeError: ari.volume_read: the handle's object is no volume");
    const std::uint64_t dim[3] = {vol->XDim(), vol->YDim(), vol->ZDim()};
    std::uint64_t at[3], n[3];
    for (int k = 0; k < 3; ++k) {
      if (!natural(a[1 + k]) || a[4 + k] <= 0 ||
          static_cast<std::uint64_t>(a[1 + k]) + static_cast<std::uint64_t>(a[4 + k]) > dim[k])
        return refuse(g, "ValueError: ari.volume_read: the box isn't inside the volume's " +
                             std::to_string(dim[0]) + "x" + std::to_string(dim[1]) + "x" +
                             std::to_string(dim[2]) + " voxels");
      at[k] = static_cast<std::uint64_t>(a[1 + k]);
      n[k] = static_cast<std::uint64_t>(a[4 + k]);
    }
    if (a[7] < 0 || a[7] > 30)
      return refuse(g, "ValueError: ari.volume_read: lod is from 0 to 30");
    const std::uint64_t step = std::uint64_t(1) << a[7];
    const std::uint64_t out_n[3] = {(n[0] + step - 1) / step, (n[1] + step - 1) / step,
                                    (n[2] + step - 1) / step};
    const std::uint64_t vs = vol->voxelSize();
    const std::uint64_t bytes = out_n[0] * out_n[1] * out_n[2] * vs;
    if (!natural(a[9]) || bytes > static_cast<std::uint64_t>(a[9]))
      return refuse(g, "ValueError: ari.volume_read: the region is " + std::to_string(bytes) +
                           " bytes, and dst holds " + std::to_string(a[9]));
    std::uint8_t *dst = room(mem, a[8], bytes, trap);
    if (!dst)
      return 0;
    const unsigned char *src = vol->data_ptr();
    for (std::uint64_t k = 0; k < out_n[2]; ++k)
      for (std::uint64_t j = 0; j < out_n[1]; ++j) {
        const std::uint64_t z = at[2] + k * step, y = at[1] + j * step;
        const unsigned char *row = src + ((z * dim[1] + y) * dim[0] + at[0]) * vs;
        if (step == 1) {
          std::memcpy(dst, row, out_n[0] * vs);
          dst += out_n[0] * vs;
        } else {
          for (std::uint64_t i = 0; i < out_n[0]; ++i, dst += vs)
            std::memcpy(dst, row + i * step * vs, vs);
        }
      }
    g->moved += bytes;
    return static_cast<std::int64_t>(bytes);
  };
  out.push_back(std::move(vread));

  // volume_sample(h: i64, points, n, dst, len) -> i64: the volume's value, interpolated, at each
  // of n points (f64 x, y, z in object coordinates, at `points`), as f64s into [dst, dst + len).
  Import sample;
  sample.name = "volume_sample";
  sample.params = 5;
  sample.wide = 1;
  sample.returns = true;
  sample.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                    std::string &trap) -> std::int64_t {
    std::int64_t r = 0;
    const se::data_object_ptr o = object(g, a[0], "volume_sample", r);
    if (!o)
      return r;
    const auto vol = held<cvc::volume>(o);
    if (!vol)
      return refuse(g, "TypeError: ari.volume_sample: the handle's object is no volume");
    if (!natural(a[2]))
      return refuse(g, "ValueError: ari.volume_sample: n is a count of points");
    const auto n = static_cast<std::uint64_t>(a[2]);
    if (!natural(a[4]) || n * 8 > static_cast<std::uint64_t>(a[4]))
      return refuse(g, "ValueError: ari.volume_sample: " + std::to_string(n) + " values are " +
                           std::to_string(n * 8) + " bytes, and dst holds " + std::to_string(a[4]));
    const std::uint8_t *in = room(mem, a[1], n * 24, trap);
    if (!in)
      return 0;
    std::vector<double> xyz(n * 3);
    std::memcpy(xyz.data(), in, n * 24); // before any write: the two may overlap
    std::vector<double> v(n);
    try {
      for (std::uint64_t i = 0; i < n; ++i)
        v[i] = vol->interpolate(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    } catch (const std::exception &e) {
      return refuse(g, std::string("ValueError: ari.volume_sample: ") + e.what());
    }
    std::uint8_t *dst = room(mem, a[3], n * 8, trap);
    if (!dst)
      return 0;
    std::memcpy(dst, v.data(), n * 8);
    g->moved += n * 32;
    return static_cast<std::int64_t>(n * 8);
  };
  out.push_back(std::move(sample));

  // mesh_read(h: i64, attr, attr_len, first, count, dst, len) -> i64: elements [first, first +
  // count) of an attribute, into [dst, dst + len): positions or normals (f64 x, y, z), indices
  // (the triangles, u32 x 3), quads (u32 x 4) or lines (u32 x 2).
  Import mread;
  mread.name = "mesh_read";
  mread.params = 7;
  mread.wide = 1;
  mread.returns = true;
  mread.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::int64_t {
    std::int64_t r = 0;
    const se::data_object_ptr o = object(g, a[0], "mesh_read", r);
    if (!o)
      return r;
    const auto m = mesh_of(o);
    if (!m)
      return refuse(g, "TypeError: ari.mesh_read: the handle's object is no mesh");
    std::string attr;
    if (!read_guest(mem, a[1], a[2], attr, trap))
      return 0;
    std::uint64_t size = 0, width = 0; // elements, and scalars per element
    bool real = false;
    if (attr == "positions") {
      size = m->points().size(), width = 3, real = true;
    } else if (attr == "normals") {
      size = m->normals().size(), width = 3, real = true;
    } else if (attr == "indices") {
      size = m->tris().size(), width = 3;
    } else if (attr == "quads") {
      size = m->quads().size(), width = 4;
    } else if (attr == "lines") {
      size = m->lines().size(), width = 2;
    } else {
      return refuse(g, "ValueError: ari.mesh_read: '" + attr +
                           "' is no attribute (positions, normals, indices, quads, lines)");
    }
    if (!natural(a[3]) || !natural(a[4]) ||
        static_cast<std::uint64_t>(a[3]) + static_cast<std::uint64_t>(a[4]) > size)
      return refuse(g, "ValueError: ari.mesh_read: the range isn't inside the mesh's " +
                           std::to_string(size) + " " + attr);
    const auto first = static_cast<std::uint64_t>(a[3]), n = static_cast<std::uint64_t>(a[4]);
    const std::uint64_t bytes = n * width * (real ? 8 : 4);
    if (!natural(a[6]) || bytes > static_cast<std::uint64_t>(a[6]))
      return refuse(g, "ValueError: ari.mesh_read: the range is " + std::to_string(bytes) +
                           " bytes, and dst holds " + std::to_string(a[6]));
    std::uint8_t *dst = room(mem, a[5], bytes, trap);
    if (!dst)
      return 0;
    if (real) {
      const auto &v = attr == "positions" ? m->points() : m->normals();
      for (std::uint64_t i = 0; i < n; ++i, dst += 24)
        std::memcpy(dst, v[first + i].data(), 24);
    } else {
      const auto put = [&](const auto &elems) -> bool {
        for (std::uint64_t i = 0; i < n; ++i)
          for (const auto idx : elems[first + i]) {
            if (idx > std::numeric_limits<std::uint32_t>::max())
              return false;
            const auto u = static_cast<std::uint32_t>(idx);
            std::memcpy(dst, &u, 4);
            dst += 4;
          }
        return true;
      };
      const bool fits = attr == "indices" ? put(m->tris())
                        : attr == "quads" ? put(m->quads())
                                          : put(m->lines());
      if (!fits)
        return refuse(g, "ValueError: ari.mesh_read: an index is past u32");
    }
    g->moved += bytes;
    return static_cast<std::int64_t>(bytes);
  };
  out.push_back(std::move(mread));

  // heightfield_read(h: i64, row, col, rows, cols, dst, len) -> i64: the heights of a rectangle of
  // samples, row by row, as f32s (NaN: a hole), into [dst, dst + len).
  Import hread;
  hread.name = "heightfield_read";
  hread.params = 7;
  hread.wide = 1;
  hread.returns = true;
  hread.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::int64_t {
    std::int64_t r = 0;
    const se::data_object_ptr o = object(g, a[0], "heightfield_read", r);
    if (!o)
      return r;
    const auto hf = held<cvc::height_field>(o);
    if (!hf)
      return refuse(g, "TypeError: ari.heightfield_read: the handle's object is no height field");
    const auto rows = static_cast<std::uint64_t>(hf->rows()),
               cols = static_cast<std::uint64_t>(hf->cols());
    if (!natural(a[1]) || !natural(a[2]) || a[3] <= 0 || a[4] <= 0 ||
        static_cast<std::uint64_t>(a[1]) + static_cast<std::uint64_t>(a[3]) > rows ||
        static_cast<std::uint64_t>(a[2]) + static_cast<std::uint64_t>(a[4]) > cols)
      return refuse(g, "ValueError: ari.heightfield_read: the rectangle isn't inside the field's " +
                           std::to_string(rows) + "x" + std::to_string(cols) + " samples");
    const auto r0 = static_cast<std::uint64_t>(a[1]), c0 = static_cast<std::uint64_t>(a[2]),
               nr = static_cast<std::uint64_t>(a[3]), nc = static_cast<std::uint64_t>(a[4]);
    const std::uint64_t bytes = nr * nc * 4;
    if (!natural(a[6]) || bytes > static_cast<std::uint64_t>(a[6]))
      return refuse(g, "ValueError: ari.heightfield_read: the rectangle is " +
                           std::to_string(bytes) + " bytes, and dst holds " + std::to_string(a[6]));
    std::uint8_t *dst = room(mem, a[5], bytes, trap);
    if (!dst)
      return 0;
    const std::vector<float> &h = hf->heights();
    for (std::uint64_t i = 0; i < nr; ++i, dst += nc * 4)
      std::memcpy(dst, h.data() + (r0 + i) * cols + c0, nc * 4);
    g->moved += bytes;
    return static_cast<std::int64_t>(bytes);
  };
  out.push_back(std::move(hread));
  return out;
}

std::vector<Import> imports_of(Guest *g) {
  std::vector<Import> out = data_imports(g);
  Import call;
  call.name = "call";
  call.params = 4;
  call.returns = true;
  call.async = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::shared_ptr<PendingImport> {
    if (!g->busy) {
      trap = "ari.call: no activation is running (a guest's setup can't call the host)";
      return nullptr;
    }
    std::string name, args;
    if (!read_guest(mem, a[0], a[1], name, trap) || !read_guest(mem, a[2], a[3], args, trap))
      return nullptr;
    auto pending = std::make_shared<PendingImport>();
    se::value_t list;
    try {
      list = args.empty() ? se::make_list() : from_cbor(args, &g->handles);
    } catch (const std::exception &e) {
      g->reply = std::string("ValueError: ") + e.what();
      pending->done = true;
      pending->value = -static_cast<std::int64_t>(g->reply.size());
      return pending;
    }
    const auto *items = std::get_if<se::list_ptr>(&list.v);
    if (!items || !*items) {
      g->reply = "TypeError: ari.call: the arguments are a CBOR array";
      pending->done = true;
      pending->value = -static_cast<std::int64_t>(g->reply.size());
      return pending;
    }
    if (g->exports.count(name)) { // a call back into this instance would wait on itself
      g->reply = "RuntimeError: ari.call: '" + name +
                 "' is this guest's own export, and a call can't re-enter the guest it came from";
      pending->done = true;
      pending->value = -static_cast<std::int64_t>(g->reply.size());
      return pending;
    }
    g->request = Guest::Request{false, std::move(name), **items, pending};
    return pending;
  };
  out.push_back(std::move(call));

  Import reply;
  reply.name = "reply";
  reply.params = 1;
  reply.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::int64_t {
    if (a[0] < 0 || !mem.span_ok(static_cast<std::uint64_t>(a[0]), g->reply.size(), trap)) {
      if (trap.empty())
        trap = "an out-of-bounds pointer into the guest's memory";
      return 0;
    }
    std::memcpy(mem.bytes().data() + a[0], g->reply.data(), g->reply.size());
    return 0;
  };
  out.push_back(std::move(reply));

  Import yield;
  yield.name = "yield";
  yield.async = [g](GuestMemory &, std::span<const std::int64_t>,
                    std::string &) -> std::shared_ptr<PendingImport> {
    if (!g->busy)
      return nullptr; // nothing to give the frame back to: go on
    auto pending = std::make_shared<PendingImport>();
    g->request = Guest::Request{true, {}, {}, pending};
    return pending;
  };
  out.push_back(std::move(yield));

  Import fail;
  fail.name = "fail";
  fail.params = 2;
  fail.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                  std::string &trap) -> std::int64_t {
    std::string text;
    if (read_guest(mem, a[0], a[1], text, trap)) {
      g->failure = text.empty() ? "fail" : text;
      trap = g->failure;
    }
    return 0;
  };
  out.push_back(std::move(fail));
  return out;
}

// One call of an export on a guest, as an activation (script_engine.h): its arguments go in
// through ari_alloc, and each step polls the call for one slice's fuel.
class GuestCall : public ScriptActivation {
public:
  GuestCall(std::shared_ptr<Guest> g, std::string name, se::value_t args)
      : g_(std::move(g)), name_(std::move(name)), args_(std::move(args)) {}
  ~GuestCall() override { release(); }

  void step(ScriptSlice &s) override {
    if (!call_) {
      if (g_->busy && g_->busy != this) {
        s.yield_frame(); // one activation per instance: the next frame, once it is free
        return;
      }
      begin(s);
    }
    if (s.has_reply() && g_->request && !g_->request->yield)
      answer(s);
    const std::uint64_t gas = g_->gas_per_step ? g_->gas_per_step : 1;
    const Call::State state = call_->poll(s.budget() * gas);
    s.charge((call_->fuel_used() + gas - 1) / gas + moved_steps());
    switch (state) {
    case Call::State::done:
      s.finish(result());
      release();
      return;
    case Call::State::trapped:
      if (!g_->failure.empty()) // the guest's own fail(): its text, as it said it
        fail(g_->failure);
      raise(call_->trap());
    case Call::State::waiting:
      if (!g_->request) {
        s.yield_frame();
        return;
      }
      if (g_->request->yield) {
        g_->request->pending->done = true;
        g_->request.reset();
        s.yield_frame();
        return;
      }
      s.call(g_->request->function, g_->request->args);
      return;
    case Call::State::sliced:
      return; // more to do: the next slice
    }
  }

  void cancel() override { release(); }

private:
  void begin(ScriptSlice &s) {
    g_->busy = this;
    g_->failure.clear();
    g_->moved = 0;
    // A value that can't cross raises a ValueError; a host object crosses as a handle.
    const std::string cbor = to_cbor(args_, &g_->handles);
    std::int64_t ptr = 0;
    std::uint64_t used = 0;
    std::string error;
    if (!g_->instance->run("ari_alloc", {static_cast<std::int64_t>(cbor.size())}, kSetupFuel, ptr,
                           used, error))
      raise("ari_alloc: " + error);
    const std::span<std::uint8_t> mem = g_->instance->memory();
    if (ptr < 0 || static_cast<std::uint64_t>(ptr) > mem.size() ||
        cbor.size() > mem.size() - static_cast<std::uint64_t>(ptr))
      raise("ari_alloc gave room outside the guest's memory");
    std::memcpy(mem.data() + ptr, cbor.data(), cbor.size());
    call_ = g_->instance->start(name_, {ptr, static_cast<std::int64_t>(cbor.size())}, error);
    if (!call_)
      fail(error);
    const std::uint64_t gas = g_->gas_per_step ? g_->gas_per_step : 1;
    s.charge((used + gas - 1) / gas);
  }

  // The last slice's call came back: its CBOR, or its error's text, waits for reply().
  void answer(ScriptSlice &s) {
    std::shared_ptr<PendingImport> pending = g_->request->pending;
    g_->request.reset();
    if (s.failed()) {
      g_->reply = s.error();
      pending->value = -static_cast<std::int64_t>(g_->reply.size());
    } else {
      try {
        g_->reply = to_cbor(s.reply(), &g_->handles);
        pending->value = static_cast<std::int64_t>(g_->reply.size());
      } catch (const std::exception &e) {
        g_->reply = std::string("TypeError: ") + e.what();
        pending->value = -static_cast<std::int64_t>(g_->reply.size());
      }
    }
    pending->done = true;
  }

  se::value_t result() {
    const auto r = static_cast<std::uint64_t>(call_->result());
    const auto ptr = static_cast<std::uint32_t>(r >> 32);
    const auto len = static_cast<std::uint32_t>(r & 0xffffffffu);
    if (len == 0)
      return se::value_t();
    const std::span<std::uint8_t> mem = g_->instance->memory();
    if (ptr > mem.size() || len > mem.size() - ptr)
      fail("its result points outside the guest's memory");
    try {
      return from_cbor(std::string_view(reinterpret_cast<const char *>(mem.data()) + ptr, len),
                       &g_->handles);
    } catch (const std::exception &e) {
      fail(std::string("its result isn't CBOR: ") + e.what());
    }
    return se::value_t();
  }

  // `what` by value: it may be the call's trap(), which release() ends.
  [[noreturn]] void fail(std::string what) {
    release();
    throw se::dsl_error("WasmError", g_->module + ": " + name_ + ": " + what, "wasm",
                        {{"trap", se::value_t(what)}});
  }

  // A trap: a MemoryError when the guest's memory is at its limit (a memory.grow was refused, and
  // the guest gave up), else a WasmError.
  [[noreturn]] void raise(std::string what) {
    if (!g_->instance->memory_at_limit())
      fail(what);
    const std::uint64_t size = g_->instance->memory().size();
    release();
    throw se::dsl_error(
        "MemoryError",
        g_->module + ": " + name_ + ": out of memory, at its limit of " +
            std::to_string(size >> 20) + " MiB (" + what + ")",
        "wasm",
        {{"trap", se::value_t(what)}, {"memory", se::value_t(static_cast<std::int64_t>(size))}});
  }

  // The steps the data calls' copies cost since the last charge: a node of work per
  // kDataNodeBytes, as data_job charges.
  std::uint64_t moved_steps() {
    const std::uint64_t steps = (g_->moved + se::kDataNodeBytes - 1) / se::kDataNodeBytes;
    g_->moved = 0;
    return steps;
  }

  void release() {
    call_.reset();
    if (g_ && g_->busy == this) {
      g_->busy = nullptr;
      g_->request.reset();
    }
  }

  std::shared_ptr<Guest> g_;
  std::string name_;
  se::value_t args_;
  std::unique_ptr<Call> call_;
};

std::string read_file(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  if (!in)
    throw std::runtime_error("wasm: module: can't read " + p.string());
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

class WasmEngine : public ScriptEngine {
public:
  explicit WasmEngine(std::unique_ptr<Runtime> rt) : runtime_(std::move(rt)) {}

  std::string language() const override { return "wasm"; }

  void load_block(const ScriptBlock &block, ScriptLane &lane,
                  std::vector<std::string> &warnings) override {
    Value b = block.content;
    if (b.is_scalar()) { // the module alone
      Value m;
      m.kind = Value::Kind::Map;
      m.entries.emplace_back("module", b);
      b = std::move(m);
    }
    if (!b.is_map())
      throw std::runtime_error("wasm: is a module's path, or a map of module:, export:, hooks:, "
                               "imports:, limits:, priority:");
    for (const Value::Entry &e : b.entries) {
      bool known = false;
      for (const char *k : kKeys)
        known = known || e.first == k;
      if (!known)
        warnings.push_back("wasm: unknown key '" + e.first +
                           "' ignored (it takes module, export, hooks, imports, limits, priority)");
    }
    const std::string module = b.find("module") ? b.find("module")->as_string() : std::string();
    if (module.empty())
      throw std::runtime_error("wasm: module: names the guest's .wasm (or .wat) file");
    if (module.rfind("cvcpkg:", 0) == 0)
      throw std::runtime_error("wasm: module: cvcpkg: packages come later (phase G5); name a "
                               "file beside the document");
    fs::path path(module);
    if (path.is_relative() && !block.dir.empty())
      path = fs::path(block.dir) / path;
    std::shared_ptr<Module> compiled = compile(path);

    cvc::state &root = cvc::state::instance(lane.app());
    InstanceLimits limits;
    limits.max_memory = count_setting(root, "ariadne.limits.wasm_max_memory", kMaxMemory);
    if (const Value *l = b.find("limits"))
      if (const Value *mb = l->find("memory_mb")) {
        const double v = mb->as_double(-1.0);
        if (v < 0)
          throw std::runtime_error("wasm: limits: memory_mb is a number of MiB");
        const auto bytes = static_cast<std::uint64_t>(v * 1024.0 * 1024.0);
        if (limits.max_memory == 0 || bytes < limits.max_memory)
          limits.max_memory = bytes; // a block may lower the host's cap, never raise it
      }

    auto g = std::make_shared<Guest>();
    g->module = path.filename().string();
    g->gas_per_step = count_setting(root, "ariadne.limits.wasm_gas_per_step", kGasPerStep);
    std::string error;
    g->instance = runtime_->instantiate(compiled, imports_of(g.get()), limits, error);
    if (!g->instance)
      throw std::runtime_error("wasm: " + g->module + ": " + error);
    if (g->instance->has_export("_initialize")) {
      std::int64_t ignored = 0;
      std::uint64_t used = 0;
      if (!g->instance->run("_initialize", {}, kSetupFuel, ignored, used, error))
        throw std::runtime_error("wasm: " + g->module + ": _initialize: " + error);
    }

    ActivationOptions options;
    const std::optional<std::set<std::string>> groups = import_groups(b.find("imports"));
    options.may_call = narrowing(groups);
    g->data_read = !groups || groups->count("data.read") != 0;
    if (const Value *p = b.find("priority"))
      options.priority = static_cast<int>(p->as_double(0.0));
    const std::string stem = path.stem().string();
    const std::weak_ptr<Guest> weak = g;
    // What runs `name` on the guest with `args`, as long as the document's instance is there.
    const auto runner = [weak](const std::string &name) {
      return [weak, name](se::value_t args) -> std::shared_ptr<ScriptActivation> {
        std::shared_ptr<Guest> guest = weak.lock();
        if (!guest)
          throw std::runtime_error(name + ": its guest has gone (the document was reloaded)");
        return std::make_shared<GuestCall>(std::move(guest), name, std::move(args));
      };
    };
    const auto needs = [&](const std::string &name, const char *what) {
      if (!g->instance->has_export(name))
        throw std::runtime_error("wasm: " + g->module + " exports no function '" + name + "' (" +
                                 what + ")");
      g->exports.insert(name);
    };
    const bool calls = b.find("export") || b.find("hooks") || g->instance->has_export("ari_init");
    if (calls && (!g->instance->has_export("ari_alloc") || g->instance->memory().empty()))
      throw std::runtime_error("wasm: " + g->module +
                               " exports no ari_alloc or memory, so no value can cross to it");

    if (const Value *ex = b.find("export")) {
      std::vector<Value> entries = ex->is_seq() ? ex->items : std::vector<Value>{*ex};
      for (const Value &e : entries) {
        const std::string name =
            e.is_map() ? (e.find("name") ? e.find("name")->as_string() : "") : e.as_string();
        if (name.empty())
          throw std::runtime_error("wasm: export: an entry is a name or {name:, as:}");
        const std::string as = e.is_map() && e.find("as") ? e.find("as")->as_string() : name;
        needs(name, "export:");
        ActivationOptions o = options;
        o.name = "wasm/" + stem + ":" + name;
        const auto run = runner(name);
        lane.export_activation(
            as,
            [run](std::span<const se::value_t> args) {
              return run(se::make_list(std::vector<se::value_t>(args.begin(), args.end())));
            },
            o);
      }
    }

    if (const Value *hooks = b.find("hooks")) {
      const auto hook = [&](const std::string &name, const std::string &what) {
        needs(name, what.c_str());
        const auto run = runner(name);
        return ScriptHook([run](const se::value_t &event) { return run(se::make_list({event})); });
      };
      const auto named = [&](const char *kind) {
        ActivationOptions o = options;
        o.name = "wasm/" + stem + ":" + kind;
        return o;
      };
      if (const Value *t = hooks->find("on_tick"))
        lane.on_frame(hook(t->as_string(), "hooks: on_tick"), named("on_tick"));
      if (const Value *on = hooks->find("on"))
        for (const Value::Entry &e : on->entries)
          lane.on_event(e.first, hook(e.second.as_string(), "hooks: on"), named("on"));
      if (const Value *w = hooks->find("watch"))
        for (const Value::Entry &e : w->entries)
          lane.on_watch(e.first, hook(e.second.as_string(), "hooks: watch"), named("watch"));
      for (const Value::Entry &e : hooks->entries)
        if (e.first != "on_tick" && e.first != "on" && e.first != "watch")
          warnings.push_back("wasm: hooks: '" + e.first +
                             "' is not a hook yet (on_tick, on, watch); ignored");
    }
    // ari_init, the guest's own setup through the host (§8.5): an activation run to its end now,
    // under the init: lane's caps, before the document's init: runs.
    if (g->instance->has_export("ari_init")) {
      ActivationOptions o = options;
      o.name = "wasm/" + stem + ":ari_init";
      o.lane = ProgramLane::init;
      const ActivationOutcome out =
          lane.run(std::make_shared<GuestCall>(g, "ari_init", se::make_list()), std::move(o));
      if (!out.ok)
        throw std::runtime_error("wasm: " + g->module + ": ari_init " + out.error);
    }
    guests_[&lane] = std::move(g);
  }

  void unload(ScriptLane &lane) override { guests_.erase(&lane); }

private:
  // The module at `path`, compiled once while the file stays the same.
  std::shared_ptr<Module> compile(const fs::path &path) {
    std::error_code ec;
    const std::string key =
        std::to_string(fs::file_size(path, ec)) + ":" +
        std::to_string(fs::last_write_time(path, ec).time_since_epoch().count());
    Compiled &c = modules_[fs::absolute(path, ec).string()];
    if (c.module && c.key == key)
      return c.module;
    const std::string bytes = read_file(path);
    std::string error;
    std::shared_ptr<Module> m = runtime_->compile(bytes, path.extension() == ".wat", error);
    if (!m)
      throw std::runtime_error("wasm: " + path.filename().string() + ": " + error);
    c = Compiled{key, m};
    return m;
  }

  struct Compiled {
    std::string key;
    std::shared_ptr<Module> module;
  };
  std::unique_ptr<Runtime> runtime_;
  std::map<std::string, Compiled> modules_;
  std::map<ScriptLane *, std::shared_ptr<Guest>> guests_;
};

} // namespace

std::shared_ptr<ScriptEngine> make_engine(std::string &error) {
  std::unique_ptr<Runtime> rt = make_runtime(error);
  if (!rt)
    return nullptr;
  return std::make_shared<WasmEngine>(std::move(rt));
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
