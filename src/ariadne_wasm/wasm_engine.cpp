// wasm_engine.cpp — see the header. The wasm: block, a guest instance per document, its imports
// (the host's functions, and its objects by handle), and the activation that runs one export call
// on it.

#include "wasm_engine.h"

#include "guest_runtime.h"
#include "value_cbor.h"

#include <algorithm>
#include <boost/any.hpp>
#include <charconv>
#include <cmath>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h> // exec_scheduler
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h> // current_pid: the process a call runs in
#include <cvc/state/state_exec/data_job.h>        // kDataNodeBytes
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

// The DSL's state natives that write: state.write's. The other state- natives read.
const char *const kStateWriters[] = {
    "state-set",    "state-set-tree",  "state-data-set",     "state-from-json",
    "state-delete", "state-cas",       "state-swap",         "state-incr",
    "state-expire", "state-expire-at", "state-clear-expiry", "state-sweep-expired"};

// The groups a block's imports: names (§8.6), by the function's name.
const char *import_group(const std::string &f) {
  if (f.rfind("state-", 0) == 0)
    return std::find(std::begin(kStateWriters), std::end(kStateWriters), f) !=
                   std::end(kStateWriters)
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

// The object a data_object holds as a T, the shapes data: blocks and the state tree park
// (shared_ptr<const T> or shared_ptr<T>); null when it holds something else.
template <class T> std::shared_ptr<const T> held(const se::data_object_ptr &o) {
  if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&o->payload))
    return *p;
  if (const auto *p = boost::any_cast<std::shared_ptr<T>>(&o->payload))
    return *p;
  return nullptr;
}

// A host object for `vol`, as a data: block parks one (shared_ptr<const volume>), and as
// state-data-get would hand it out.
se::data_object_ptr volume_object(std::shared_ptr<const cvc::volume> vol) {
  auto obj = std::make_shared<se::data_object>();
  obj->payload = std::move(vol);
  obj->type_name = obj->payload.type().name();
  return obj;
}

// The host objects an instance holds, by handle (§8.4.1): the generation in the high 32 bits, so a
// dropped handle names nothing even once its slot is reused, and the slot plus one in the low. A
// generation stays below 2^31, so a handle is positive as an i64, and an import can return one.
class HandleTable : public Handles {
public:
  std::uint64_t put(const se::data_object_ptr &obj) override { return put(obj, nullptr); }

  // A handle for a volume the guest may write in place (`own`, which `obj` holds): one it made.
  std::uint64_t put(const se::data_object_ptr &obj, std::shared_ptr<cvc::volume> own) {
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
    slot.own = std::move(own);
    if (++slot.gen >= 0x80000000u)
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
      slots_[i].own.reset();
      free_.push_back(i);
    }
  }

  // The volume `handle` names, to write into: the guest's own. A version anyone else may hold (the
  // state tree, a renderer, another handle) is copied first, and the handle names the copy from
  // then on, so a write changes nothing another holder sees; the guest publishes it with
  // state_data_set. A volume only the guest holds is written in place. The copy shares the
  // voxels until they're written (cvc::voxels copies on write), so its first write copies them:
  // `copied` says it was made. Null when the handle names nothing, or no volume.
  cvc::volume *own_volume(std::uint64_t handle, bool &copied) {
    copied = false;
    Slot *slot = const_cast<Slot *>(find(handle));
    if (!slot)
      return nullptr;
    // Held by the slot alone: its object by the slot, and the volume by the slot and the object.
    if (slot->own && slot->obj.use_count() == 1 && slot->own.use_count() == 2)
      return slot->own.get();
    const std::shared_ptr<const cvc::volume> vol = held<cvc::volume>(slot->obj);
    if (!vol)
      return nullptr;
    auto own = std::make_shared<cvc::volume>(*vol);
    slot->obj = volume_object(own);
    slot->own = std::move(own);
    copied = true;
    return slot->own.get();
  }

private:
  struct Slot {
    se::data_object_ptr obj;
    std::shared_ptr<cvc::volume> own; // the volume obj holds, when the guest may write it in place
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
// on it (one at a time, under the block's lock: busy), through what they leave here.
struct Guest {
  std::unique_ptr<Instance> instance;
  cvc::app *app = nullptr; // whose scheduler runs its activations
  HandleTable handles;     // the host objects it holds; they go with it
  bool data_read = true;   // imports: lets the data calls through
  bool data_write = true;  // and the writers (volume_write, volume_new)
  std::uint64_t moved = 0; // bytes the data calls copied, for the activation's next charge
  std::string module;      // the module's name, for messages
  std::uint64_t gas_per_step = kGasPerStep;
  const void *busy = nullptr; // the activation whose call is on the instance
  int busy_pid = -1;          // the process it runs in
  std::string busy_name;      // the export it runs, for messages
  // What the call, yield or a state import asked for, for the activation: a call or a yield ends
  // the slice, and a state import's call is made in it (`now`, ScriptSlice::call_now).
  struct Request {
    bool yield = false;
    std::string function;
    std::vector<se::value_t> args;
    std::shared_ptr<PendingImport> pending;
    bool now = false;
  };
  std::optional<Request> request;
  std::string reply;   // the last call's CBOR result, or its error's text, for reply()
  std::string failure; // fail()'s text
};

std::span<std::uint8_t> bytes_of(std::string &s) {
  return {reinterpret_cast<std::uint8_t *>(s.data()), s.size()};
}
std::span<const std::uint8_t> bytes_of(std::string_view s) {
  return {reinterpret_cast<const std::uint8_t *>(s.data()), s.size()};
}

// Whether [at, at + n) lies inside the guest's memory; when not, a trap's message.
bool inside(GuestMemory &mem, std::int64_t at, std::uint64_t n, std::string &trap) {
  if (at >= 0 && mem.span_ok(static_cast<std::uint64_t>(at), n, trap))
    return true;
  if (trap.empty())
    trap = "an out-of-bounds pointer into the guest's memory";
  return false;
}

// The text of guest memory [ptr, ptr + len), or a trap's message.
bool read_guest(GuestMemory &mem, std::int64_t ptr, std::int64_t len, std::string &out,
                std::string &trap) {
  if (len < 0 || !inside(mem, ptr, static_cast<std::uint64_t>(len), trap)) {
    if (trap.empty())
      trap = "an out-of-bounds pointer into the guest's memory";
    return false;
  }
  out.resize(static_cast<std::size_t>(len));
  return mem.read(static_cast<std::uint64_t>(ptr), bytes_of(out));
}

// A data call's refusal: its text, "Class: message", waits for reply(), and the call returns minus
// its length.
std::int64_t refuse(Guest *g, std::string text) {
  g->reply = std::move(text);
  return -static_cast<std::int64_t>(g->reply.size());
}

// Whether the block's imports: let data call `what` through (data.read, or data.write for a
// writer); when not, the refusal is in `r`.
bool granted(Guest *g, const char *what, bool write, std::int64_t &r) {
  if (write ? g->data_write : g->data_read)
    return true;
  r = refuse(g, std::string("PermissionError: ari.") + what +
                    ": the wasm: block's imports: have no " + (write ? "data.write" : "data.read"));
  return false;
}

// The host object `h` names, for data call `what`; null, with the refusal in `r`, when it names
// nothing or the block's imports: have no data.read (data.write, for a writer).
se::data_object_ptr object(Guest *g, std::int64_t h, const char *what, std::int64_t &r,
                           bool write = false) {
  if (!granted(g, what, write, r))
    return nullptr;
  se::data_object_ptr o = g->handles.get(static_cast<std::uint64_t>(h));
  if (!o)
    r = refuse(g, std::string("ValueError: ari.") + what + ": handle " +
                      std::to_string(static_cast<std::uint64_t>(h)) +
                      " names nothing (it was dropped, or never given)");
  return o;
}

std::shared_ptr<const cvc::geometry> mesh_of(const se::data_object_ptr &o) {
  if (std::shared_ptr<const cvc::geometry> m = held<cvc::geometry>(o))
    return m;
  return held<cvc::height_field>(o); // a height field is a mesh too
}

// [dst, dst + n) of the guest's memory, for a data call to fill: the memory itself where the
// runtime maps it, else a buffer of the host's that commit() copies in.
class GuestOut {
public:
  // False, with a trap, when [dst, dst + n) isn't inside the memory.
  bool open(GuestMemory &mem, std::int64_t dst, std::uint64_t n, std::string &trap) {
    if (!inside(mem, dst, n, trap))
      return false;
    mem_ = &mem;
    at_ = static_cast<std::uint64_t>(dst);
    data_ = mem.map(at_, n);
    if (!data_) {
      staged_.resize(static_cast<std::size_t>(n));
      data_ = staged_.data();
    }
    return true;
  }
  std::uint8_t *data() const { return data_; }
  // What was written, into the guest's memory (where it was staged).
  void commit() {
    if (mem_ && !staged_.empty())
      mem_->write(at_, staged_);
  }

private:
  GuestMemory *mem_ = nullptr;
  std::uint64_t at_ = 0;
  std::uint8_t *data_ = nullptr;
  std::vector<std::uint8_t> staged_;
};

// [src, src + n) of the guest's memory, for a data call to read: the memory itself where the
// runtime maps it, else a copy.
class GuestIn {
public:
  // False, with a trap, when [src, src + n) isn't inside the memory.
  bool open(GuestMemory &mem, std::int64_t src, std::uint64_t n, std::string &trap) {
    if (!inside(mem, src, n, trap))
      return false;
    data_ = mem.map(static_cast<std::uint64_t>(src), n);
    if (!data_) {
      copy_.resize(static_cast<std::size_t>(n));
      mem.read(static_cast<std::uint64_t>(src), copy_);
      data_ = copy_.data();
    }
    return true;
  }
  const std::uint8_t *data() const { return data_; }

private:
  const std::uint8_t *data_ = nullptr;
  std::vector<std::uint8_t> copy_;
};

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

// The box a data call names, a[0..5] (x, y, z, nx, ny, nz), as its corner `at` and its size `n`;
// false, with the refusal in `r`, when it isn't inside `vol`.
bool box_of(Guest *g, const cvc::volume &vol, std::span<const std::int64_t> a, const char *what,
            std::uint64_t at[3], std::uint64_t n[3], std::int64_t &r) {
  const std::uint64_t dim[3] = {vol.XDim(), vol.YDim(), vol.ZDim()};
  for (int k = 0; k < 3; ++k) {
    if (!natural(a[k]) || a[3 + k] <= 0 ||
        static_cast<std::uint64_t>(a[k]) + static_cast<std::uint64_t>(a[3 + k]) > dim[k]) {
      r = refuse(g, std::string("ValueError: ari.") + what +
                        ": the box isn't inside the volume's " + std::to_string(dim[0]) + "x" +
                        std::to_string(dim[1]) + "x" + std::to_string(dim[2]) + " voxels");
      return false;
    }
    at[k] = static_cast<std::uint64_t>(a[k]);
    n[k] = static_cast<std::uint64_t>(a[3 + k]);
  }
  return true;
}

// `key` of a CBOR map the guest gave, or null.
const se::value_t *field(const se::value_t &map, const char *key) {
  if (const auto *d = std::get_if<se::dict_ptr>(&map.v); d && *d)
    for (const auto &[k, v] : **d)
      if (k == key)
        return &v;
  return nullptr;
}

// A list of `n` numbers in a map the guest gave (volume_new's dims and bounds), or false.
bool numbers(const se::value_t *v, std::size_t n, std::vector<double> &out) {
  const auto *l = v ? std::get_if<se::list_ptr>(&v->v) : nullptr;
  if (!l || !*l || (*l)->size() != n)
    return false;
  out.clear();
  for (const se::value_t &x : **l) {
    if (const auto *i = std::get_if<std::int64_t>(&x.v))
      out.push_back(static_cast<double>(*i));
    else if (const auto *f = std::get_if<double>(&x.v))
      out.push_back(*f);
    else
      return false;
  }
  return true;
}

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
    if (!box_of(g, *vol, a.subspan(1), "volume_read", at, n, r))
      return r;
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
    GuestOut out;
    if (!out.open(mem, a[8], bytes, trap))
      return 0;
    std::uint8_t *dst = out.data();
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
    out.commit();
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
    GuestIn in;
    if (!in.open(mem, a[1], n * 24, trap))
      return 0;
    std::vector<double> xyz(n * 3);
    std::memcpy(xyz.data(), in.data(), n * 24); // before any write: the two may overlap
    std::vector<double> v(n);
    try {
      for (std::uint64_t i = 0; i < n; ++i)
        v[i] = vol->interpolate(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    } catch (const std::exception &e) {
      return refuse(g, std::string("ValueError: ari.volume_sample: ") + e.what());
    }
    GuestOut out;
    if (!out.open(mem, a[3], n * 8, trap))
      return 0;
    std::memcpy(out.data(), v.data(), n * 8);
    out.commit();
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
    GuestOut out;
    if (!out.open(mem, a[5], bytes, trap))
      return 0;
    std::uint8_t *dst = out.data();
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
    out.commit();
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
    GuestOut out;
    if (!out.open(mem, a[5], bytes, trap))
      return 0;
    std::uint8_t *dst = out.data();
    const std::vector<float> &h = hf->heights();
    for (std::uint64_t i = 0; i < nr; ++i, dst += nc * 4)
      std::memcpy(dst, h.data() + (r0 + i) * cols + c0, nc * 4);
    out.commit();
    g->moved += bytes;
    return static_cast<std::int64_t>(bytes);
  };
  out.push_back(std::move(hread));

  // volume_write(h: i64, x, y, z, nx, ny, nz, src, len) -> i64: the box's voxels from [src, src +
  // len), x fastest, in the volume's own voxel type, into the guest's own copy of the volume
  // (HandleTable::own_volume); the bytes read. The node the volume came from changes only when the
  // guest stores the handle back (state_data_set).
  Import vwrite;
  vwrite.name = "volume_write";
  vwrite.params = 9;
  vwrite.wide = 1;
  vwrite.returns = true;
  vwrite.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                    std::string &trap) -> std::int64_t {
    std::int64_t r = 0;
    std::uint64_t at[3], n[3], vs = 0;
    {
      // Let go of these before the write: the slot's own volume is written in place only while
      // the slot alone holds it.
      const se::data_object_ptr o = object(g, a[0], "volume_write", r, true);
      if (!o)
        return r;
      const auto vol = held<cvc::volume>(o);
      if (!vol)
        return refuse(g, "TypeError: ari.volume_write: the handle's object is no volume");
      if (!box_of(g, *vol, a.subspan(1), "volume_write", at, n, r))
        return r;
      vs = vol->voxelSize();
    }
    const std::uint64_t bytes = n[0] * n[1] * n[2] * vs;
    if (!natural(a[8]) || bytes > static_cast<std::uint64_t>(a[8]))
      return refuse(g, "ValueError: ari.volume_write: the box is " + std::to_string(bytes) +
                           " bytes, and src holds " + std::to_string(a[8]));
    GuestIn in;
    if (!in.open(mem, a[7], bytes, trap))
      return 0;
    const std::uint8_t *src = in.data();
    bool fresh = false;
    cvc::volume *own = g->handles.own_volume(static_cast<std::uint64_t>(a[0]), fresh);
    if (!own)
      return refuse(g, "TypeError: ari.volume_write: the handle's object is no volume");
    const std::uint64_t dim[3] = {own->XDim(), own->YDim(), own->ZDim()};
    // The write's access copies voxels another volume shares (cvc::voxels copies on write), as a
    // fresh copy's are: a copy no bigger than what the guest was handed, noted as a native's is.
    unsigned char *voxels = **own;
    const std::uint64_t copied = fresh ? dim[0] * dim[1] * dim[2] * vs : 0;
    se::eval_alloc_note(copied);
    for (std::uint64_t k = 0; k < n[2]; ++k)
      for (std::uint64_t j = 0; j < n[1]; ++j, src += n[0] * vs)
        std::memcpy(voxels + (((at[2] + k) * dim[1] + at[1] + j) * dim[0] + at[0]) * vs, src,
                    n[0] * vs);
    own->unsetMinMax(); // its min and max are found again from the voxels
    g->moved += bytes + copied;
    return static_cast<std::int64_t>(bytes);
  };
  out.push_back(std::move(vwrite));

  // volume_new(spec, spec_len) -> i64: a new volume, zeroed, as a handle (> 0) the guest may write
  // in place. spec is a CBOR map as data_info gives one: dims [nx, ny, nz]; voxel, a type name
  // (unsigned char, unsigned short, unsigned int, float or double; unsigned char when absent);
  // bounds [x0, y0, z0, x1, y1, z1] (the volume's default when absent). Its bytes count against
  // the call's allocation limit, as a native's allocation does.
  Import vnew;
  vnew.name = "volume_new";
  vnew.params = 2;
  vnew.returns = true;
  vnew.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                  std::string &trap) -> std::int64_t {
    std::int64_t r = 0;
    if (!granted(g, "volume_new", true, r))
      return r;
    std::string text;
    if (!read_guest(mem, a[0], a[1], text, trap))
      return 0;
    se::value_t spec;
    try {
      spec = from_cbor(text, &g->handles);
    } catch (const std::exception &e) {
      return refuse(g, std::string("ValueError: ari.volume_new: ") + e.what());
    }
    std::vector<double> dims, bounds;
    if (!numbers(field(spec, "dims"), 3, dims))
      return refuse(g, "ValueError: ari.volume_new: dims is a list of three sizes");
    std::uint64_t size[3];
    for (int k = 0; k < 3; ++k) {
      if (!(dims[k] >= 1.0 && dims[k] <= double(1u << 20)) || dims[k] != std::floor(dims[k]))
        return refuse(g, "ValueError: ari.volume_new: a size is a whole number of voxels, from 1 "
                         "to 2^20");
      size[k] = static_cast<std::uint64_t>(dims[k]);
    }
    cvc::data_type type = cvc::UChar;
    if (const se::value_t *v = field(spec, "voxel")) {
      const auto *name = std::get_if<std::string>(&v->v);
      int t = cvc::UChar;
      while (t <= cvc::Double && !(name && *name == cvc::data_type_strings[t]))
        ++t;
      if (t > cvc::Double)
        return refuse(g, "ValueError: ari.volume_new: voxel is unsigned char, unsigned short, "
                         "unsigned int, float or double");
      type = static_cast<cvc::data_type>(t);
    }
    std::optional<cvc::bounding_box> box;
    if (const se::value_t *b = field(spec, "bounds")) {
      if (!numbers(b, 6, bounds))
        return refuse(g, "ValueError: ari.volume_new: bounds is a list of six numbers");
      try {
        box.emplace(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);
      } catch (const std::exception &e) {
        return refuse(g, std::string("ValueError: ari.volume_new: bounds: ") + e.what());
      }
    }
    const std::uint64_t voxels = size[0] * size[1] * size[2];
    if (voxels > (std::uint64_t(1) << 40))
      return refuse(g, "ValueError: ari.volume_new: the volume is past 2^40 voxels");
    const std::uint64_t bytes = voxels * cvc::data_type_sizes[type];
    std::shared_ptr<cvc::volume> vol;
    try {
      se::eval_alloc_check(static_cast<std::size_t>(bytes), "ari.volume_new");
      vol = std::make_shared<cvc::volume>(*g->app, cvc::dimension(size[0], size[1], size[2]), type);
    } catch (const std::exception &e) {
      return refuse(g, std::string("MemoryError: ari.volume_new: ") + e.what());
    }
    if (box)
      vol->boundingBox(*box);
    std::uint64_t h = 0;
    try {
      h = g->handles.put(volume_object(vol), vol);
    } catch (const std::exception &e) {
      return refuse(g, std::string("ValueError: ari.volume_new: ") + e.what());
    }
    g->moved += bytes;
    return static_cast<std::int64_t>(h);
  };
  out.push_back(std::move(vnew));
  return out;
}

// The state imports (§8.4): fast paths beside call, each for one of the lane's own state natives,
// called in the slice that asks for it (ScriptSlice::call_now) where call ends the slice for the
// activation's loop to make the call. So a guest's state_get is what (state-get path) is in a
// program there: the same chroot, link: holes and imports: narrowing. Each takes its path as text
// and its values as CBOR, an empty one nil, and returns as call does.
struct StateImport {
  const char *name;   // the import, in module "ari"
  const char *native; // the function it calls
  int values;         // CBOR values after the path
  bool optional;      // an empty value is left out (state-incr's delta: 1)
};

constexpr StateImport kStateImports[] = {
    {"state_get", "state-get", 0, false},           {"state_exists", "state-exists", 0, false},
    {"state_children", "state-children", 0, false}, {"state_data_get", "state-data-get", 0, false},
    {"state_set", "state-set", 1, false},           {"state_data_set", "state-data-set", 1, false},
    {"state_delete", "state-delete", 0, false},     {"state_swap", "state-swap", 1, false},
    {"state_incr", "state-incr", 1, true},          {"state_cas", "state-cas", 2, false}};

std::vector<Import> state_imports(Guest *g) {
  std::vector<Import> out;
  for (const StateImport &si : kStateImports) {
    Import imp;
    imp.name = si.name;
    imp.params = 2 + 2 * si.values;
    imp.returns = true;
    imp.async = [g, si](GuestMemory &mem, std::span<const std::int64_t> a,
                        std::string &trap) -> std::shared_ptr<PendingImport> {
      if (!g->busy) {
        trap = std::string("ari.") + si.name +
               ": no activation is running (a guest's setup can't call the host)";
        return nullptr;
      }
      std::string path;
      if (!read_guest(mem, a[0], a[1], path, trap))
        return nullptr;
      std::vector<se::value_t> args{se::value_t(std::move(path))};
      auto pending = std::make_shared<PendingImport>();
      for (int i = 0; i < si.values; ++i) {
        std::string cbor;
        if (!read_guest(mem, a[2 + 2 * i], a[3 + 2 * i], cbor, trap))
          return nullptr;
        if (cbor.empty()) {
          if (!si.optional)
            args.emplace_back();
          continue;
        }
        try {
          args.push_back(from_cbor(cbor, &g->handles));
        } catch (const std::exception &e) {
          pending->done = true;
          pending->value = refuse(g, std::string("ValueError: ari.") + si.name + ": " + e.what());
          return pending;
        }
      }
      g->request = Guest::Request{false, si.native, std::move(args), pending, true};
      return pending;
    };
    out.push_back(std::move(imp));
  }
  return out;
}

std::vector<Import> imports_of(Guest *g) {
  std::vector<Import> out = data_imports(g);
  for (Import &imp : state_imports(g))
    out.push_back(std::move(imp));
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
    g->request = Guest::Request{false, std::move(name), **items, pending};
    return pending;
  };
  out.push_back(std::move(call));

  Import reply;
  reply.name = "reply";
  reply.params = 1;
  reply.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::int64_t {
    if (inside(mem, a[0], g->reply.size(), trap))
      mem.write(static_cast<std::uint64_t>(a[0]), bytes_of(g->reply));
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
      // One activation per instance: the block's lock (ActivationOptions::lock) holds the others
      // back, so the instance is busy here only in the process that holds the lock, which a call
      // from inside one of the guest's own host calls is granted again.
      if (g_->busy && g_->busy != this) {
        if (g_->busy_pid >= 0 && g_->busy_pid == g_->app->exec_scheduler().current_pid())
          throw se::dsl_error("RecursionError",
                              g_->module + ": " + name_ + ": the guest is running '" +
                                  g_->busy_name +
                                  "', and this call came from inside it: a call can't re-enter "
                                  "the guest it came from",
                              "wasm", {{"running", se::value_t(g_->busy_name)}});
        if (!s.may_wait()) // a pure call from the read lane, which can't wait its turn
          throw se::dsl_error("RuntimeError",
                              g_->module + ": " + name_ + ": the guest is running '" +
                                  g_->busy_name + "', and a pure call can't wait for it",
                              "wasm", {{"running", se::value_t(g_->busy_name)}});
        s.yield_frame(); // held outside the lock: the next frame, once it is free
        return;
      }
      begin(s);
    }
    if (s.has_reply() && g_->request && !g_->request->yield)
      answer(s);
    const std::uint64_t gas = g_->gas_per_step ? g_->gas_per_step : 1;
    // The call runs until the slice's budget is spent or it stops: a state import's call is made
    // here, a step each, and the guest goes on in the same slice.
    for (std::uint64_t left = std::max<std::uint64_t>(s.budget(), 1);;) {
      const Call::State state = call_->poll(left * gas);
      const std::uint64_t spent = (call_->fuel_used() + gas - 1) / gas + moved_steps();
      s.charge(spent);
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
        if (g_->request->now) {
          s.charge(1);
          if (s.call_now(g_->request->function, g_->request->args)) {
            answer(s);
            if (spent + 1 >= left)
              return; // the slice is spent: the guest goes on in the next
            left -= spent + 1;
            continue;
          }
        }
        s.call(g_->request->function, g_->request->args); // the slice ends for the loop's call
        return;
      case Call::State::sliced:
        return; // more to do: the next slice
      }
    }
  }

  void cancel() override { release(); }

private:
  void begin(ScriptSlice &s) {
    g_->busy = this;
    g_->busy_pid = g_->app->exec_scheduler().current_pid();
    g_->busy_name = name_;
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
    GuestMemory &mem = g_->instance->memory();
    if (std::string why; !inside(mem, ptr, cbor.size(), why))
      raise("ari_alloc gave room outside the guest's memory");
    mem.write(static_cast<std::uint64_t>(ptr), bytes_of(cbor));
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
    GuestMemory &mem = g_->instance->memory();
    std::string text;
    if (std::string why; !read_guest(mem, ptr, len, text, why))
      fail("its result points outside the guest's memory");
    try {
      return from_cbor(text, &g_->handles);
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
      g_->busy_pid = -1;
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
    // 0: the runtime's own (the browser's room for a paused stack; wasmtime has no use for one).
    limits.unwind_bytes = count_setting(root, "ariadne.limits.wasm_unwind_bytes", 0);
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
    g->app = &lane.app();
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
    g->data_write = !groups || groups->count("data.write") != 0;
    if (const Value *p = b.find("priority"))
      options.priority = static_cast<int>(p->as_double(0.0));
    const std::string stem = path.stem().string();
    // The instance's lock, under the document's prefix: its activations wait for each other as
    // programs wait in with-lock (§8.5). A key holds no '#'.
    options.lock = "wasm:" + stem;
    std::replace(options.lock.begin(), options.lock.end(), '#', '_');
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
    };
    const bool calls = b.find("export") || b.find("hooks") || g->instance->has_export("ari_init");
    if (calls && (!g->instance->has_export("ari_alloc") || g->instance->memory().size() == 0))
      throw std::runtime_error("wasm: " + g->module +
                               " exports no ari_alloc or memory, so no value can cross to it");

    if (const Value *ex = b.find("export")) {
      std::vector<Value> entries = ex->is_seq() ? ex->items : std::vector<Value>{*ex};
      for (const Value &e : entries) {
        const std::string name =
            e.is_map() ? (e.find("name") ? e.find("name")->as_string() : "") : e.as_string();
        if (name.empty())
          throw std::runtime_error("wasm: export: an entry is a name or {name:, as:, pure:}");
        const std::string as = e.is_map() && e.find("as") ? e.find("as")->as_string() : name;
        needs(name, "export:");
        ActivationOptions o = options;
        o.name = "wasm/" + stem + ":" + name;
        const auto run = runner(name);
        const ScriptCall call = [run](std::span<const se::value_t> args) {
          return run(se::make_list(std::vector<se::value_t>(args.begin(), args.end())));
        };
        lane.export_activation(as, call, o);
        // pure: true makes it a function of the read lane too (§8.6): reactive expressions call it.
        if (e.is_map())
          if (const Value *pure = e.find("pure"); pure && pure->as_bool(false))
            lane.export_pure(as, call, o);
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
      if (const Value *k = hooks->find("on_key"))
        lane.on_key(hook(k->as_string(), "hooks: on_key"), named("on_key"));
      if (const Value *p = hooks->find("on_pointer"))
        lane.on_pointer(hook(p->as_string(), "hooks: on_pointer"), named("on_pointer"));
      if (const Value *s = hooks->find("on_signal"))
        lane.on_signal(hook(s->as_string(), "hooks: on_signal"), named("on_signal"));
      if (const Value *m = hooks->find("msg_recv")) {
        if (!m->is_map())
          throw std::runtime_error("wasm: hooks: msg_recv is a map of channel: export");
        for (const Value::Entry &e : m->entries)
          lane.on_message(e.first, hook(e.second.as_string(), "hooks: msg_recv"),
                          named("msg_recv"));
      }
      static const char *const kHooks[] = {"on_tick",    "on",        "watch",   "on_key",
                                           "on_pointer", "on_signal", "msg_recv"};
      for (const Value::Entry &e : hooks->entries)
        if (std::find(std::begin(kHooks), std::end(kHooks), e.first) == std::end(kHooks))
          warnings.push_back("wasm: hooks: '" + e.first +
                             "' is not a hook (on_tick, on, watch, on_key, on_pointer, on_signal, "
                             "msg_recv); ignored");
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
    // The time's count is an __int128 in Emscripten's libc++, which to_string takes no overload of.
    const std::string key = std::to_string(fs::file_size(path, ec)) + ":" +
                            std::to_string(static_cast<long long>(
                                fs::last_write_time(path, ec).time_since_epoch().count()));
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
