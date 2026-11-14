// meter.cpp — the gas pass (meter.h; ARIADNE_SCRIPTING_HOSTS.md §8.5).
//
// The module is read section by section. The sections that name function indices are rewritten,
// and so is the code, a function body at a time: each instruction is copied as it was, apart from
// a call's or ref.func's function index, and a metered block's instructions are held until the
// block ends, when its charge (meter.h) goes in before them.

#include "meter.h"

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

using namespace std::string_view_literals;

struct Bad : std::runtime_error {
  using std::runtime_error::runtime_error;
};

std::string hex(unsigned v) {
  static const char digits[] = "0123456789abcdef";
  std::string s = "0x";
  bool any = false;
  for (int shift = 28; shift >= 0; shift -= 4) {
    const unsigned d = (v >> shift) & 0xf;
    if (d || any || shift < 8) {
      s.push_back(digits[d]);
      any = true;
    }
  }
  return s;
}

// A cursor over part of the module, which refuses to read past its end.
class In {
public:
  In(const std::uint8_t *b, const std::uint8_t *e) : p_(b), e_(e) {}
  explicit In(std::string_view s)
      : In(reinterpret_cast<const std::uint8_t *>(s.data()),
           reinterpret_cast<const std::uint8_t *>(s.data()) + s.size()) {}

  bool done() const { return p_ == e_; }
  const std::uint8_t *at() const { return p_; }
  std::uint8_t byte() {
    need(1);
    return *p_++;
  }
  std::uint8_t peek() const {
    need(1);
    return *p_;
  }
  void skip(std::size_t n) {
    need(n);
    p_ += n;
  }
  std::string_view take(std::size_t n) {
    need(n);
    std::string_view s(reinterpret_cast<const char *>(p_), n);
    p_ += n;
    return s;
  }
  In sub(std::size_t n) {
    need(n);
    In r(p_, p_ + n);
    p_ += n;
    return r;
  }
  std::string_view rest() { return take(static_cast<std::size_t>(e_ - p_)); }
  std::string_view since(const std::uint8_t *from) const {
    return {reinterpret_cast<const char *>(from), static_cast<std::size_t>(p_ - from)};
  }

  // An unsigned LEB128 number of at most `bits` bits.
  std::uint64_t u(unsigned bits) {
    std::uint64_t r = 0;
    for (unsigned shift = 0;; shift += 7) {
      if (shift >= bits)
        throw Bad("a LEB128 number is too long");
      const std::uint64_t b = byte();
      if (bits - shift < 7 && ((b & 0x7f) >> (bits - shift)) != 0)
        throw Bad("a LEB128 number is too large");
      r |= (b & 0x7f) << shift;
      if (!(b & 0x80))
        return r;
    }
  }
  std::uint32_t u32() { return static_cast<std::uint32_t>(u(32)); }
  // A signed LEB128 number of at most `bits` bits, skipped: only where it ends matters here.
  void s(unsigned bits) {
    const unsigned most = (bits + 6) / 7;
    for (unsigned i = 0;; ++i) {
      if (i == most)
        throw Bad("a LEB128 number is too long");
      if (!(byte() & 0x80))
        return;
    }
  }
  std::string_view name() { return take(u32()); }

private:
  void need(std::size_t n) const {
    if (static_cast<std::size_t>(e_ - p_) < n)
      throw Bad("the module ends early");
  }
  const std::uint8_t *p_, *e_;
};

void put_u(std::string &o, std::uint64_t v) {
  do {
    std::uint8_t b = v & 0x7f;
    v >>= 7;
    if (v)
      b |= 0x80;
    o.push_back(static_cast<char>(b));
  } while (v);
}

void put_s(std::string &o, std::int64_t v) {
  for (;;) {
    const std::uint8_t b = static_cast<std::uint8_t>(v & 0x7f);
    v >>= 7; // arithmetic (C++20)
    if ((v == 0 && !(b & 0x40)) || (v == -1 && (b & 0x40))) {
      o.push_back(static_cast<char>(b));
      return;
    }
    o.push_back(static_cast<char>(b | 0x80));
  }
}

void put_name(std::string &o, std::string_view s) {
  put_u(o, s.size());
  o.append(s);
}

void put_section(std::string &o, std::uint8_t id, const std::string &body) {
  o.push_back(static_cast<char>(id));
  put_u(o, body.size());
  o += body;
}

// What the module has, and where the pass's own global and import go.
struct Layout {
  std::uint32_t func_imports = 0; // the gas import's index; defined functions shift past it
  std::uint32_t globals = 0;      // imported and defined: the gas global's index
  std::uint32_t types = 0;        // the gas import's type's index

  std::uint32_t func(std::uint32_t f) const {
    if (f < func_imports)
      return f;
    if (f == UINT32_MAX)
      throw Bad("a function index out of range");
    return f + 1;
  }
  std::uint32_t global(std::uint32_t g) const {
    if (g >= globals) // the gas global's, or past it: the module would gain access it didn't have
      throw Bad("a global index out of range (" + std::to_string(g) + ")");
    return g;
  }
};

void valtype(In &in) {
  const std::uint8_t b = in.byte();
  if (b == 0x63 || b == 0x64) // (ref null ht), (ref ht)
    in.s(33);
}

void blocktype(In &in) {
  const std::uint8_t b = in.peek();
  if (b == 0x63 || b == 0x64) {
    in.byte();
    in.s(33);
  } else {
    in.s(33); // 0x40 (empty), a value type's byte, or a type index
  }
}

void limits(In &in) {
  const std::uint32_t flags = in.u32();
  in.u(64);
  if (flags & 1)
    in.u(64);
  if (flags & 8) // a custom page size
    in.u32();
}

void memarg(In &in) {
  if (in.u32() & 0x40) // a memory index follows
    in.u32();
  in.u(64);
}

enum class Flow {
  plain,
  open,        // block, try_table: entered only from the instruction before
  opens_block, // loop, if: a metered block starts after it
  else_,       // the then-body ends, the else-body starts
  end,
  branch_if // br_if, br_on_null, br_on_non_null: what follows is a metered block
};

struct Op {
  Flow flow = Flow::plain;
  std::uint32_t cost = 1;
};

void prefixed_fc(In &in) {
  const std::uint32_t sub = in.u32();
  switch (sub) {
  case 0: // 0-7: the saturating truncations
  case 1:
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
    return;
  case 8:  // memory.init data memory
  case 10: // memory.copy memory memory
  case 12: // table.init elem table
  case 14: // table.copy table table
    in.u32();
    in.u32();
    return;
  case 9:  // data.drop
  case 11: // memory.fill
  case 13: // elem.drop
  case 15: // table.grow
  case 16: // table.size
  case 17: // table.fill
    in.u32();
    return;
  default:
    throw Bad("an instruction the gas pass doesn't know (0xfc " + std::to_string(sub) + ")");
  }
}

void prefixed_fd(In &in) {
  const std::uint32_t sub = in.u32();
  if (sub <= 11 || sub == 92 || sub == 93) // the loads and v128.store, load32_zero, load64_zero
    memarg(in);
  else if (sub == 12 || sub == 13) // v128.const, i8x16.shuffle
    in.skip(16);
  else if (sub >= 21 && sub <= 34) // extract_lane, replace_lane
    in.skip(1);
  else if (sub >= 84 && sub <= 91) { // the lane loads and stores
    memarg(in);
    in.skip(1);
  } else if (sub > 0x113) // past relaxed SIMD
    throw Bad("an instruction the gas pass doesn't know (0xfd " + std::to_string(sub) + ")");
}

void prefixed_fe(In &in) {
  const std::uint32_t sub = in.u32();
  if (sub == 3) // atomic.fence
    in.skip(1);
  else if (sub <= 2 || (sub >= 0x10 && sub <= 0x4e)) // notify, wait, the atomic loads, stores, rmw
    memarg(in);
  else
    throw Bad("an instruction the gas pass doesn't know (0xfe " + std::to_string(sub) + ")");
}

// Read one instruction and copy it to `out`, with its function or global index as the metered
// module numbers it.
Op instruction(In &in, std::string &out, const Layout &l) {
  const std::uint8_t *start = in.at();
  const std::uint8_t op = in.byte();
  Op r;
  switch (op) {
  case 0x00: // unreachable
  case 0x01: // nop
  case 0x0f: // return
  case 0x1a: // drop
    r.cost = 0;
    break;
  case 0x02: // block
    blocktype(in);
    r = {Flow::open, 0};
    break;
  case 0x03: // loop
    blocktype(in);
    r = {Flow::opens_block, 0};
    break;
  case 0x04: // if
    blocktype(in);
    r.flow = Flow::opens_block;
    break;
  case 0x05:
    r = {Flow::else_, 0};
    break;
  case 0x0b:
    r = {Flow::end, 0};
    break;
  case 0x08: // throw
  case 0x0c: // br
  case 0x14: // call_ref
  case 0x15: // return_call_ref
  case 0x20: // local.get
  case 0x21: // local.set
  case 0x22: // local.tee
  case 0x25: // table.get
  case 0x26: // table.set
  case 0x3f: // memory.size
  case 0x40: // memory.grow
    in.u32();
    break;
  case 0x0a: // throw_ref
  case 0x1b: // select
  case 0xd1: // ref.is_null
  case 0xd3: // ref.eq
  case 0xd4: // ref.as_non_null
    break;
  case 0x0d: // br_if
  case 0xd5: // br_on_null
  case 0xd6: // br_on_non_null
    in.u32();
    r.flow = Flow::branch_if;
    break;
  case 0x0e: { // br_table
    const std::uint32_t n = in.u32();
    for (std::uint64_t i = 0; i <= n; ++i)
      in.u32();
    break;
  }
  case 0x10: // call
  case 0x12: // return_call
  case 0xd2: // ref.func
    out.push_back(static_cast<char>(op));
    put_u(out, l.func(in.u32()));
    return r;
  case 0x23: // global.get
  case 0x24: // global.set
    out.push_back(static_cast<char>(op));
    put_u(out, l.global(in.u32()));
    return r;
  case 0x11: // call_indirect
  case 0x13: // return_call_indirect
    in.u32();
    in.u32();
    break;
  case 0x1c: { // select with types
    const std::uint32_t n = in.u32();
    for (std::uint32_t i = 0; i < n; ++i)
      valtype(in);
    break;
  }
  case 0x1f: { // try_table
    blocktype(in);
    const std::uint32_t n = in.u32();
    for (std::uint32_t i = 0; i < n; ++i) {
      switch (in.byte()) {
      case 0: // catch tag label
      case 1: // catch_ref tag label
        in.u32();
        in.u32();
        break;
      case 2: // catch_all label
      case 3: // catch_all_ref label
        in.u32();
        break;
      default:
        throw Bad("a try_table catch of a kind the gas pass doesn't know");
      }
    }
    r.flow = Flow::open;
    break;
  }
  case 0x41:
    in.s(32);
    break;
  case 0x42:
    in.s(64);
    break;
  case 0x43:
    in.skip(4);
    break;
  case 0x44:
    in.skip(8);
    break;
  case 0xd0: // ref.null
    in.s(33);
    break;
  case 0xfc:
    prefixed_fc(in);
    break;
  case 0xfd:
    prefixed_fd(in);
    break;
  case 0xfe:
    prefixed_fe(in);
    break;
  default:
    if (op >= 0x28 && op <= 0x3e) { // the loads and stores
      memarg(in);
      break;
    }
    if (op >= 0x45 && op <= 0xc4) // the numeric instructions, sign extension among them
      break;
    if (op == 0xfb)
      throw Bad("a GC instruction (0xfb), which the gas pass doesn't read");
    if (op == 0x06 || op == 0x07 || op == 0x09 || op == 0x18 || op == 0x19)
      throw Bad("the legacy exception handling (" + hex(op) +
                "), which the gas pass doesn't read: try_table is the standard's");
    throw Bad("an instruction the gas pass doesn't know (" + hex(op) + ")");
  }
  out.append(in.since(start));
  return r;
}

// A constant expression: a global's value, a segment's offset, an element.
void const_expr(In &in, std::string &out, const Layout &l) {
  for (;;) {
    const Op op = instruction(in, out, l);
    if (op.flow == Flow::end)
      return;
    if (op.flow != Flow::plain)
      throw Bad("a constant expression with control flow");
  }
}

void charge(std::string &o, const Layout &l, std::uint64_t cost) {
  const auto c = static_cast<std::int64_t>(cost);
  o.push_back(0x23); // global.get ari_gas
  put_u(o, l.globals);
  o.push_back(0x42); // i64.const C
  put_s(o, c);
  o.push_back(0x54); // i64.lt_u
  o.push_back(0x04); // if
  o.push_back(0x40);
  o.push_back(0x42); // i64.const C
  put_s(o, c);
  o.push_back(0x10); // call ari.gas
  put_u(o, l.func_imports);
  o.push_back(0x0b); // end
  o.push_back(0x23); // global.get ari_gas
  put_u(o, l.globals);
  o.push_back(0x42); // i64.const C
  put_s(o, c);
  o.push_back(0x7d); // i64.sub
  o.push_back(0x24); // global.set ari_gas
  put_u(o, l.globals);
}

// A function body, metered.
std::string body(In in, const Layout &l, std::uint64_t &charges) {
  const std::uint8_t *locals = in.at();
  const std::uint32_t groups = in.u32();
  for (std::uint32_t i = 0; i < groups; ++i) {
    in.u32();
    valtype(in);
  }
  std::string out(in.since(locals));
  std::string block; // the metered block's instructions, held until it ends
  std::uint64_t cost = 0;
  auto flush = [&] {
    if (cost) {
      charge(out, l, cost);
      ++charges;
    }
    out += block;
    block.clear();
    cost = 0;
  };
  for (std::uint64_t depth = 1; depth > 0;) {
    const Op op = instruction(in, block, l);
    cost += op.cost;
    switch (op.flow) {
    case Flow::plain:
      break;
    case Flow::open:
      ++depth;
      break;
    case Flow::opens_block:
      ++depth;
      flush();
      break;
    case Flow::else_:
    case Flow::branch_if:
      flush();
      break;
    case Flow::end:
      --depth;
      flush();
      break;
    }
  }
  if (!in.done())
    throw Bad("a function body goes on past its end");
  return out;
}

// The name section, with the function indices it names shifted. A section the pass can't read is
// dropped: names are only for people.
bool names(In in, std::string &o, const Layout &l) {
  try {
    std::string out;
    while (!in.done()) {
      const std::uint8_t id = in.byte();
      In sub = in.sub(in.u32());
      std::string s;
      if (id == 1) { // function names
        const std::uint32_t n = sub.u32();
        put_u(s, n);
        for (std::uint32_t i = 0; i < n; ++i) {
          put_u(s, l.func(sub.u32()));
          put_name(s, sub.name());
        }
      } else if (id == 2 || id == 3) { // local names, label names: by function
        const std::uint32_t n = sub.u32();
        put_u(s, n);
        for (std::uint32_t i = 0; i < n; ++i) {
          put_u(s, l.func(sub.u32()));
          const std::uint32_t m = sub.u32();
          put_u(s, m);
          for (std::uint32_t j = 0; j < m; ++j) {
            put_u(s, sub.u32());
            put_name(s, sub.name());
          }
        }
      } else {
        s.assign(sub.rest());
      }
      if (!sub.done())
        return false;
      out.push_back(static_cast<char>(id));
      put_u(out, s.size());
      out += s;
    }
    o += out;
    return true;
  } catch (const Bad &) {
    return false;
  }
}

void elements(In &in, std::string &o, const Layout &l) {
  const std::uint32_t n = in.u32();
  put_u(o, n);
  for (std::uint32_t i = 0; i < n; ++i) {
    const std::uint32_t flags = in.u32();
    if (flags > 7)
      throw Bad("an element segment of a kind the gas pass doesn't know");
    put_u(o, flags);
    if ((flags & 3) == 2) // an active segment's table
      put_u(o, in.u32());
    if (!(flags & 1)) // an active segment's offset
      const_expr(in, o, l);
    if (flags & 4) { // its elements as expressions
      if (flags != 4) {
        const std::uint8_t *t = in.at();
        valtype(in);
        o.append(in.since(t));
      }
      const std::uint32_t m = in.u32();
      put_u(o, m);
      for (std::uint32_t j = 0; j < m; ++j)
        const_expr(in, o, l);
    } else { // as function indices
      if (flags != 0)
        o.push_back(static_cast<char>(in.byte())); // the element kind
      const std::uint32_t m = in.u32();
      put_u(o, m);
      for (std::uint32_t j = 0; j < m; ++j)
        put_u(o, l.func(in.u32()));
    }
  }
  if (!in.done())
    throw Bad("the element section goes on past its segments");
}

// Where a section goes in a module: its id's place in the standard order.
int place(std::uint8_t id) {
  static const int order[14] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 11, 6};
  return order[id];
}

std::string gas_type() { return std::string("\x60\x01\x7e\x00", 4); } // (func (param i64))

std::string gas_import(const Layout &l) {
  std::string s;
  put_name(s, kGasModule);
  put_name(s, kGasImport);
  s.push_back(0x00);
  put_u(s, l.types);
  return s;
}

// (mut i64) (i64.const kStartGas)
std::string gas_global() {
  std::string s("\x7e\x01\x42", 3);
  put_s(s, static_cast<std::int64_t>(kStartGas));
  s.push_back(0x0b);
  return s;
}

std::string gas_export(const Layout &l) {
  std::string s;
  put_name(s, kGasGlobal);
  s.push_back(0x03);
  put_u(s, l.globals);
  return s;
}

// A vector section with one more entry at its end.
std::string append(In &in, std::string_view entry) {
  std::string s;
  put_u(s, static_cast<std::uint64_t>(in.u32()) + 1);
  s.append(in.rest());
  s.append(entry);
  return s;
}

std::string one(std::string_view entry) {
  std::string s;
  put_u(s, 1);
  s.append(entry);
  return s;
}

} // namespace

bool meter(std::string_view module, Metered &out, std::string &error) {
  out = Metered{};
  try {
    In in(module);
    if (in.take(4) != "\0asm"sv)
      throw Bad("not a WebAssembly module");
    if (in.take(4) != "\x01\0\0\0"sv)
      throw Bad("not a core module (a component?): the gas pass meters core modules");
    struct Section {
      std::uint8_t id;
      std::string_view body;
    };
    std::vector<Section> sections;
    while (!in.done()) {
      const std::uint8_t id = in.byte();
      if (id > 13)
        throw Bad("a section of a kind the gas pass doesn't know (" + std::to_string(id) + ")");
      const std::uint32_t n = in.u32();
      sections.push_back({id, in.take(n)});
    }

    Layout l;
    std::uint32_t funcs = 0, codes = 0;
    bool metered = false, exports_gas = false, has[14] = {};
    for (const Section &s : sections) {
      has[s.id] = true;
      In b(s.body);
      switch (s.id) {
      case 1: { // function types only: the GC proposal's groups would number them differently
        l.types = b.u32();
        for (std::uint32_t i = 0; i < l.types; ++i) {
          if (b.byte() != 0x60)
            throw Bad("a GC type (a rec group, struct or array), which the gas pass doesn't read");
          for (int side = 0; side < 2; ++side) {
            const std::uint32_t n = b.u32();
            for (std::uint32_t j = 0; j < n; ++j)
              valtype(b);
          }
        }
        break;
      }
      case 2: {
        const std::uint32_t n = b.u32();
        for (std::uint32_t i = 0; i < n; ++i) {
          const std::string_view mod = b.name(), field = b.name();
          switch (b.byte()) {
          case 0x00:
            b.u32();
            if (mod == kGasModule && field == kGasImport)
              metered = true;
            ++l.func_imports;
            break;
          case 0x01:
            valtype(b);
            limits(b);
            break;
          case 0x02:
            limits(b);
            break;
          case 0x03:
            valtype(b);
            b.byte();
            ++l.globals;
            break;
          case 0x04: // a tag
            b.byte();
            b.u32();
            break;
          default:
            throw Bad("an import of a kind the gas pass doesn't know");
          }
        }
        break;
      }
      case 3:
        funcs = b.u32();
        break;
      case 6:
        l.globals += b.u32();
        break;
      case 7: {
        const std::uint32_t n = b.u32();
        for (std::uint32_t i = 0; i < n; ++i) {
          if (b.name() == kGasGlobal)
            exports_gas = true;
          b.byte();
          b.u32();
        }
        break;
      }
      case 10:
        codes = b.u32();
        break;
      default:
        break;
      }
    }
    if (metered) {
      out.bytes.assign(module);
      out.already = true;
      return true;
    }
    if (exports_gas)
      throw Bad("the module exports '" + std::string(kGasGlobal) + "' itself");
    if (funcs != codes)
      throw Bad("the module's function and code sections disagree");

    std::string o(module.substr(0, 8));
    // The sections the pass needs and the module doesn't have, put in their places.
    std::vector<std::pair<std::uint8_t, std::string>> missing;
    if (!has[1])
      missing.push_back({1, one(gas_type())});
    if (!has[2])
      missing.push_back({2, one(gas_import(l))});
    if (!has[6])
      missing.push_back({6, one(gas_global())});
    if (!has[7])
      missing.push_back({7, one(gas_export(l))});
    std::size_t next = 0;
    auto missing_before = [&](int where) {
      while (next < missing.size() && place(missing[next].first) < where) {
        put_section(o, missing[next].first, missing[next].second);
        ++next;
      }
    };

    for (const Section &s : sections) {
      if (s.id != 0)
        missing_before(place(s.id));
      In b(s.body);
      std::string body_out;
      switch (s.id) {
      case 0: { // custom
        const std::string_view name = b.name();
        if (name == "name") {
          std::string n;
          put_name(n, name);
          if (names(b, n, l))
            put_section(o, 0, n);
          continue;
        }
        if (name.substr(0, 7) == ".debug_" || name == "sourceMappingURL" ||
            name == "external_debug_info")
          continue; // offsets into code that moved
        b.rest();
        body_out.assign(s.body);
        break;
      }
      case 1:
        body_out = append(b, gas_type());
        break;
      case 2:
        body_out = append(b, gas_import(l));
        break;
      case 4: { // tables: a table's initial value may be a ref.func
        const std::uint32_t n = b.u32();
        put_u(body_out, n);
        for (std::uint32_t i = 0; i < n; ++i) {
          const std::uint8_t *t = b.at();
          if (b.peek() == 0x40) {
            b.byte();
            if (b.byte() != 0x00)
              throw Bad("a table of a kind the gas pass doesn't know");
            valtype(b);
            limits(b);
            body_out.append(b.since(t));
            const_expr(b, body_out, l);
          } else {
            valtype(b);
            limits(b);
            body_out.append(b.since(t));
          }
        }
        break;
      }
      case 6: {
        const std::uint32_t n = b.u32();
        put_u(body_out, static_cast<std::uint64_t>(n) + 1);
        for (std::uint32_t i = 0; i < n; ++i) {
          const std::uint8_t *t = b.at();
          valtype(b);
          b.byte(); // mutability
          body_out.append(b.since(t));
          const_expr(b, body_out, l);
        }
        body_out += gas_global();
        break;
      }
      case 7: {
        const std::uint32_t n = b.u32();
        put_u(body_out, static_cast<std::uint64_t>(n) + 1);
        for (std::uint32_t i = 0; i < n; ++i) {
          put_name(body_out, b.name());
          const std::uint8_t kind = b.byte();
          body_out.push_back(static_cast<char>(kind));
          const std::uint32_t index = b.u32();
          put_u(body_out, kind == 0x00 ? l.func(index) : kind == 0x03 ? l.global(index) : index);
        }
        body_out += gas_export(l);
        break;
      }
      case 8:
        put_u(body_out, l.func(b.u32()));
        break;
      case 9:
        elements(b, body_out, l);
        break;
      case 10: {
        const std::uint32_t n = b.u32();
        put_u(body_out, n);
        for (std::uint32_t i = 0; i < n; ++i) {
          const std::string f = body(b.sub(b.u32()), l, out.charges);
          put_u(body_out, f.size());
          body_out += f;
        }
        break;
      }
      case 11: { // data: an active segment's offset may name a global
        const std::uint32_t n = b.u32();
        put_u(body_out, n);
        for (std::uint32_t i = 0; i < n; ++i) {
          const std::uint32_t flags = b.u32();
          if (flags > 2)
            throw Bad("a data segment of a kind the gas pass doesn't know");
          put_u(body_out, flags);
          if (flags == 2)
            put_u(body_out, b.u32()); // its memory
          if (flags != 1)
            const_expr(b, body_out, l);
          const std::string_view bytes = b.name();
          put_name(body_out, bytes);
        }
        break;
      }
      default: // function, memory, tag, data count: nothing the pass renumbers
        body_out.assign(b.rest());
        break;
      }
      if (!b.done())
        throw Bad("a section goes on past its entries (" + std::to_string(s.id) + ")");
      put_section(o, s.id, body_out);
    }
    missing_before(99);
    out.bytes = std::move(o);
    return true;
  } catch (const Bad &e) {
    error = e.what();
    return false;
  }
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
