/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// Ariadne data (data.h): the store, its sources and the built-in decoders.

#include "../net/json_lite.h"

#include <algorithm>
#include <atomic>
#include <boost/any.hpp>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/bind.h> // read_or
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/height_field.h>
#include <cvc/image/image.h>
#include <cvc/image/image_decode.h> // a city's satellite.png, from its bytes
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h> // read_model_from_memory
#include <cvc/net/http_client.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <cvc/state/state_exec/types.h> // format_number_shortest
#include <cvc/volume/volume.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
#define CVC_DATA_INLINE 1 // no threads: a load runs where it is asked for
#endif

namespace cvc {
namespace ariadne {

namespace fs = std::filesystem;

namespace {

[[noreturn]] void fail(const std::string &what) { throw std::runtime_error(what); }

std::string join(const std::string &a, const std::string &b) {
  return a + cvc::state::SEPARATOR + b;
}

bool starts_with(const std::string &s, const std::string &p) {
  return s.size() >= p.size() && s.compare(0, p.size(), p) == 0;
}

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string extension(const std::string &path) {
  const std::size_t slash = path.find_last_of('/');
  const std::size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
    return "";
  return lower(path.substr(dot));
}

std::string basename(const std::string &path) {
  const std::size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

std::string sha256_of(const std::string &bytes) {
  return cvc::sha256_hex(reinterpret_cast<const unsigned char *>(bytes.data()), bytes.size());
}

std::string percent(const std::string &s) {
  static const char *hex = "0123456789ABCDEF";
  std::string out;
  for (unsigned char c : s) {
    if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += static_cast<char>(c);
    } else {
      out += '%';
      out += hex[c >> 4];
      out += hex[c & 15];
    }
  }
  return out;
}

std::string read_file(const fs::path &p, std::uint64_t cap) {
  std::error_code ec;
  const auto size = fs::file_size(p, ec);
  if (ec)
    fail("cannot read " + p.string() + ": " + ec.message());
  if (size > cap)
    fail(p.string() + " is " + std::to_string(size) + " bytes, past the cap of " +
         std::to_string(cap));
  std::ifstream in(p, std::ios::binary);
  if (!in)
    fail("cannot open " + p.string());
  std::string bytes(static_cast<std::size_t>(size), '\0');
  in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (static_cast<std::uint64_t>(in.gcount()) != size)
    fail("cannot read all of " + p.string());
  return bytes;
}

// ── decoders ─────────────────────────────────────────────────────────────────────────────────────

// Files a path-based reader opens, written to a fresh temporary directory removed afterwards.
class temp_dir {
public:
  temp_dir() {
    std::random_device rd;
    std::error_code ec;
    for (int attempt = 0; attempt < 16; ++attempt) {
      dir_ =
          fs::temp_directory_path(ec) / ("cvc-data-" + std::to_string(rd()) + std::to_string(rd()));
      if (!ec && fs::create_directories(dir_, ec))
        return;
    }
    fail("cannot make a temporary directory: " + ec.message());
  }
  ~temp_dir() {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  temp_dir(const temp_dir &) = delete;
  temp_dir &operator=(const temp_dir &) = delete;

  std::string put(const std::string &name, const std::string &bytes) {
    const fs::path p = dir_ / name;
    std::ofstream out(p, std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!out)
      fail("cannot write " + p.string());
    return p.string();
  }
  std::string path() const { return dir_.string(); }

private:
  fs::path dir_;
};

const cvc::archive::file &only_file(DataDecode &d) {
  if (d.files.size() != 1)
    fail("as: " + d.spec.as + " decodes one file, and " + d.spec.uri + " has " +
         std::to_string(d.files.size()) + " (name one with path:)");
  return d.files.front();
}

// A JSON value as state: an object's members as children (a '.' in a key becomes '_'), an
// array's items as children 0, 1, ... with the count as the node's value, a scalar as the value.
void json_to_state(const cvc::net::json_lite::Value &v, cvc::state &n) {
  using K = cvc::net::json_lite::Value::Kind;
  switch (v.kind) {
  case K::object:
    for (const auto &[key, member] : v.members) {
      std::string k = key;
      std::replace(k.begin(), k.end(), cvc::state::SEPARATOR[0], '_');
      if (!k.empty())
        json_to_state(member, n(k));
    }
    break;
  case K::array:
    for (std::size_t i = 0; i < v.items.size(); ++i)
      json_to_state(v.items[i], n(std::to_string(i)));
    n.value(std::to_string(v.items.size()));
    break;
  case K::string:
    n.value(v.str);
    break;
  case K::boolean:
    n.value(std::string(v.b ? "1" : "0"));
    break;
  case K::number: {
    if (v.integral) {
      n.value(std::to_string(v.i));
    } else { // the shortest text that reads back as it, on every platform (not std::to_chars:
             // libc++ lacks it for doubles before macOS 13.3)
      n.value(cvc::state_exec::format_number_shortest(v.num));
    }
    break;
  }
  default:
    n.value(std::string());
  }
}

void decode_files(DataDecode &) {}

void decode_bytes(DataDecode &d) {
  const auto &f = only_file(d);
  d.out("content").data(boost::any(DataBytes(f.bytes)));
  d.out("content").value(std::to_string(f.bytes->size()));
}

void decode_text(DataDecode &d) { d.out("text").value(*only_file(d).bytes); }

void decode_json(DataDecode &d) {
  const auto &f = only_file(d);
  std::string error;
  const auto v = cvc::net::json_lite::parse(*f.bytes, &error);
  if (!v)
    fail(f.path + " is not JSON: " + error);
  cvc::state &j = d.out("json");
  j.reset();
  json_to_state(*v, j);
}

void decode_mesh(DataDecode &d) {
  const auto &f = only_file(d);
  const std::string ext = extension(f.path);
  std::shared_ptr<cvc::geometry> geom;
  if (ext != ".off" && cvc::can_read_model_from_memory(ext)) { // .off: libcvc's own reader
    geom = std::make_shared<cvc::geometry>(
        cvc::read_model_from_memory(d.app, f.bytes->data(), f.bytes->size(), ext).merged());
  } else { // a format only a path-based reader reads (.off, .raw*): the file, briefly
    temp_dir tmp;
    geom = std::make_shared<cvc::geometry>(
        cvc::read_geometry(d.app, tmp.put(basename(f.path), *f.bytes)));
  }
  d.out("geometry").data(boost::any(std::shared_ptr<const cvc::geometry>(std::move(geom))));
  d.out("geometry").value(std::string("geometry"));
}

void decode_volume(DataDecode &d) {
  const auto &f = only_file(d);
  temp_dir tmp;
  auto vol = std::make_shared<cvc::volume>(d.app, tmp.put(basename(f.path), *f.bytes));
  d.out("volume").data(boost::any(std::shared_ptr<const cvc::volume>(std::move(vol))));
  d.out("volume").value(std::string("volume"));
}

void publish_bounds(DataDecode &d, const cvc::world::city_bundle &city) {
  d.out("bounds.min_x").value(city.min_x);
  d.out("bounds.min_y").value(city.min_y);
  d.out("bounds.max_x").value(city.max_x);
  d.out("bounds.max_y").value(city.max_y);
}

void publish_terrain(DataDecode &d, std::shared_ptr<const cvc::height_field> field) {
  if (!field)
    fail("the city bundle has no terrain");
  cvc::state &t = d.out("terrain");
  t("rows").value(field->rows());
  t("cols").value(field->cols());
  t.data(boost::any(std::move(field)));
  t.value(std::string("height_field"));
}

// A city bundle's files as load_city_bundle reads them, straight from the store: no file on disk.
cvc::world::city_bundle_reader bundle_files(const DataDecode &d) {
  return [&d](const std::string &name) {
    const cvc::archive::file *f = d.find(name);
    return f ? f->bytes : DataBytes();
  };
}

// A city bundle (city_bundle.h) from its files: terrain.json, and buildings.glb (or
// buildings_flat.glb) when it has them.
void decode_city_bundle(DataDecode &d) {
  const cvc::archive::file *terrain = d.find("terrain.json");
  if (!terrain)
    fail("as: city_bundle needs a terrain.json, and " + d.spec.uri + " has none" +
         (d.spec.path.empty() ? " (name its directory with path:)" : " under " + d.spec.path));
  cvc::world::city_bundle city = cvc::world::load_city_bundle(d.app, bundle_files(d), d.spec.uri);
  auto world = std::make_shared<const cvc::world::world_geometry>(std::move(city.geometry));
  d.out("world").data(boost::any(world));
  d.out("world").value(std::string("world_geometry"));
  publish_terrain(d, world->terrain());
  if (const cvc::world::geometry_item *b = world->find("buildings")) {
    d.out("buildings").data(boost::any(b->geometry));
    d.out("buildings").value(std::string("geometry"));
  }
  d.out("buildings_error").value(city.buildings_error);
  publish_bounds(d, city);
  // The aerial image, decoded here off the frame thread: the scene drapes it over the terrain.
  cvc::state &satellite = d.out("satellite");
  satellite.value(std::string());
  if (const cvc::archive::file *sat = d.find("satellite.png")) {
    cvc::image_decode_limits lim; // a city's aerial image may be as big as a texture can be
    lim.max_side = 16384;
    lim.max_pixels = std::size_t(16384) * 16384;
    lim.max_bytes = std::size_t(1) << 30;
    std::string why;
    cvc::image img = cvc::decode_image(sat->bytes->data(), sat->bytes->size(), lim, &why);
    if (img.empty())
      d.out("satellite_error").value(sat->path + ": " + why);
    else
      satellite.data(boost::any(std::make_shared<const cvc::image>(std::move(img))));
    satellite.value(sat->path);
  }
}

// A terrain.json alone: the bundle's height field.
void decode_heightfield(DataDecode &d) {
  const cvc::archive::file *terrain =
      d.files.size() == 1 ? &d.files.front() : d.find("terrain.json");
  if (!terrain)
    fail("as: heightfield reads a terrain.json, and " + d.spec.uri + " has none");
  cvc::world::city_bundle_options opt;
  opt.buildings = false;
  const DataBytes text = terrain->bytes;
  cvc::world::city_bundle city = cvc::world::load_city_bundle(
      d.app,
      [text](const std::string &name) { return name == "terrain.json" ? text : DataBytes(); },
      d.spec.uri, opt);
  publish_terrain(d, city.geometry.terrain());
  publish_bounds(d, city);
}

// An app's decoders: the built-ins, then what its host registered.
struct decoder_table {
  static constexpr const char *app_slot = "cvc.ariadne.data-decoders";
  std::mutex m;
  std::map<std::string, DataDecoder> decoders = {
      {"files", decode_files},
      {"bytes", decode_bytes},
      {"text", decode_text},
      {"json", decode_json},
      {"mesh", decode_mesh},
      {"volume", decode_volume},
      {"heightfield", decode_heightfield},
      {"city_bundle", decode_city_bundle},
  };
};

DataDecoder find_decoder(cvc::app &app, const std::string &as) {
  decoder_table &t = app.slot<decoder_table>();
  std::lock_guard<std::mutex> lock(t.m);
  const auto it = t.decoders.find(as);
  return it == t.decoders.end() ? DataDecoder() : it->second;
}

// The decoder a request without as: gets: files for an archive or a directory, else by the file's
// extension.
std::string default_decoder(bool many, const std::string &name) {
  if (many)
    return "files";
  const std::string ext = extension(name);
  if (ext == ".json")
    return "json";
  if (ext == ".txt" || ext == ".md" || ext == ".csv" || ext == ".yaml" || ext == ".yml" ||
      ext == ".ari")
    return "text";
  for (const char *e : {".obj", ".ply", ".off", ".stl", ".glb", ".gltf", ".fbx", ".dae", ".raw",
                        ".rawn", ".rawc", ".rawnc"})
    if (ext == e)
      return "mesh";
  for (const char *e : {".rawiv", ".rawv", ".mrc", ".map", ".vtk"})
    if (ext == e)
      return "volume";
  return "bytes";
}

// ── sources ──────────────────────────────────────────────────────────────────────────────────────

struct Limits {
  std::uint64_t max_bytes = 2ull << 30;
  std::size_t max_entries = 100000;
};

struct Network {
  bool on = false;
  std::shared_ptr<const cvc::net::NetPolicy> policy; // null: every origin
  // The app's HTTP client, taken with the app: the download's thread sends through it holding no
  // app.
  std::shared_ptr<cvc::net::HttpClient> client;
};

// What a source produced: the bytes of one file or archive, or a directory's files.
struct Fetched {
  std::string bytes;
  std::string name; // the file's name, for a single file
  bool directory = false;
  std::vector<cvc::archive::file> files; // a directory's
  std::string canonical;
  std::string sha256;
};

cvc::net::HttpResponse http_get(const std::string &url, const Network &net, std::uint64_t cap) {
  if (!net.on)
    fail("the network is off, so " + url +
         " cannot be fetched: run the host with the network on (--net-allow ORIGIN)");
  cvc::net::HttpRequest req;
  req.url = url;
  req.max_bytes = static_cast<std::size_t>(std::min<std::uint64_t>(cap, SIZE_MAX));
  req.connect_timeout_secs = 30;
  req.timeout_secs = 1800;
  req.user_agent = "cvc-data/1";
  cvc::net::HttpResponse r =
      net.policy ? cvc::net::send(*net.client, req, *net.policy) : cvc::net::send(*net.client, req);
  if (!r.ok)
    fail(url + ": " + r.error);
  return r;
}

std::string registry_url(cvc::app &app) {
  std::string reg = read_or<std::string>(app, "sys.data.cvcpkg.registry", "");
  if (reg.empty())
    if (const char *e = std::getenv("CVCPKG_URL"))
      reg = e;
  if (reg.empty())
    reg = "https://cvcpkg.org";
  while (!reg.empty() && reg.back() == '/')
    reg.pop_back();
  return reg;
}

// cvcpkg://ORG/NAME[@VERSION] (or cvcpkg://NAME): the registry's newest noarch build of the
// package, or that version's; its archive from the cache when the cache has it, else downloaded,
// checked against the registry's size and sha256, and kept.
Fetched fetch_cvcpkg(const Uri &u, const std::string &registry, const Network &net,
                     const Limits &lim) {
  std::string ref = u.path, version;
  if (const std::size_t at = ref.find('@'); at != std::string::npos) {
    version = ref.substr(at + 1);
    ref = ref.substr(0, at);
  }
  while (!ref.empty() && ref.back() == '/')
    ref.pop_back();
  std::string org, name = ref;
  if (const std::size_t slash = ref.find('/'); slash != std::string::npos) {
    org = ref.substr(0, slash);
    name = ref.substr(slash + 1);
  }
  if (name.empty() || name.find('/') != std::string::npos)
    fail(u.raw + ": a cvcpkg URI is cvcpkg://ORG/NAME[@VERSION]");
  std::string query = registry + "/v1/packages/" + percent(name) + "/latest?org=" + percent(org) +
                      "&platform=any&arch=noarch&build_type=none&link=none";
  if (!version.empty())
    query += "&version=" + percent(version);
  const cvc::net::HttpResponse meta = http_get(query, net, 4u << 20);
  if (meta.status == 404)
    fail(u.raw + ": " + registry + " has no data build of " + (org.empty() ? "" : org + "/") +
         name + (version.empty() ? "" : "@" + version) + " (platform any, arch noarch)");
  if (meta.status != 200)
    fail(u.raw + ": " + registry + " answered " + std::to_string(meta.status));
  std::string error;
  const auto row = cvc::net::json_lite::parse(meta.body, &error);
  if (!row || !row->is_object())
    fail(u.raw + ": the registry's reply is not a package (" + error + ")");
  const std::string sha = lower(row->string_at("sha256").value_or(""));
  const std::string archive_url = row->string_at("archive_url").value_or("");
  const std::int64_t size = row->int_at("size_bytes").value_or(-1);
  const std::string got_version = row->string_at("version").value_or(version);
  if (sha.size() != 64 || archive_url.empty())
    fail(u.raw + ": the registry's reply names no archive and sha256");
  if (size > 0 && static_cast<std::uint64_t>(size) > lim.max_bytes)
    fail(u.raw + ": the archive is " + std::to_string(size) + " bytes, past the cap of " +
         std::to_string(lim.max_bytes) + " (sys.data.max_bytes)");

  Fetched out;
  out.canonical = "cvcpkg://" + (org.empty() ? "" : org + "/") + name + "@" + got_version;
  out.sha256 = sha;
  out.name = basename(archive_url);

  const std::string cache = data_cache_dir();
  const fs::path cached = cache.empty() ? fs::path() : fs::path(cache) / sha;
  std::error_code ec;
  if (!cache.empty() && fs::is_regular_file(cached, ec)) {
    try {
      out.bytes = read_file(cached, lim.max_bytes);
      if (sha256_of(out.bytes) == sha)
        return out;
    } catch (const std::exception &) {
    }
    fs::remove(cached, ec); // corrupt or unreadable: fetched again
  }
  const std::string url =
      starts_with(archive_url, "http://") || starts_with(archive_url, "https://")
          ? archive_url
          : registry + archive_url;
  cvc::net::HttpResponse r =
      http_get(url, net, size > 0 ? static_cast<std::uint64_t>(size) + 1 : lim.max_bytes);
  if (r.status != 200)
    fail(url + " answered " + std::to_string(r.status));
  if (size >= 0 && r.body.size() != static_cast<std::uint64_t>(size))
    fail(url + " sent " + std::to_string(r.body.size()) + " bytes, not the registry's " +
         std::to_string(size));
  if (sha256_of(r.body) != sha)
    fail(url + " does not match the registry's sha256");
  out.bytes = std::move(r.body);
  if (!cache.empty()) { // best effort: a failed write only means the next run downloads again
    fs::create_directories(cache, ec);
    const fs::path part = cached.string() + ".part" +
                          std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    {
      std::ofstream f(part, std::ios::binary);
      f.write(out.bytes.data(), static_cast<std::streamsize>(out.bytes.size()));
    }
    fs::rename(part, cached, ec);
    if (ec)
      fs::remove(part, ec);
  }
  return out;
}

bool keep_under(const std::string &root, const std::string &p) {
  return root.empty() || p == root || starts_with(p, root + "/");
}

// What a scheme with no transport of its own here (state://, cvc://, a host's) resolves to, with
// the app's resolver, or an error. run_load passes one that holds the app while it resolves.
using AppResolve = std::function<UriResult(const std::string &uri, const std::string &base)>;

Fetched fetch(const DataSpec &spec, const std::string &registry, const Network &net,
              const Limits &lim, const std::string &keep_root, const AppResolve &resolve_uri) {
  const Uri u = parse_uri(spec.uri);
  if (u.scheme == "file") {
    const fs::path p = resolve_file_path(u.path, spec.base);
    Fetched out;
    out.canonical = p.generic_string();
    std::error_code ec;
    if (fs::is_directory(p, ec)) {
      out.directory = true;
      std::uint64_t total = 0;
      for (fs::recursive_directory_iterator it(p, ec), end; !ec && it != end; it.increment(ec)) {
        if (!it->is_regular_file(ec))
          continue;
        const std::string rel =
            cvc::archive::safe_path(fs::relative(it->path(), p, ec).generic_string());
        if (rel.empty() || !keep_under(keep_root, rel))
          continue;
        if (out.files.size() >= lim.max_entries)
          fail(spec.uri + " has more than " + std::to_string(lim.max_entries) + " files");
        auto bytes = std::make_shared<std::string>(read_file(it->path(), lim.max_bytes));
        total += bytes->size();
        if (total > lim.max_bytes)
          fail(spec.uri + " holds more than " + std::to_string(lim.max_bytes) + " bytes");
        out.files.push_back({rel, std::move(bytes)});
      }
      if (ec)
        fail("cannot list " + p.string() + ": " + ec.message());
      std::sort(out.files.begin(), out.files.end(),
                [](const auto &a, const auto &b) { return a.path < b.path; });
      return out;
    }
    out.bytes = read_file(p, lim.max_bytes);
    out.name = p.filename().string();
    return out;
  }
  if (u.scheme == "http" || u.scheme == "https") {
    cvc::net::HttpResponse r = http_get(spec.uri, net, lim.max_bytes);
    if (r.status < 200 || r.status >= 300)
      fail(spec.uri + " answered " + std::to_string(r.status));
    Fetched out;
    out.bytes = std::move(r.body);
    out.canonical = r.canonical_url.empty() ? spec.uri : r.canonical_url;
    std::string path = parse_uri(out.canonical).path;
    if (const std::size_t q = path.find_first_of("?#"); q != std::string::npos)
      path = path.substr(0, q);
    out.name = basename(path);
    return out;
  }
  if (u.scheme == "cvcpkg")
    return fetch_cvcpkg(u, registry, net, lim);
  UriResult r = resolve_uri(spec.uri, spec.base);
  if (!r.ok)
    fail(r.error.empty() ? spec.uri + ": cannot be resolved" : r.error);
  Fetched out;
  out.bytes = std::move(r.content);
  out.canonical = r.canonical;
  out.name = basename(u.path);
  return out;
}

// The request's files: a directory's, an archive's (only those under `root`), a compressed file's
// contents, or the one file. `root` is stripped from the paths.
std::vector<cvc::archive::file> files_of(Fetched &f, const std::string &root, const Limits &lim,
                                         bool *many) {
  std::vector<cvc::archive::file> files;
  *many = true;
  if (f.directory) {
    files = std::move(f.files);
  } else {
    cvc::archive::limits al;
    al.max_bytes = lim.max_bytes;
    al.max_entries = lim.max_entries;
    switch (cvc::archive::classify(f.bytes)) {
    case cvc::archive::kind::tar:
    case cvc::archive::kind::zip:
      files = cvc::archive::unpack(f.bytes, al,
                                   [&root](const std::string &p) { return keep_under(root, p); });
      break;
    case cvc::archive::kind::compressed: {
      std::string name = f.name;
      for (const char *ext : {".gz", ".zst", ".zstd"})
        if (name.size() > std::strlen(ext) &&
            lower(name).compare(name.size() - std::strlen(ext), std::string::npos, ext) == 0) {
          name.resize(name.size() - std::strlen(ext));
          break;
        }
      files.push_back({name, std::make_shared<const std::string>(
                                 cvc::archive::decompress(f.bytes, lim.max_bytes))});
      *many = false;
      return files;
    }
    default:
      files.push_back({f.name.empty() ? std::string("data") : f.name,
                       std::make_shared<const std::string>(std::move(f.bytes))});
      *many = false;
      return files;
    }
  }
  if (!root.empty()) {
    for (auto &file : files)
      file.path = file.path == root ? basename(file.path) : file.path.substr(root.size() + 1);
    if (files.empty())
      fail("nothing under " + root);
    if (files.size() == 1 && files.front().path.find('/') == std::string::npos &&
        !extension(files.front().path).empty())
      *many = false; // path: named one file
  }
  return files;
}

std::string slug(const std::string &uri) {
  Uri u = parse_uri(uri);
  std::string p = u.path;
  if (const std::size_t at = p.find('@'); u.scheme == "cvcpkg" && at != std::string::npos)
    p = p.substr(0, at);
  while (!p.empty() && (p.back() == '/' || p.back() == '\\'))
    p.pop_back();
  const std::size_t cut = p.find_last_of("/\\");
  std::string s = lower(cut == std::string::npos ? p : p.substr(cut + 1));
  std::string out;
  for (char c : s) {
    const bool alnum = std::isalnum(static_cast<unsigned char>(c)) != 0;
    if (alnum)
      out += c;
    else if (!out.empty() && out.back() != '_')
      out += '_';
  }
  while (!out.empty() && out.back() == '_')
    out.pop_back();
  if (out.size() > 40)
    out.resize(40);
  return out.empty() ? std::string("data") : out;
}

std::string canonical_request(const DataSpec &spec) {
  const Uri u = parse_uri(spec.uri);
  const std::string source = u.scheme == "file" ? resolve_file_path(u.path, spec.base) : spec.uri;
  return source + "\n" + cvc::archive::safe_path(spec.path) + "\n" + spec.as;
}

} // namespace

// ── DataDecode ───────────────────────────────────────────────────────────────────────────────────

DataBytes DataDecode::file(const std::string &path) const {
  for (const auto &f : files)
    if (f.path == path)
      return f.bytes;
  return nullptr;
}

const cvc::archive::file *DataDecode::find(const std::string &name) const {
  for (const auto &f : files)
    if (f.path == name)
      return &f;
  for (const auto &f : files)
    if (basename(f.path) == name)
      return &f;
  return nullptr;
}

cvc::state &DataDecode::out(const std::string &child) const {
  return cvc::state::instance(app)(join(node, child));
}

void register_data_decoder(cvc::app &app, const std::string &as, DataDecoder decoder) {
  decoder_table &t = app.slot<decoder_table>();
  std::lock_guard<std::mutex> lock(t.m);
  if (decoder)
    t.decoders[as] = std::move(decoder);
  else
    t.decoders.erase(as);
}

bool has_data_decoder(cvc::app &app, const std::string &as) {
  return static_cast<bool>(find_decoder(app, as));
}

std::vector<std::string> data_decoders(cvc::app &app) {
  decoder_table &t = app.slot<decoder_table>();
  std::lock_guard<std::mutex> lock(t.m);
  std::vector<std::string> out;
  for (const auto &kv : t.decoders)
    out.push_back(kv.first);
  return out;
}

std::string data_key(const DataSpec &spec) {
  return slug(spec.uri) + "_" + sha256_of(canonical_request(spec)).substr(0, 8);
}

std::string data_cache_dir() {
#if defined(__EMSCRIPTEN__)
  if (const char *e = std::getenv("CVC_DATA_CACHE"))
    return e;
  return "";
#else
  if (const char *e = std::getenv("CVC_DATA_CACHE"))
    return e;
  const auto env = [](const char *n) -> std::string {
    const char *v = std::getenv(n);
    return v && *v ? std::string(v) : std::string();
  };
#if defined(_WIN32)
  const std::string base = env("LOCALAPPDATA");
  return base.empty() ? std::string() : (fs::path(base) / "cvc" / "data").string();
#elif defined(__APPLE__)
  const std::string home = env("HOME");
  return home.empty() ? std::string()
                      : (fs::path(home) / "Library" / "Caches" / "cvc" / "data").string();
#else
  const std::string xdg = env("XDG_CACHE_HOME");
  if (!xdg.empty())
    return (fs::path(xdg) / "cvc" / "data").string();
  const std::string home = env("HOME");
  return home.empty() ? std::string() : (fs::path(home) / ".cache" / "cvc" / "data").string();
#endif
#endif
}

// ── the store ────────────────────────────────────────────────────────────────────────────────────

// What a load's worker shares with its store: the app while the store is alive, under a lock, so
// a worker the store left behind (shutdown) never writes to an app that may be gone.
struct DataStoreLife {
  std::mutex mu;
  cvc::app *app = nullptr; // null once the store has stopped
};

struct DataStore::Impl {
  explicit Impl(cvc::app &a) : app(a), life(std::make_shared<DataStoreLife>()) { life->app = &a; }

  cvc::app &app;
  std::shared_ptr<DataStoreLife> life;
  std::mutex mu;
  struct Job {
    std::thread worker;
    std::shared_ptr<std::atomic<bool>> done;
  };
  std::map<std::string, Job> jobs;       // by store key
  std::map<std::string, DataSpec> lazy;  // store key -> its request, bound but not loaded
  std::map<std::string, DataSpec> specs; // store key -> its request, every one seen
  bool network_set = false;
  std::shared_ptr<const cvc::net::NetPolicy> policy;
  bool stopped = false;

  cvc::state &root() { return cvc::state::instance(app); }

  void reap() { // join the workers that have finished (caller holds mu)
    for (auto it = jobs.begin(); it != jobs.end();) {
      if (it->second.done->load()) {
        if (it->second.worker.joinable())
          it->second.worker.join();
        it = jobs.erase(it);
      } else {
        ++it;
      }
    }
  }

  Network network() const {
    Network n;
    n.client = cvc::net::http_client(app);
    if (network_set) {
      n.on = true;
      n.policy = policy;
    } else if (auto access = net_host_access(app)) {
      n.on = !access->live || access->live->load();
      n.policy = access->policy;
    }
    return n;
  }
};

namespace {

// Write under the store's lock, unless the store has stopped (then the load just ends).
struct stopped_error {};
template <class F> void with_app(const std::shared_ptr<DataStoreLife> &life, F &&f) {
  std::lock_guard<std::mutex> lock(life->mu);
  if (!life->app)
    throw stopped_error{};
  f(*life->app);
}

void set_status(cvc::app &app, const std::string &node, const std::string &status,
                double progress) {
  cvc::state &n = cvc::state::instance(app)(node);
  n(std::string("progress")).value(progress);
  n(std::string("status")).value(status);
  n.value(status);
}

void run_load(std::shared_ptr<DataStoreLife> life, DataSpec spec, std::string node,
              std::string registry, Network net, Limits lim) {
  try {
    with_app(life, [&](cvc::app &a) { set_status(a, node, "fetching", 0.0); });
    const std::string root = cvc::archive::safe_path(spec.path);
    // The network's wait holds no app. A local scheme (state://, cvc://) resolves with the app,
    // held while it does (with_app: the load stops if the app has gone).
    const AppResolve with_app_resolve = [&life](const std::string &uri, const std::string &base) {
      UriResult r;
      with_app(life, [&](cvc::app &a) { r = resolve(a, uri, base); });
      return r;
    };
    Fetched f = fetch(spec, registry, net, lim, root, with_app_resolve);
    const std::uint64_t size = f.directory ? 0 : f.bytes.size();
    with_app(life, [&](cvc::app &a) { set_status(a, node, "unpacking", 0.5); });
    bool many = true;
    std::vector<cvc::archive::file> files = files_of(f, root, lim, &many);
    with_app(life, [&](cvc::app &a) {
      cvc::state &n = cvc::state::instance(a)(node);
      cvc::state &fs_node = n(std::string("files"));
      fs_node.reset();
      for (std::size_t i = 0; i < files.size(); ++i) {
        cvc::state &e = fs_node(std::to_string(i));
        e.property("size", std::to_string(files[i].bytes->size()));
        e.data(boost::any(DataBytes(files[i].bytes)));
        e.value(files[i].path);
      }
      fs_node.value(std::to_string(files.size()));
      set_status(a, node, "decoding", 0.8);
    });
    DataSpec decoded = spec;
    if (decoded.as.empty())
      decoded.as = default_decoder(many, files.empty() ? std::string() : files.front().path);
    DataDecoder decoder;
    with_app(life, [&](cvc::app &a) { decoder = find_decoder(a, decoded.as); });
    if (!decoder)
      fail("no decoder as: " + decoded.as +
           " (known: files, bytes, text, json, mesh, heightfield, "
           "volume, city_bundle, and any a host registered)");
    with_app(life, [&](cvc::app &a) {
      DataDecode d{a, decoded, node, files};
      decoder(d);
      cvc::state &n = cvc::state::instance(a)(node);
      n(std::string("as")).value(decoded.as);
      n(std::string("canonical")).value(f.canonical);
      n(std::string("sha256")).value(f.sha256);
      n(std::string("size")).value(std::to_string(size));
      n(std::string("error")).value(std::string());
      const int rev = read_or<int>(a, join(node, "rev"), 0);
      n(std::string("rev")).value(rev + 1);
      set_status(a, node, "ready", 1.0);
    });
  } catch (const stopped_error &) {
  } catch (const std::exception &e) {
    try {
      with_app(life, [&](cvc::app &a) {
        cvc::state::instance(a)(join(node, "error")).value(std::string(e.what()));
        set_status(a, node, "error", 1.0);
      });
    } catch (const stopped_error &) {
    }
  }
}

// An app's one store, made on first use and held by the app: it goes with the app's slots (once
// the app's pools have stopped), and stops writing as soon as the app's state tree goes, so its
// loads can never write into an app that has died.
struct store_slot {
  static constexpr const char *app_slot = "cvc.ariadne.data-store";
  std::mutex m;
  std::shared_ptr<DataStore> store;
};

} // namespace

DataStore::DataStore(cvc::app &app) : impl_(std::make_shared<Impl>(app)) {}

DataStore::~DataStore() { shutdown(); }

std::shared_ptr<DataStore> DataStore::of(cvc::app &app) {
  store_slot &slot = app.slot<store_slot>();
  std::lock_guard<std::mutex> lock(slot.m);
  if (slot.store)
    return slot.store;
  std::shared_ptr<DataStore> s(new DataStore(app));
  slot.store = s;
  // The app's state tree going (~state signals before its children go) ends the store: a load
  // still running writes nothing more.
  cvc::state::instance(app).destroyed.connect([life = s->impl_->life] {
    std::lock_guard<std::mutex> lock(life->mu);
    life->app = nullptr;
  });
  return s;
}

std::string DataStore::load(const DataSpec &spec, bool reload) {
  const std::string key = data_key(spec);
  const std::string node = join(kDataRoot, key);
  Impl &im = *impl_;
  std::unique_lock<std::mutex> lock(im.mu);
  if (im.stopped)
    return node;
  im.reap();
  im.lazy.erase(key);
  im.specs[key] = spec;
  if (im.jobs.count(key))
    return node; // loading
  cvc::state &n = im.root()(node);
  const std::string status = n(std::string("status")).value();
  if (!reload && status == "ready")
    return node;
  n(std::string("uri")).value(spec.uri);
  set_status(im.app, node, "fetching", 0.0);
  Limits lim;
  lim.max_bytes = static_cast<std::uint64_t>(
      read_or<double>(im.app, "sys.data.max_bytes", static_cast<double>(lim.max_bytes)));
  lim.max_entries = static_cast<std::size_t>(
      read_or<double>(im.app, "sys.data.max_entries", static_cast<double>(lim.max_entries)));
  const std::string registry = registry_url(im.app);
  const Network net = im.network();
#if defined(CVC_DATA_INLINE)
  lock.unlock();
  run_load(im.life, spec, node, registry, net, lim);
#else
  auto done = std::make_shared<std::atomic<bool>>(false);
  std::thread worker([life = im.life, spec, node, registry, net, lim, done] {
    run_load(life, spec, node, registry, net, lim);
    done->store(true);
  });
  im.jobs[key] = Impl::Job{std::move(worker), done};
#endif
  return node;
}

std::string DataStore::bind(const std::string &prefix, const DataSpec &spec) {
  std::string node;
  if (spec.lazy) {
    const std::string key = data_key(spec);
    node = join(kDataRoot, key);
    std::lock_guard<std::mutex> lock(impl_->mu);
    cvc::state &n = impl_->root()(node);
    if (n(std::string("status")).value().empty()) {
      n(std::string("uri")).value(spec.uri);
      set_status(impl_->app, node, "idle", 0.0);
      impl_->lazy[key] = spec;
    }
    impl_->specs[key] = spec;
  } else {
    node = load(spec);
  }
  const std::string entry = resolve_bind(prefix, join(kDataRoot, spec.name));
  cvc::state &link = impl_->root()(entry);
  if (link.linkTarget() != node)
    link.linkTo(node, cvc::state::link_mode::transparent);
  return node;
}

std::string DataStore::resolve(const std::string &path) const {
  cvc::state *s = cvc::state::instance(impl_->app).findThroughLinks(path);
  if (!s)
    return "";
  const std::string full = s->fullName();
  const std::string store = std::string(kDataRoot) + cvc::state::SEPARATOR;
  return starts_with(full, store) &&
                 full.find(cvc::state::SEPARATOR, store.size()) == std::string::npos
             ? full
             : "";
}

void DataStore::touch(const std::string &path) {
  const std::string node = resolve(path);
  if (node.empty())
    return;
  DataSpec spec;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    const auto it = impl_->lazy.find(node.substr(std::string(kDataRoot).size() + 1));
    if (it == impl_->lazy.end())
      return;
    spec = it->second;
  }
  load(spec);
}

std::string DataStore::reload(const std::string &path) {
  const std::string node = resolve(path);
  if (node.empty())
    return "";
  DataSpec spec;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    const auto it = impl_->specs.find(node.substr(std::string(kDataRoot).size() + 1));
    if (it == impl_->specs.end())
      return "";
    spec = it->second;
  }
  return load(spec, true);
}

bool DataStore::wait(const std::string &path, double timeout_seconds) {
  const auto start = std::chrono::steady_clock::now();
  touch(path);
  for (;;) {
    const std::string s = data_status(impl_->app, path);
    if (s == "ready")
      return true;
    if (s == "error" || s.empty())
      return false;
    if (timeout_seconds >= 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >=
            timeout_seconds)
      return false;
#if defined(CVC_DATA_INLINE)
    return false; // nothing else can make progress while this waits
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif
  }
}

void DataStore::set_network(std::shared_ptr<const cvc::net::NetPolicy> policy) {
  std::lock_guard<std::mutex> lock(impl_->mu);
  impl_->network_set = true;
  impl_->policy = std::move(policy);
}

void DataStore::shutdown() {
  if (!impl_)
    return;
  std::map<std::string, Impl::Job> jobs;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    if (impl_->stopped)
      return;
    impl_->stopped = true;
    jobs.swap(impl_->jobs);
  }
  {
    std::lock_guard<std::mutex> lock(impl_->life->mu); // a decode in progress finishes first
    impl_->life->app = nullptr;
  }
  for (auto &[key, job] : jobs) {
    if (job.done->load())
      job.worker.join();
    else
      job.worker.detach(); // it ends on its own, writing nothing more
  }
}

// ── readers ──────────────────────────────────────────────────────────────────────────────────────

DataBytes data_file(cvc::app &app, const std::string &path, const std::string &file_path) {
  cvc::state *n = cvc::state::instance(app).findThroughLinks(join(path, "files"));
  if (!n)
    return nullptr;
  const int count = read_or<int>(app, n->fullName(), 0);
  for (int i = 0; i < count; ++i) {
    cvc::state &e = (*n)(std::to_string(i));
    if (e.value() == file_path) {
      try {
        return boost::any_cast<DataBytes>(e.data());
      } catch (const boost::bad_any_cast &) {
        return nullptr;
      }
    }
  }
  return nullptr;
}

std::vector<std::string> data_files(cvc::app &app, const std::string &path) {
  std::vector<std::string> out;
  cvc::state *n = cvc::state::instance(app).findThroughLinks(join(path, "files"));
  if (!n)
    return out;
  const int count = read_or<int>(app, n->fullName(), 0);
  for (int i = 0; i < count; ++i)
    out.push_back((*n)(std::to_string(i)).value());
  return out;
}

std::string data_status(cvc::app &app, const std::string &path) {
  cvc::state *n = cvc::state::instance(app).findThroughLinks(join(path, "status"));
  return n ? n->value() : std::string();
}

} // namespace ariadne
} // namespace cvc
