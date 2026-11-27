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
#include <cvc/ariadne/scene_generated_world.h> // the generated_world generator
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/core/thread_support.h>
#include <cvc/geometry/contours.h>
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
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/world/city_bundle.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

#if defined(CVC_NO_THREADS)
#define CVC_DATA_INLINE 1 // no threads: a load runs where it is asked for
#endif

#if defined(__EMSCRIPTEN__)
#include <emscripten.h>

// A saved file, in a browser: the page downloads it under its name (data_save).
// clang-format off
EM_JS(int, cvc_data_download, (const char *name, const char *bytes, int size), {
  if (typeof document === 'undefined')
    return 0;
  const url = URL.createObjectURL(
      new Blob([HEAPU8.slice(bytes, bytes + size)], {type : 'application/octet-stream'}));
  const a = document.createElement('a');
  a.href = url;
  a.download = UTF8ToString(name);
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
  return 1;
});
// clang-format on
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

// A .vgr's point classes (VolumeGridRover's), at `classes`. A .pts read with as: point_classes
// has its coordinates rounded to voxel indices: the decoder doesn't know the volume's frame.
void decode_point_classes(DataDecode &d) {
  const auto &f = only_file(d);
  temp_dir tmp;
  auto pc = std::make_shared<const cvc::point_classes>(
      cvc::read_point_classes(d.app, tmp.put(basename(f.path), *f.bytes)));
  d.out("classes").value(std::to_string(pc->point_count()));
  d.out("classes").data(boost::any(std::shared_ptr<const cvc::point_classes>(std::move(pc))));
}

// A contour stack (cvc/geometry/contours.h: any format the app reads, by extension), at `contours`.
// A Reconstruct series is its .ser and the section files beside it: an archive's, or the files a
// page preloads with it; the .ser (else the one contour file) is what is read.
void decode_contours(DataDecode &d) {
  if (d.files.empty())
    fail("as: contours: " + d.spec.uri + " has no file");
  temp_dir tmp;
  std::string main;
  const std::vector<std::string> readable = cvc::contour_read_extensions(d.app);
  for (const auto &f : d.files) {
    const std::string at = tmp.put(basename(f.path), *f.bytes);
    const std::string ext = extension(f.path);
    if (ext == ".ser" ||
        (main.empty() && std::find(readable.begin(), readable.end(), ext) != readable.end() &&
         extension(main) != ".ser"))
      main = at;
  }
  if (main.empty())
    main = tmp.path() + "/" + basename(d.files.front().path);
  auto stack = std::make_shared<const cvc::contour_stack>(cvc::read_contours(d.app, main));
  d.out("contours").value(std::to_string(stack->polygon_count()));
  d.out("contours").data(boost::any(std::shared_ptr<const cvc::contour_stack>(std::move(stack))));
}

// A volume grid session (cvc/volume/volume_grid.h's JSON), at `session`.
void decode_volume_grid_session(DataDecode &d) {
  const auto &f = only_file(d);
  auto s = std::make_shared<const cvc::volume_grid_session>(
      cvc::volume_grid_session_from_json(*f.bytes, f.path));
  d.out("session").value(std::string("volume_grid_session"));
  d.out("session").data(boost::any(std::shared_ptr<const cvc::volume_grid_session>(std::move(s))));
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
  // A version 2 bundle's manifest (ARIADNE_WORLDS section 18.8): its content hash, its water
  // level and region when it has them, and the generator that made it (empty for a version 1).
  d.out("version").value(city.version);
  d.out("hash").value(city.hash);
  d.out("water_level")
      .value(city.water_level ? cvc::state_exec::format_number_shortest(*city.water_level)
                              : std::string());
  cvc::state &region = d.out("region");
  region.reset();
  region.value(std::string(city.region ? "1" : ""));
  if (city.region) {
    region("min_x").value(city.region->min_x);
    region("min_y").value(city.region->min_y);
    region("max_x").value(city.region->max_x);
    region("max_y").value(city.region->max_y);
  }
  cvc::state &gen = d.out("generator");
  gen.value(city.generator.name);
  gen("version").value(city.generator.version);
  gen("seed").value(std::to_string(city.generator.seed));
  gen("params").value(city.generator.params);
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
      {"point_classes", decode_point_classes},
      {"volume_grid_session", decode_volume_grid_session},
      {"contours", decode_contours},
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
// An app's generators: the built-in generated_world, then what its host registered.
struct generator_table {
  static constexpr const char *app_slot = "cvc.ariadne.data-generators";
  std::mutex m;
  std::map<std::string, DataGenerator> generators = {
      {"generated_world", generated_world_dataset},
  };
};

DataGenerator find_generator(cvc::app &app, const std::string &name) {
  generator_table &t = app.slot<generator_table>();
  std::lock_guard<std::mutex> lock(t.m);
  const auto it = t.generators.find(name);
  return it == t.generators.end() ? DataGenerator() : it->second;
}

std::string generator_names(cvc::app &app) {
  generator_table &t = app.slot<generator_table>();
  std::lock_guard<std::mutex> lock(t.m);
  std::string s;
  for (const auto &kv : t.generators)
    s += (s.empty() ? "" : ", ") + kv.first;
  return s;
}

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
  if (ext == ".vgr")
    return "point_classes";
  if (ext == ".ser" || ext == ".contour" || ext == ".cvcc")
    return "contours";
  return "bytes";
}

// -- encoders -------------------------------------------------------------------------------------

// The payload as a T, whichever way it is held: std::shared_ptr<const T>, std::shared_ptr<T> or T.
template <class T> std::shared_ptr<const T> held(const boost::any &a) {
  if (const auto *c = boost::any_cast<std::shared_ptr<const T>>(&a))
    return *c;
  if (const auto *m = boost::any_cast<std::shared_ptr<T>>(&a))
    return *m;
  if (const auto *v = boost::any_cast<T>(&a))
    return std::make_shared<const T>(*v);
  return nullptr;
}

std::string read_whole(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  if (!in)
    fail("cannot read back " + path);
  return ss.str();
}

std::string encode_bytes(const DataEncode &e) {
  if (const auto *b = boost::any_cast<DataBytes>(&e.payload))
    return *b ? **b : std::string();
  if (const auto *s = boost::any_cast<std::string>(&e.payload))
    return *s;
  fail("bytes: the data are not bytes");
}

std::string encode_volume(const DataEncode &e) {
  // A volume, or a list of them written as one file's variables (a format that holds several,
  // such as .rawv: a volume_grid's segmented classes, as VolumeGridRover's SegTmp.rawv).
  std::shared_ptr<const std::vector<cvc::volume>> vols = held<std::vector<cvc::volume>>(e.payload);
  if (!vols)
    if (const std::shared_ptr<const cvc::volume> v = held<cvc::volume>(e.payload))
      vols = std::make_shared<const std::vector<cvc::volume>>(1, *v);
  if (!vols)
    fail("volume: the data are not a volume");
  if (vols->empty())
    fail("volume: the list of volumes is empty");
  temp_dir tmp;
  const std::string p = tmp.path() + "/" + basename(e.file);
  cvc::writeVolumeFile(e.app, *vols, p);
  return read_whole(p);
}

std::string encode_point_classes(const DataEncode &e) {
  const std::shared_ptr<const cvc::point_classes> pc = held<cvc::point_classes>(e.payload);
  if (!pc)
    fail("point_classes: the data are not point classes");
  temp_dir tmp;
  const std::string p = tmp.path() + "/" + basename(e.file);
  cvc::write_point_classes(e.app, *pc, p);
  return read_whole(p);
}

// A contour stack in a one-file format, by the file's extension (.cvcc, .contour, .pcd): a format
// that writes files beside it (Reconstruct's .ser, Tiling's .config) can't go out as one download.
std::string encode_contours(const DataEncode &e) {
  const std::shared_ptr<const cvc::contour_stack> c = held<cvc::contour_stack>(e.payload);
  if (!c)
    fail("contours: the data are not a contour stack");
  temp_dir tmp;
  const std::string p = tmp.path() + "/" + basename(e.file);
  cvc::write_contours(e.app, *c, p);
  std::size_t files = 0;
  for (const auto &entry : fs::directory_iterator(tmp.path()))
    files += entry.is_regular_file() ? 1 : 0;
  if (files != 1)
    fail("contours: " + extension(e.file) +
         " writes files beside it; save as .cvcc, .contour or .pcd");
  return read_whole(p);
}

std::string encode_volume_grid_session(const DataEncode &e) {
  const std::shared_ptr<const cvc::volume_grid_session> s =
      held<cvc::volume_grid_session>(e.payload);
  if (!s)
    fail("volume_grid_session: the data are not a volume grid session");
  return cvc::volume_grid_session_json(*s);
}

// An app's encoders: the built-ins, then what its host registered.
struct encoder_table {
  static constexpr const char *app_slot = "cvc.ariadne.data-encoders";
  std::mutex m;
  std::map<std::string, DataEncoder> encoders = {
      {"bytes", encode_bytes},
      {"volume", encode_volume},
      {"point_classes", encode_point_classes},
      {"volume_grid_session", encode_volume_grid_session},
      {"contours", encode_contours},
  };
};

DataEncoder find_encoder(cvc::app &app, const std::string &as) {
  encoder_table &t = app.slot<encoder_table>();
  std::lock_guard<std::mutex> lock(t.m);
  const auto it = t.encoders.find(as);
  return it == t.encoders.end() ? DataEncoder() : it->second;
}

// Write a saved file's bytes: beside the document (a local one), or in a browser as a download.
void write_saved(const std::string &bytes, const std::string &file, const std::string &base) {
#if defined(__EMSCRIPTEN__)
  (void)base;
  if (!cvc_data_download(basename(file).c_str(), bytes.data(), static_cast<int>(bytes.size())))
    fail("cannot save " + file + ": this page has no document to download it from");
#else
  if (base.find("://") != std::string::npos)
    fail("cannot save " + file + ": the document is not a local file (" + base + ")");
  fs::path p(file);
  if (p.is_relative() && !base.empty())
    p = fs::path(base) / p;
  std::ofstream out(p, std::ios::binary | std::ios::trunc);
  out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  out.close();
  if (!out)
    fail("cannot write " + p.string());
#endif
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
  if (!spec.generate.empty()) // a made dataset: named by its uri when it has one, else its params
    return "generate\n" + spec.generate + "\n" + (spec.uri.empty() ? spec.params : spec.uri) +
           "\n" + spec.as;
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

void register_data_generator(cvc::app &app, const std::string &name, DataGenerator generator) {
  generator_table &t = app.slot<generator_table>();
  std::lock_guard<std::mutex> lock(t.m);
  if (generator)
    t.generators[name] = std::move(generator);
  else
    t.generators.erase(name);
}

bool has_data_generator(cvc::app &app, const std::string &name) {
  return static_cast<bool>(find_generator(app, name));
}

std::vector<std::string> data_generators(cvc::app &app) {
  generator_table &t = app.slot<generator_table>();
  std::lock_guard<std::mutex> lock(t.m);
  std::vector<std::string> out;
  for (const auto &kv : t.generators)
    out.push_back(kv.first);
  return out;
}

void register_data_encoder(cvc::app &app, const std::string &as, DataEncoder encoder) {
  encoder_table &t = app.slot<encoder_table>();
  std::lock_guard<std::mutex> lock(t.m);
  if (encoder)
    t.encoders[as] = std::move(encoder);
  else
    t.encoders.erase(as);
}

bool has_data_encoder(cvc::app &app, const std::string &as) {
  return static_cast<bool>(find_encoder(app, as));
}

std::vector<std::string> data_encoders(cvc::app &app) {
  encoder_table &t = app.slot<encoder_table>();
  std::lock_guard<std::mutex> lock(t.m);
  std::vector<std::string> out;
  for (const auto &kv : t.encoders)
    out.push_back(kv.first);
  return out;
}

std::string default_data_encoder(const boost::any &payload) {
  if (held<cvc::volume>(payload) || held<std::vector<cvc::volume>>(payload))
    return "volume";
  if (held<cvc::point_classes>(payload))
    return "point_classes";
  if (held<cvc::volume_grid_session>(payload))
    return "volume_grid_session";
  if (held<cvc::contour_stack>(payload))
    return "contours";
  if (boost::any_cast<DataBytes>(&payload) || boost::any_cast<std::string>(&payload))
    return "bytes";
  return std::string();
}

void data_save(cvc::app &app, const std::string &path, const std::string &file,
               const std::string &as, const std::string &base) {
  if (file.empty())
    fail("no file to save " + path + " as");
  cvc::state *n = cvc::state::instance(app).findThroughLinks(path);
  const boost::any payload = n ? n->data() : boost::any();
  if (payload.empty())
    fail(path + " holds no data to save");
  const std::string name = as.empty() ? default_data_encoder(payload) : as;
  if (name.empty())
    fail(path + " holds data no built-in encoder takes: name one (as)");
  const DataEncoder encoder = find_encoder(app, name);
  if (!encoder) {
    std::string known;
    for (const std::string &k : data_encoders(app))
      known += (known.empty() ? "" : ", ") + k;
    fail("no encoder as: " + name + " (known: " + known + ")");
  }
  write_saved(encoder(DataEncode{app, payload, file}), file, base);
}

DataSaved data_save_files(cvc::app &app, const std::string &path, const std::string &target,
                          const std::string &base, const DataSaveOptions &options) {
  if (target.empty())
    fail("no file or directory to save " + path + " as");
  cvc::state &root = cvc::state::instance(app);
  if (!root.findThroughLinks(path))
    fail("no dataset at " + path);
  // The dataset's files in its order, then the exports, each path once (a later one wins).
  std::vector<std::pair<std::string, std::string>> files;
  std::map<std::string, std::size_t> at;
  const auto add = [&](const std::string &name, const std::string &bytes) {
    const auto hit = at.find(name);
    if (hit != at.end()) {
      files[hit->second].second = bytes;
      return;
    }
    at[name] = files.size();
    files.emplace_back(name, bytes);
  };
  for (const std::string &f : data_files(app, path))
    if (const DataBytes b = data_file(app, path, f))
      add(f, *b);
  if (files.empty())
    fail(path + " holds no files to save (status: " + data_status(app, path) + ")");
  if (options.exports) {
    const std::shared_ptr<const cvc::world::world_geometry> w = dataset_world(app, path);
    if (!w)
      fail(path + " holds no world to export");
    std::optional<double> level;
    if (cvc::state *n = root.findThroughLinks(join(path, "water_level"))) {
      try {
        if (!n->value().empty())
          level = std::stod(n->value());
      } catch (const std::exception &) {
      }
    }
    for (const auto &[name, bytes] : cvc::world::city_bundle_exports(app, *w, level))
      add(name, bytes ? *bytes : std::string());
  }
  DataSaved out;
  out.files = files.size();
  for (const auto &f : files)
    out.bytes += f.second.size();
#if defined(__EMSCRIPTEN__)
  out.where = basename(extension(target) == ".zip" ? target : target + ".zip");
  write_saved(cvc::archive::pack_zip(files), out.where, base);
#else
  if (base.find("://") != std::string::npos)
    fail("cannot save " + target + ": the document is not a local file (" + base + ")");
  fs::path p(target);
  if (p.is_relative() && !base.empty())
    p = fs::path(base) / p;
  out.where = p.string();
  if (extension(target) == ".zip") {
    write_saved(cvc::archive::pack_zip(files), p.string(), std::string());
    return out;
  }
  cvc::world::city_bundle_files dir;
  for (auto &f : files)
    dir[f.first] = std::make_shared<const std::string>(std::move(f.second));
  try {
    cvc::world::write_city_bundle(p.string(), dir);
  } catch (const std::exception &e) {
    fail(std::string("cannot save ") + target + ": " + e.what());
  }
#endif
  return out;
}

std::string data_key(const DataSpec &spec) {
  return slug(spec.generate.empty() ? spec.uri : spec.generate) + "_" +
         sha256_of(canonical_request(spec)).substr(0, 8);
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

// A request's files under its store node's `files`, each one's bytes behind a pointer.
void publish_files(cvc::app &app, const std::string &node,
                   const std::vector<cvc::archive::file> &files) {
  cvc::state &fs_node = cvc::state::instance(app)(join(node, "files"));
  fs_node.reset();
  for (std::size_t i = 0; i < files.size(); ++i) {
    cvc::state &e = fs_node(std::to_string(i));
    e.property("size", std::to_string(files[i].bytes->size()));
    e.data(boost::any(DataBytes(files[i].bytes)));
    e.value(files[i].path);
  }
  fs_node.value(std::to_string(files.size()));
}

// The decoder a request's files go through: its as:, else city_bundle for a made one, else by
// what was fetched.
std::string decoder_name(const DataSpec &spec, bool many,
                         const std::vector<cvc::archive::file> &files) {
  if (!spec.as.empty())
    return spec.as;
  if (!spec.generate.empty())
    return "city_bundle";
  return default_decoder(many, files.empty() ? std::string() : files.front().path);
}

DataDecoder need_decoder(cvc::app &app, const std::string &as) {
  DataDecoder decoder = find_decoder(app, as);
  if (!decoder)
    fail("no decoder as: " + as +
         " (known: files, bytes, text, json, mesh, heightfield, volume, city_bundle, "
         "point_classes, volume_grid_session, contours, and any a host registered)");
  return decoder;
}

// The outputs of a request whose files are in (`also`'s, then the decoder's), and the keys that
// say it is ready, its rev up by one.
void finish_decode(cvc::app &a, const DataSpec &decoded, const std::string &node,
                   const std::vector<cvc::archive::file> &files, const DataDecoder &also,
                   const DataDecoder &decoder, const std::string &canonical,
                   const std::string &sha256, std::uint64_t size) {
  DataDecode d{a, decoded, node, files};
  if (also)
    also(d);
  if (decoder)
    decoder(d);
  cvc::state &n = cvc::state::instance(a)(node);
  n(std::string("as")).value(decoded.as);
  n(std::string("canonical")).value(canonical);
  n(std::string("sha256")).value(sha256);
  n(std::string("size")).value(std::to_string(size));
  n(std::string("error")).value(std::string());
  const int rev = read_or<int>(a, join(node, "rev"), 0);
  n(std::string("rev")).value(rev + 1);
  set_status(a, node, "ready", 1.0);
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
    std::vector<cvc::archive::file> files;
    bool many = true;
    std::string canonical, sha256;
    std::uint64_t size = 0;
    if (!spec.generate.empty()) {
      // Made, not fetched: the generator makes the files, holding the app as a decoder does.
      with_app(life, [&](cvc::app &a) {
        set_status(a, node, "generating", 0.1);
        const DataGenerator generator = find_generator(a, spec.generate);
        if (!generator)
          fail("no generator '" + spec.generate + "' (known: " + generator_names(a) + ")");
        const std::vector<cvc::archive::file> none;
        DataDecode d{a, spec, node, none};
        files = generator(d);
      });
      canonical = spec.uri.empty() ? "generate:" + spec.generate : spec.uri;
    } else {
      Fetched f = fetch(spec, registry, net, lim, root, with_app_resolve);
      size = f.directory ? 0 : f.bytes.size();
      canonical = f.canonical;
      sha256 = f.sha256;
      with_app(life, [&](cvc::app &a) { set_status(a, node, "unpacking", 0.5); });
      files = files_of(f, root, lim, &many);
    }
    DataSpec decoded = spec;
    // A generator that made no files, with no as:, wrote its outputs itself: nothing to decode.
    const bool made_outputs = !spec.generate.empty() && files.empty() && spec.as.empty();
    decoded.as = made_outputs ? std::string() : decoder_name(spec, many, files);
    with_app(life, [&](cvc::app &a) {
      publish_files(a, node, files);
      set_status(a, node, "decoding", 0.8);
    });
    with_app(life, [&](cvc::app &a) {
      const DataDecoder decoder = made_outputs ? DataDecoder() : need_decoder(a, decoded.as);
      finish_decode(a, decoded, node, files, nullptr, decoder, canonical, sha256, size);
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

std::string DataStore::put(const std::string &prefix, const DataSpec &spec,
                           std::vector<cvc::archive::file> files, const DataDecoder &also) {
  const std::string key = data_key(spec);
  const std::string node = join(kDataRoot, key);
  Impl &im = *impl_;
  {
    std::lock_guard<std::mutex> lock(im.mu);
    if (im.stopped)
      return node;
    im.lazy.erase(key);
    im.specs[key] = spec;
  }
  DataSpec decoded = spec;
  decoded.as = decoder_name(spec, true, files);
  {
    std::lock_guard<std::mutex> lock(im.life->mu); // as a load's decode holds it
    if (!im.life->app)
      return node;
    cvc::app &a = *im.life->app;
    try {
      cvc::state::instance(a)(join(node, "uri")).value(spec.uri);
      publish_files(a, node, files);
      set_status(a, node, "decoding", 0.8);
      const DataDecoder decoder = files.empty() ? DataDecoder() : need_decoder(a, decoded.as);
      finish_decode(a, decoded, node, files, also, decoder,
                    spec.uri.empty() ? "generate:" + spec.generate : spec.uri, std::string(), 0);
    } catch (const std::exception &e) {
      cvc::state::instance(a)(join(node, "error")).value(std::string(e.what()));
      set_status(a, node, "error", 1.0);
      throw;
    }
  }
  const std::string entry = resolve_bind(prefix, join(kDataRoot, spec.name));
  cvc::state &link = im.root()(entry);
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

std::string DataStore::set_output(const std::string &path, const std::string &item,
                                  const boost::any &payload) {
  const char *const own[] = {"status",    "error", "progress", "bytes", "sha256", "uri",
                             "canonical", "as",    "size",     "rev",   "files"};
  if (item.empty() || item.find(cvc::state::SEPARATOR) != std::string::npos)
    throw std::invalid_argument("data-put: '" + item + "' is not an output name (one word)");
  for (const char *k : own)
    if (item == k)
      throw std::invalid_argument("data-put: '" + item + "' is the store's own key, not an output");
  if (payload.empty())
    throw std::invalid_argument("data-put: nothing to put as " + item);
  const std::string node = resolve(path);
  if (node.empty())
    throw std::invalid_argument("data-put: '" + path + "' names no dataset");
  Impl &im = *impl_;
  std::lock_guard<std::mutex> lock(im.life->mu); // as a load's decode holds it
  if (!im.life->app)
    return node;
  cvc::app &a = *im.life->app;
  cvc::state &out = cvc::state::instance(a)(join(node, item));
  if (out.value().empty())
    out.value(item);
  out.data(payload);
  const int rev = read_or<int>(a, join(node, "rev"), 0);
  cvc::state::instance(a)(join(node, "rev")).value(rev + 1);
  return node;
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
