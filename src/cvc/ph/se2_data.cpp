/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::se2 data: reading cvc-plant-log run directories and .npy arrays (se2_train.h).
//
// cvc::ph sits below cvc::physics and cvc::nav, so it reads the plant log's files rather than
// linking plant_log: flights.npy is (F, T, 6 + inputs) float64, meta.json carries "flight_seed"
// and "dt".

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/ph/se2_train.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace ph {

namespace {

namespace pt = boost::property_tree;

std::string header_value(const std::string &header, const std::string &key) {
  const auto at = header.find("'" + key + "'");
  if (at == std::string::npos)
    throw std::runtime_error("npy: header has no " + key);
  const auto colon = header.find(':', at);
  return header.substr(colon + 1);
}

} // namespace

std::vector<double> read_npy(const std::string &path, std::vector<std::size_t> &shape) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("npy: cannot read " + path);
  unsigned char magic[8];
  in.read(reinterpret_cast<char *>(magic), 8);
  if (!in || std::memcmp(magic, "\x93NUMPY", 6) != 0)
    throw std::runtime_error("npy: " + path + " is not a .npy file");
  std::size_t header_len = 0;
  if (magic[6] == 1) {
    unsigned char b[2];
    in.read(reinterpret_cast<char *>(b), 2);
    header_len = b[0] | (b[1] << 8);
  } else {
    unsigned char b[4];
    in.read(reinterpret_cast<char *>(b), 4);
    header_len = b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<std::size_t>(b[3]) << 24);
  }
  std::string header(header_len, '\0');
  in.read(&header[0], static_cast<std::streamsize>(header_len));
  if (!in)
    throw std::runtime_error("npy: " + path + " is truncated");
  const std::string descr = header_value(header, "descr");
  const bool f8 = descr.find("<f8") != std::string::npos;
  const bool f4 = descr.find("<f4") != std::string::npos;
  if (!f8 && !f4)
    throw std::runtime_error("npy: " + path + " is not little-endian float64 or float32");
  if (header_value(header, "fortran_order").find("False") == std::string::npos)
    throw std::runtime_error("npy: " + path + " is in Fortran order");
  const std::string sh = header_value(header, "shape");
  const auto open = sh.find('('), close = sh.find(')');
  shape.clear();
  std::stringstream dims(sh.substr(open + 1, close - open - 1));
  std::string tok;
  std::size_t total = 1;
  while (std::getline(dims, tok, ','))
    if (tok.find_first_not_of(" \t") != std::string::npos) {
      shape.push_back(static_cast<std::size_t>(std::stoull(tok)));
      total *= shape.back();
    }
  std::vector<double> out(total);
  if (f8) {
    in.read(reinterpret_cast<char *>(out.data()), static_cast<std::streamsize>(total * 8));
  } else {
    std::vector<float> tmp(total);
    in.read(reinterpret_cast<char *>(tmp.data()), static_cast<std::streamsize>(total * 4));
    for (std::size_t i = 0; i < total; ++i)
      out[i] = tmp[i];
  }
  if (!in)
    throw std::runtime_error("npy: " + path + " is shorter than its shape");
  return out;
}

void write_npy(const std::string &path, const std::vector<double> &data,
               const std::vector<std::size_t> &shape) {
  std::size_t total = 1;
  std::string dims;
  for (std::size_t d : shape) {
    total *= d;
    dims += std::to_string(d) + ",";
  }
  if (shape.size() > 1)
    dims.pop_back();
  if (total != data.size())
    throw std::runtime_error("npy: data does not match the shape");
  std::string header = "{'descr': '<f8', 'fortran_order': False, 'shape': (" + dims + "), }";
  while ((10 + header.size() + 1) % 64 != 0)
    header += ' ';
  header += '\n';
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error("npy: cannot write " + path);
  const unsigned char magic[8] = {0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0};
  out.write(reinterpret_cast<const char *>(magic), 8);
  const unsigned char hl[2] = {static_cast<unsigned char>(header.size() & 0xff),
                               static_cast<unsigned char>(header.size() >> 8)};
  out.write(reinterpret_cast<const char *>(hl), 2);
  out.write(header.data(), static_cast<std::streamsize>(header.size()));
  out.write(reinterpret_cast<const char *>(data.data()),
            static_cast<std::streamsize>(data.size() * sizeof(double)));
  if (!out)
    throw std::runtime_error("npy: writing " + path + " failed");
}

se2_flights load_plant_log_runs(const std::vector<std::string> &dirs) {
  se2_flights all;
  bool first = true;
  for (const std::string &dir : dirs) {
    const std::filesystem::path d(dir);
    pt::ptree meta;
    try {
      pt::read_json((d / "meta.json").string(), meta);
    } catch (const pt::json_parser_error &e) {
      throw std::runtime_error(dir + ": " + e.what());
    }
    if (meta.get<std::string>("format", "") != "cvc-plant-log-1")
      throw std::runtime_error(dir + ": not a cvc-plant-log run");
    std::vector<std::size_t> shape;
    std::vector<double> data = read_npy((d / "flights.npy").string(), shape);
    if (shape.size() != 3 || shape[2] < 7)
      throw std::runtime_error(dir + ": flights.npy is not (flights, rows, 6 + inputs)");
    std::vector<int> seeds;
    for (const auto &kv : meta.get_child("flight_seed"))
      seeds.push_back(kv.second.get_value<int>());
    if (seeds.size() != shape[0])
      throw std::runtime_error(dir + ": meta.json's flight_seed does not match flights.npy");
    const double dt = meta.get<double>("dt");
    if (first) {
      all.rows = shape[1];
      all.width = static_cast<int>(shape[2]);
      all.dt = dt;
      first = false;
    } else if (shape[1] != all.rows || static_cast<int>(shape[2]) != all.width ||
               std::fabs(dt - all.dt) > 1e-12) {
      throw std::runtime_error(dir + ": rows, width or dt differ from the first run's");
    }
    for (std::size_t f = 0; f < shape[0]; ++f)
      for (int j = 6; j < all.width; ++j)
        data[f * shape[1] * shape[2] + j] = 0.0; // row 0's inputs
    all.data.insert(all.data.end(), data.begin(), data.end());
    all.seed.insert(all.seed.end(), seeds.begin(), seeds.end());
    all.count += shape[0];
  }
  if (first)
    throw std::runtime_error("load_plant_log_runs: no run directories");
  return all;
}

se2_flights select_seeds(const se2_flights &f, const std::vector<int> &seeds, bool keep) {
  if (f.seed.size() != f.count)
    throw std::invalid_argument("select_seeds: the flights carry no seeds");
  se2_flights out;
  out.rows = f.rows;
  out.width = f.width;
  out.dt = f.dt;
  const std::size_t n = f.rows * f.width;
  for (std::size_t i = 0; i < f.count; ++i) {
    const bool in = std::find(seeds.begin(), seeds.end(), f.seed[i]) != seeds.end();
    if (in != keep)
      continue;
    out.data.insert(out.data.end(), f.flight(i), f.flight(i) + n);
    out.seed.push_back(f.seed[i]);
    ++out.count;
  }
  return out;
}

} // namespace ph
} // namespace cvc
