/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH's plant, potential, command and allocation (coph_model.h, coph_allocation.h: C2 of
// docs/roadmap/NAV-COPH-ROADMAP.md) against GRL-SNAM's grl_snam.coph. The golden,
// src/cvc/tests/data/coph_golden.json, is written by GRL-SNAM (python -m
// grl_snam.tools.coph_golden --out DIR); this side reproduces its FLOAT64 sections:
//   G2   plant       model.flow and model.balance, 40 (x, u, d, dt) cases, d from 1e-8 to 10
//   G3   potential   risk, V, dV/dq, and the command at 25 points on 5 scenes, legacy and progress
//   G11  allocation  allocation.allocate, 60 (u, heading, speed, car) cases
// Each section's tier is in the file: |mine - golden| <= rtol |golden| + atol (1e-12 and 1e-12).
// The golden's doubles are raw bits, 16 hex digits of the uint64.
//
// C++-only: coph_exp and coph_expm1 are correctly rounded (against mpmath), and the energy account
// closes over a 2,000-step run.

#include <algorithm>
#include <bit>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/nav/coph_allocation.h>
#include <cvc/nav/coph_model.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <iomanip>
#include <limits>
#include <optional>
#include <string>
#include <vector>

using namespace cvc::nav;
namespace pt = boost::property_tree;
namespace fs = std::filesystem;

namespace {

const fs::path kData = fs::path(__FILE__).parent_path() / "data";

const pt::ptree &golden() {
  static const pt::ptree t = [] {
    pt::ptree r;
    pt::read_json((kData / "coph_golden.json").string(), r);
    return r;
  }();
  return t;
}

std::string str(const pt::ptree &n) { return n.get_value<std::string>(); }

double from_hex(const std::string &h) {
  return std::bit_cast<double>(static_cast<std::uint64_t>(std::stoull(h, nullptr, 16)));
}

double num(const pt::ptree &n) { return from_hex(str(n)); }

std::vector<double> doubles(const pt::ptree &arr) {
  std::vector<double> v;
  for (const auto &c : arr)
    v.push_back(num(c.second));
  return v;
}

coph_vec2 vec2(const pt::ptree &arr) {
  const std::vector<double> v = doubles(arr);
  EXPECT_EQ(v.size(), 2u);
  return {v.at(0), v.at(1)};
}

const pt::ptree &at(const pt::ptree &arr, std::size_t i) {
  auto it = arr.begin();
  std::advance(it, static_cast<std::ptrdiff_t>(i));
  return it->second;
}

// A section's tier, and the comparison it sets. It also keeps how close the values came: the
// worst |mine - golden| as a fraction of the allowance, and how many were bit for bit.
struct tier {
  double rtol, atol;
  mutable double worst = 0.0;
  mutable int compared = 0, exact = 0;
  explicit tier(const pt::ptree &section)
      : rtol(std::stod(section.get<std::string>("tier.rtol"))),
        atol(std::stod(section.get<std::string>("tier.atol"))) {}
  bool close(double mine, double gold) const {
    const double allowed = rtol * std::fabs(gold) + atol;
    worst = std::max(worst, std::fabs(mine - gold) / allowed);
    ++compared;
    exact += mine == gold;
    return std::fabs(mine - gold) <= allowed;
  }
  void report(const char *what) const {
    std::printf("%s: %d of %d values bit for bit; the worst is %.3g of the tier's allowance\n",
                what, exact, compared, worst);
  }
};

::testing::AssertionResult at_tier(const tier &t, double mine, double gold,
                                   const std::string &what) {
  if (t.close(mine, gold))
    return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure()
         << std::setprecision(17) << what << ": " << mine << " vs the golden's " << gold
         << " (off by " << (mine - gold) << ")";
}

#define EXPECT_AT_TIER(t, mine, gold, what) EXPECT_TRUE(at_tier(t, mine, gold, what))

// ---------------------------------------------------------------------------
// coph_exp and coph_expm1: correctly rounded
// ---------------------------------------------------------------------------

// (x, expm1(x), exp(x)) as raw bits, each rounded to nearest from mpmath at 400 bits: the 71
// arguments the G2 plant cases take (-d dt and -2 d dt), the reduction's boundaries near ln 2 / 2,
// tiny and huge arguments, and 80 log-uniform ones (|x| from 1e-10 to 10^2.5, seed 20261114).
// kNone: exp's result is outside the normal range.
constexpr std::uint64_t kNone = 0x7ff8dead0000beefull;
struct exp_row {
  std::uint64_t x, expm1, exp;
};
constexpr exp_row kExpTable[] = {
    {0xc086280000000000, 0xbff0000000000000, kNone},
    {0xc086200000000000, 0xbff0000000000000, 0x0017c8ab2288c9ab},
    {0xc085e00000000000, 0xbff0000000000000, 0x00d14f2b0fb9307f},
    {0xc068bb27e9baffa1, 0xbff0000000000000, 0x2e17aa9f28aaaf40},
    {0xc059000000000000, 0xbff0000000000000, 0x36ea8c1f14e2af5d},
    {0xc0571c39065bcd22, 0xbff0000000000000, 0x3798dc9b84545e34},
    {0xc04d346299de4781, 0xbff0000000000000, 0x3aaa991d770e73ab},
    {0xc04a4c43d394a344, 0xbff0000000000000, 0x3b31641a8774a089},
    {0xc049000000000000, 0xbff0000000000000, 0x3b6d257d547e083f},
    {0xc044c99a7f464c27, 0xbff0000000000000, 0x3c3038ee53d4d221},
    {0xc044000000000000, 0xbff0000000000000, 0x3c539792499b1a24},
    {0xc043400000000000, 0xbff0000000000000, 0x3c75f38ed3eb9bdb},
    {0xc042c00000000000, 0xbff0000000000000, 0x3c8dd5c566301ec8},
    {0xc042b33333333333, 0xbfefffffffffffff, 0x3c907c8570407bb7},
    {0xc034000000000000, 0xbfeffffffee4b79b, 0x3e21b48655f37267},
    {0xc028bf3813387acf, 0xbfeffff72169ccf9, 0x3ed1bd2c660dcaab},
    {0xc028000000000000, 0xbfeffff31d59e25e, 0x3ed9c54c3b43bc8b},
    {0xc027099795a03633, 0xbfefffeb26635d93, 0x3ee4d99ca26d7ad5},
    {0xc024000000000000, 0xbfefffa0ca192a6e, 0x3f07cd79b5647c9b},
    {0xc01c18e00187c0c1, 0xbfeff8b58a488e33, 0x3f4d29d6ddc73347},
    {0xc018000000000000, 0xbfefebb1ae0eec2b, 0x3f644e51f113d4d6},
    {0xc01766edaef7ae3c, 0xbfefe86b879cf3c3, 0x3f679478630c3cf9},
    {0xc012f4065f51a657, 0xbfefb84aa6ccb341, 0x3f81ed564cd32fd9},
    {0xc0104cc17f4819a0, 0xbfef74cb34dcdae3, 0x3f9166996464a3a1},
    {0xc00ccccccccccccc, 0xbfef202a0035e27d, 0x3f9bfabff943b06b},
    {0xc008000000000000, 0xbfee6824f33314f5, 0x3fa97db0ccceb0af},
    {0xc004000000000000, 0xbfed5f8f47ed617b, 0x3fb50385c094f425},
    {0xc003333333333333, 0xbfed18d6a9aec808, 0x3fb7394ab289bfc4},
    {0xc000464ae09aeddd, 0xbfebd0bd6dfcc2bb, 0x3fc0bd0a480cf513},
    {0xbffccccccccccccc, 0xbfeab5df1b20bd73, 0x3fc52883937d0a34},
    {0xbffb333333333333, 0xbfea2774e1d59d7e, 0x3fc7622c78a98a07},
    {0xbff8000000000000, 0xbfe8dc1e236d28f9, 0x3fcc8f87724b5c1d},
    {0xbff3333333333333, 0xbfe65c9df4c2f691, 0x3fd346c4167a12df},
    {0xbff051eb851eb852, 0xbfe47601999b504d, 0x3fd713fcccc95f67},
    {0xbff0000000000000, 0xbfe43a54e4e98864, 0x3fd78b56362cef38},
    {0xbfeccccccccccccc, 0xbfe2fd619ffbc8f1, 0x3fda053cc0086e1f},
    {0xbfeb333333333333, 0xbfe2529debaea931, 0x3fdb5ac428a2ad9e},
    {0xbfe70a3d70a3d70a, 0xbfe06c86886cb9f7, 0x3fdf26f2ef268c13},
    {0xbfe62e42fefa39ef, 0xbfe0000000000000, 0x3fe0000000000000},
    {0xbfe3333333333333, 0xbfdce04528d3f639, 0x3fe18fdd6b9604e3},
    {0xbfe051eb851eb852, 0xbfd9917afde6f563, 0x3fe33742810c854e},
    {0xbfe0000000000000, 0xbfd92e9a0720d3ec, 0x3fe368b2fc6f960a},
    {0xbfd84c897133540b, 0xbfd437eec96d5ce2, 0x3fe5e4089b49518f},
    {0xbfd70a3d70a3d70a, 0xbfd359456515371f, 0x3fe6535d4d756470},
    {0xbfd62e42fefa39f0, 0xbfd2bec333018867, 0x3fe6a09e667f3bcc},
    {0xbfd62e42fefa39ef, 0xbfd2bec333018867, 0x3fe6a09e667f3bcd},
    {0xbfd62e42fefa39ee, 0xbfd2bec333018866, 0x3fe6a09e667f3bcd},
    {0xbfd5eaeac8e07034, 0xbfd28f0b70bb0343, 0x3fe6b87a47a27e5e},
    {0xbfd3333333333334, 0xbfd0966f2c7907f7, 0x3fe7b4c869c37c05},
    {0xbfd3333333333333, 0xbfd0966f2c7907f6, 0x3fe7b4c869c37c05},
    {0xbfd0000000000000, 0xbfcc5041854df7d4, 0x3fe8ebef9eac820b},
    {0xbfca1cac083126e9, 0xbfc79eedd450e43d, 0x3fea18448aebc6f1},
    {0xbfc70a3d70a3d70a, 0xbfc515dd9f5531bf, 0x3feaba88982ab390},
    {0xbfc6f124cd78ec1c, 0xbfc500e5423951cf, 0x3feabfc6af71ab8c},
    {0xbfc3333333333334, 0xbfc1d4522eec8c81, 0x3feb8aeb7444dce0},
    {0xbfc3333333333333, 0xbfc1d4522eec8c80, 0x3feb8aeb7444dce0},
    {0xbfc3079cb97ef593, 0xbfc1aec79a9d1bdf, 0x3feb944e1958b908},
    {0xbfc26e978d4fdf3b, 0xbfc12a971c0a1073, 0x3febb55a38fd7be3},
    {0xbfbaa7f8750e6698, 0xbfb950b66032d5c4, 0x3fecd5e933f9a547},
    {0xbfba1cac083126e9, 0xbfb8d30e2842b8c4, 0x3fece59e3af7a8e7},
    {0xbfb999999999999a, 0xbfb85c933156a62c, 0x3fecf46d99d52b3a},
    {0xbfb70a3d70a3d70a, 0xbfb6089b18772ed0, 0x3fed3eec9cf11a26},
    {0xbfb5c28f5c28f5c3, 0xbfb4dc610185805a, 0x3fed6473dfcf4ff5},
    {0xbfb3333333333334, 0xbfb27f6723244fc6, 0x3fedb0131b9b7607},
    {0xbfb2e938fede1b8c, 0xbfb23abb6bdad303, 0x3fedb8a89284a5a0},
    {0xbfb26e978d4fdf3b, 0xbfb1c8ba2a92c300, 0x3fedc6e8baada7a0},
    {0xbfaeb851eb851eb8, 0xbfadd109ff722791, 0x3fee22ef6008dd87},
    {0xbfa856db5fc87bb0, 0xbfa7c5136d3872b8, 0x3fee83aec92c78d4},
    {0xbfa5c28f5c28f5c3, 0xbfa54dd84b942168, 0x3feeab227b46bdea},
    {0xbfa26e978d4fdf3b, 0xbfa21aaade2c1b27, 0x3feede55521d3e4e},
    {0xbf9eb851eb851eb8, 0xbf9e4386ab708619, 0x3fef0de3caa47bcf},
    {0xbf947ae147ae147b, 0xbf9446cc8a427492, 0x3fef5dc99badec5b},
    {0xbf926e978d4fdf3b, 0xbf924460da7645ce, 0x3fef6ddcf92c4dd2},
    {0xbf8eb851eb851eb8, 0xbf8e7da1a40bb638, 0x3fef8609796fd127},
    {0xbf8e3d6c039b3de5, 0xbf8e048c823f4720, 0x3fef87edcdf702e3},
    {0xbf889374bc6a7efa, 0xbf886ddb993e10f3, 0x3fef9e48919b07bc},
    {0xbf847ae147ae147b, 0xbf8460c0b518c09f, 0x3fefae7cfd2b9cfe},
    {0xbf7eb851eb851eb8, 0xbf7e9ae7030764f0, 0x3fefc2ca31f9f136},
    {0xbf789374bc6a7efa, 0xbf78809e8c078e0e, 0x3fefcefec2e7f0e4},
    {0xbf6d1dc7b13a9310, 0xbf6d108ca9f94e75, 0x3fefe2ef735606b2},
    {0xbf63a92a30553261, 0xbf63a3213c9dc23b, 0x3fefec5cdec3623e},
    {0xbf53a92a30553261, 0xbf53a62567618241, 0x3feff62ced4c4f3f},
    {0xbf50624dd2f1a9fc, 0xbf50603521cac48c, 0x3feff7cfe56f1a9e},
    {0xbf4ddd19426bc6b9, 0xbf4dd99db115c657, 0x3feff8899893ba8e},
    {0xbf40624dd2f1a9fc, 0xbf4061416eeb48bc, 0x3feffbe7afa4452e},
    {0xbf2a36e2eb1c432d, 0xbf2a36372193c1a3, 0x3feffe5c9c8de6c4},
    {0xbf242470e8358863, 0xbf24240b7bd4f68e, 0x3feffebdbf4842b1},
    {0xbf1f75104d551d69, 0xbf1f74949caa08fa, 0x3fefff045b5b1ab0},
    {0xbf1ae2f8704cc087, 0xbf1ae29e14a0e7c6, 0x3fefff28eb0f5af9},
    {0xbf1a36e2eb1c432d, 0xbf1a368d059c5f9d, 0x3fefff2e4b97d31d},
    {0xbf0f75104d551d69, 0xbf0f74d274ae838a, 0x3fefff822cb62d46},
    {0xbf07e66d9f2c6a69, 0xbf07e649ebde3b87, 0x3fefffa066d85087},
    {0xbefb5c67fbf323af, 0xbefb5c509706291b, 0x3fefffc9475ed1f4},
    {0xbef92a737110e454, 0xbef92a5fa68e9351, 0x3fefffcdab40b2e3},
    {0xbee92a737110e454, 0xbee92a698bcd23bf, 0x3fefffe6d5967433},
    {0xbee864203fc4fd1b, 0xbee86416f412850e, 0x3fefffe79be90bed},
    {0xbee4f8b588e368f1, 0xbee4f8aea9acf2c9, 0x3fefffeb07515653},
    {0xbed4f8b588e368f1, 0xbed4f8b21947cdca, 0x3feffff583a6f35c},
    {0xbec92a737110e454, 0xbec92a70f7bf77ab, 0x3feffff9b563c210},
    {0xbec0c6f7a0b5ed8d, 0xbec0c6f6873c61c6, 0x3feffffbce425e31},
    {0xbeb421f5f40d8376, 0xbeb421f52964110a, 0x3feffffd7bc15ad3},
    {0xbeb0c6f7a0b5ed8d, 0xbeb0c6f713f92497, 0x3feffffde7211d81},
    {0xbea421f5f40d8376, 0xbea421f58eb8c8ec, 0x3feffffebde0a714},
    {0xbe9c5bb2cf1de551, 0xbe9c5bb26a97e781, 0x3fefffff1d226cab},
    {0xbe94de202a49855c, 0xbe94de1ff3dacd6e, 0x3fefffff590f0061},
    {0xbe901b2b29a4692b, 0xbe901b2b0937606c, 0x3fefffff7f26a7b6},
    {0xbe8c1fc91e87580c, 0xbe8c1fc8ed17d936, 0x3fefffff8f80dc4c},
    {0xbe863ff601c6bbe5, 0xbe863ff5e2d5d7cd, 0x3fefffffa7002875},
    {0xbe843dcd79aa0035, 0xbe843dcd600e8fe1, 0x3fefffffaf08ca80},
    {0xbe83f875d3f6bafb, 0xbe83f875bb0990f0, 0x3fefffffb01e2914},
    {0xbe801b2b29a4692b, 0xbe801b2b196de4c0, 0x3fefffffbf93539a},
    {0xbe7ad7f29abcaf48, 0xbe7ad7f2843813ce, 0x3fefffffca501af8},
    {0xbe6ad7f29abcaf48, 0xbe6ad7f28f7a6188, 0x3fefffffe5280d71},
    {0xbe55798ee2308c3a, 0xbe55798ede9635e7, 0x3feffffff5433891},
    {0xbe49c511dc3a41df, 0xbe49c511d9a22cae, 0x3feffffff98ebb8a},
    {0xbe45798ee2308c3a, 0xbe45798ee0636111, 0x3feffffffaa19c48},
    {0xbe3f6dd2ad7ed738, 0xbe3f6dd2ab90f327, 0x3feffffffc1245ab},
    {0xbe39c511dc3a41df, 0xbe39c511daee3746, 0x3feffffffcc75dc5},
    {0xbe261d37fe00c53e, 0xbe261d37fd868301, 0x3feffffffe9e2c80},
    {0xbe249da7e361ce4c, 0xbe249da7e2f78d77, 0x3feffffffeb62582},
    {0xbe149da7e361ce4c, 0xbe149da7e32cade2, 0x3fefffffff5b12c1},
    {0xbe112e0be826d695, 0xbe112e0be801f1d9, 0x3fefffffff768fa1},
    {0xbe05b9d22b6d97ec, 0xbe05b9d22b5017b6, 0x3fefffffffa918b7},
    {0xbe012e0be826d695, 0xbe012e0be8146437, 0x3fefffffffbb47d0},
    {0xbe006f23caa3f1ea, 0xbe006f23ca93109e, 0x3fefffffffbe4371},
    {0xbd719799812dea11, 0xbd719799812de065, 0x3fefffffffffdcd1},
    {0xbc90000000000000, 0xbc90000000000000, 0x3ff0000000000000},
    {0xbbc79ca10c924223, 0xbbc79ca10c924223, 0x3ff0000000000000},
    {0x81a56e1fc2f8f359, 0x81a56e1fc2f8f359, 0x3ff0000000000000},
    {0x8000000000000001, 0x8000000000000001, 0x3ff0000000000000},
    {0x0000000000000001, 0x0000000000000001, 0x3ff0000000000000},
    {0x01a56e1fc2f8f359, 0x01a56e1fc2f8f359, 0x3ff0000000000000},
    {0x3bc79ca10c924223, 0x3bc79ca10c924223, 0x3ff0000000000000},
    {0x3c90000000000000, 0x3c90000000000000, 0x3ff0000000000000},
    {0x3d719799812dea11, 0x3d719799812df3bd, 0x3ff0000000001198},
    {0x3de8469c41138594, 0x3de8469c411cbad7, 0x3ff00000000c234e},
    {0x3df46f4accc83045, 0x3df46f4accd53ce6, 0x3ff0000000146f4b},
    {0x3df947980ede470b, 0x3df947980ef23f89, 0x3ff0000000194798},
    {0x3e03c9666cc448bd, 0x3e03c9666cdcc0f7, 0x3ff00000002792cd},
    {0x3e0d00f034b8f938, 0x3e0d00f034ed8c9f, 0x3ff00000003a01e0},
    {0x3e1606e0fd0729e3, 0x3e1606e0fd43cfbe, 0x3ff0000000581b84},
    {0x3e206bccbff411dc, 0x3e206bccc0377b9b, 0x3ff0000000835e66},
    {0x3e2f037038ecaa5e, 0x3e2f037039dd1fac, 0x3ff0000000f81b82},
    {0x3e31943818919d23, 0x3e319438192c1fc5, 0x3ff0000001194382},
    {0x3e45798ee2308c3a, 0x3e45798ee3fdb764, 0x3ff0000002af31dc},
    {0x3e513e0fa533d936, 0x3e513e0fa7867375, 0x3ff00000044f83ea},
    {0x3e79fc82a34d0489, 0x3e79fc82b867593f, 0x3ff0000019fc82b8},
    {0x3ea09c6426968d48, 0x3ea09c646b9191c9, 0x3ff0000084e3235d},
    {0x3ea2dee9ff54ecdd, 0x3ea2deea585bae9a, 0x3ff0000096f752c3},
    {0x3eb81c4d2bc306ca, 0x3eb81c4e4e6bdc81, 0x3ff0000181c4e4e7},
    {0x3ec88004f4c37de1, 0x3ec880074d049707, 0x3ff000031000e9a1},
    {0x3ec92a737110e454, 0x3ec92a75ea62a400, 0x3ff00003254ebd4c},
    {0x3ecf8cd199388c85, 0x3ecf8cd57ca11755, 0x3ff00003f19aaf94},
    {0x3ed218ceba35485f, 0x3ed218d14934794e, 0x3ff000048634524d},
    {0x3ee3469fac327d90, 0x3ee346a57a706511, 0x3ff00009a352bd38},
    {0x3f22f52deff90ea1, 0x3f22f587ca66a1e1, 0x3ff00097ac3e5335},
    {0x3f23356d33836f62, 0x3f2335c97301c520, 0x3ff00099ae4b980e},
    {0x3f2afe330bd83779, 0x3f2afee936bdada3, 0x3ff000d7f749b5ed},
    {0x3f50624dd2f1a9fc, 0x3f506466dfb8cf3a, 0x3ff0041919b7ee34},
    {0x3f5986f6fc457cf8, 0x3f598c0ef066fb01, 0x3ff0066303bc19bf},
    {0x3f8589044ac84ef9, 0x3f85a61a9f235cdd, 0x3ff02b4c353e46ba},
    {0x3f91241fa77941a4, 0x3f91490e7fcd3211, 0x3ff0452439ff34c8},
    {0x3fb346f735fe7c51, 0x3fb4058626870cff, 0x3ff14058626870d0},
    {0x3fb999999999999a, 0x3fbaec7b35a00d3a, 0x3ff1aec7b35a00d4},
    {0x3fcc78889ce7f109, 0x3fcfe2b7b8eb38e1, 0x3ff3fc56f71d671c},
    {0x3fd0000000000000, 0x3fd22d78f0fa061a, 0x3ff48b5e3c3e8186},
    {0x3fd4de0f567d84b5, 0x3fd8abdbe37edf6f, 0x3ff62af6f8dfb7dc},
    {0x3fd62e42fefa39ee, 0x3fda827999fcef31, 0x3ff6a09e667f3bcc},
    {0x3fd62e42fefa39ef, 0x3fda827999fcef32, 0x3ff6a09e667f3bcc},
    {0x3fd62e42fefa39f0, 0x3fda827999fcef33, 0x3ff6a09e667f3bcd},
    {0x3fd7ee3a161278d1, 0x3fdd04bbd06f04d4, 0x3ff7412ef41bc135},
    {0x3fe0000000000000, 0x3fe4c2531c3c0d38, 0x3ffa61298e1e069c},
    {0x3fe3c43d068ababb, 0x3feb596d8058f2d2, 0x3ffdacb6c02c7969},
    {0x3fe62e42fefa39ef, 0x3ff0000000000000, 0x4000000000000000},
    {0x3fe6a099778e82eb, 0x3ff07323b139f451, 0x40003991d89cfa28},
    {0x3ff0000000000000, 0x3ffb7e151628aed3, 0x4005bf0a8b145769},
    {0x3ff374c976eaac0f, 0x4002fd5aa23c7383, 0x400afd5aa23c7383},
    {0x3ff91dd6333538ee, 0x400e71e496553ce1, 0x401338f24b2a9e71},
    {0x3ff94339233358e5, 0x400ecc22f7455007, 0x401366117ba2a803},
    {0x4004000000000000, 0x40265d6fd931e0bb, 0x40285d6fd931e0bb},
    {0x4024000000000000, 0x40d5825dcf950560, 0x40d5829dcf950560},
    {0x4024a201b913a313, 0x40dd83f5c921ec5c, 0x40dd8435c921ec5c},
    {0x402a18410c21ff55, 0x411c50099f181a1e, 0x411c500d9f181a1e},
    {0x4034000000000000, 0x41bceb088a68e804, 0x41bceb088b68e804},
    {0x403d7fb8b8859106, 0x42978df7c158aaaa, 0x42978df7c158aeaa},
    {0x404213c1f2e61fa6, 0x4331df7d7fb08248, 0x4331df7d7fb08249},
    {0x4042b33333333333, 0x434f0e4e21b02686, 0x434f0e4e21b02687},
    {0x4042c00000000000, 0x4351293919c201e5, 0x4351293919c201e5},
    {0x4043649b59128b49, 0x436f0be47b351d95, 0x436f0be47b351d95},
    {0x4049000000000000, 0x44719103e4080b45, 0x44719103e4080b45},
    {0x4059000000000000, 0x48f3494a9b171bf5, 0x48f3494a9b171bf5},
    {0x405a982a91cf94a3, 0x49862b1cf7d3a987, 0x49862b1cf7d3a987},
    {0x405afaddf2152710, 0x49a9e8955c582d78, 0x49a9e8955c582d78},
    {0x405bbb13938d8451, 0x49f050be4dac7368, 0x49f050be4dac7368},
    {0x40614906ac58804d, 0x4c669bffa143c76c, 0x4c669bffa143c76c},
    {0x406ec8b2f5fc09ba, 0x5623a1ea553a6e66, 0x5623a1ea553a6e66},
    {0x4085e00000000000, 0x7f0d945df4f8ec8e, 0x7f0d945df4f8ec8e},
    {0x4086280000000000, 0x7fdd422d2be5dc9b, 0x7fdd422d2be5dc9b},
};

TEST(NavCophExp, CorrectlyRoundedAgainstMpmath) {
  int checked = 0;
  for (const exp_row &r : kExpTable) {
    const double x = std::bit_cast<double>(r.x);
    EXPECT_EQ(std::bit_cast<std::uint64_t>(coph_expm1(x)), r.expm1) << "expm1(" << x << ")";
    if (r.exp != kNone)
      EXPECT_EQ(std::bit_cast<std::uint64_t>(coph_exp(x)), r.exp) << "exp(" << x << ")";
    ++checked;
  }
  EXPECT_EQ(checked, 198);
}

TEST(NavCophExp, SpecialValues) {
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(std::signbit(coph_expm1(-0.0)) && coph_expm1(-0.0) == 0.0);
  EXPECT_EQ(coph_exp(0.0), 1.0);
  EXPECT_EQ(coph_exp(-0.0), 1.0);
  EXPECT_TRUE(std::isnan(coph_expm1(std::nan(""))));
  EXPECT_TRUE(std::isnan(coph_exp(std::nan(""))));
  EXPECT_EQ(coph_expm1(inf), inf);
  EXPECT_EQ(coph_exp(inf), inf);
  EXPECT_EQ(coph_expm1(-inf), -1.0);
  EXPECT_EQ(coph_exp(-inf), 0.0);
  EXPECT_EQ(coph_expm1(710.0), inf);
  EXPECT_EQ(coph_exp(-746.0), 0.0);
  EXPECT_TRUE(std::isfinite(coph_exp(709.782712893384)));
}

// ---------------------------------------------------------------------------
// G2: the plant
// ---------------------------------------------------------------------------

TEST(NavCophPlant, ReproducesG2) {
  const pt::ptree &plant = golden().get_child("plant");
  const tier t(plant);
  int cases = 0, exact = 0;
  for (const auto &[_, c] : plant.get_child("cases")) {
    const std::string name = c.get<std::string>("name");
    const std::vector<double> xv = doubles(c.get_child("x"));
    ASSERT_EQ(xv.size(), 4u) << name;
    const coph_state x{xv[0], xv[1], xv[2], xv[3]};
    const coph_vec2 u = vec2(c.get_child("u"));
    const double d = num(c.get_child("d")), dt = num(c.get_child("dt"));
    const pt::ptree &out = c.get_child("out");

    const coph_state after = coph_flow<double>(x, u, d, dt);
    const std::vector<double> want = doubles(out.get_child("flow"));
    ASSERT_EQ(want.size(), 4u) << name;
    bool same = true;
    for (int i = 0; i < 4; ++i) {
      EXPECT_AT_TIER(t, after[i], want[i], name + " flow[" + std::to_string(i) + "]");
      same = same && after[i] == want[i];
    }
    const coph_balance_terms b = coph_balance(x, u, d, dt);
    const std::pair<const char *, double> terms[] = {{"work", b.work},
                                                     {"heat", b.heat},
                                                     {"kinetic_change", b.kinetic_change},
                                                     {"residual", b.residual}};
    for (const auto &[key, mine] : terms) {
      const double gold = num(out.get_child(key));
      EXPECT_AT_TIER(t, mine, gold, name + " " + key);
      same = same && mine == gold;
    }
    exact += same;
    ++cases;
  }
  EXPECT_EQ(cases, 40);
  // Correctly rounded exp and expm1, in torch's order, give torch's bits.
  std::printf("G2: %d of %d cases bit for bit\n", exact, cases);
  t.report("G2");
}

// The small-damping case that needs correctly rounded expm1: dt - I cancels to about d dt^2 / 2, so
// one ULP of expm1 moves q_after by about 1e-8, a thousand times the tier.
TEST(NavCophPlant, SmallDampingNeedsTheCorrectlyRoundedExpm1) {
  const coph_state x{11.0, 7.5, 1.0, 0.0};
  const coph_vec2 u{2.12, -2.12};
  const double d = 1e-8, dt = 1.0;
  const coph_state mine = coph_flow<double>(x, u, d, dt);
  // The same formula with an expm1 one ULP off.
  const double off = std::nextafter(coph_expm1(-d * dt), 0.0);
  const double integral = -off / d;
  const double q_off = (x[0] + integral * x[2]) + ((dt - integral) * u[0]) / d;
  EXPECT_GT(std::fabs(q_off - mine[0]), 1e-9);
}

// ---------------------------------------------------------------------------
// G3: the potential, its gradient and the command
// ---------------------------------------------------------------------------

struct scene {
  std::string name;
  std::vector<coph_vec2> centers;
  std::vector<double> weights;
  std::vector<coph_shape> obstacles;
  std::pair<double, double> policy_bounds;
};

std::vector<scene> scenes() {
  std::vector<scene> out;
  for (const auto &[_, s] : golden().get_child("potential.scenes")) {
    scene sc;
    sc.name = s.get<std::string>("name");
    const std::vector<double> flat = doubles(s.get_child("centers"));
    for (std::size_t i = 0; i + 1 < flat.size(); i += 2)
      sc.centers.push_back({flat[i], flat[i + 1]});
    sc.weights = doubles(s.get_child("weights"));
    for (const auto &[__, o] : s.get_child("obstacles")) {
      const std::vector<double> v = doubles(o);
      if (v.size() == 5)
        sc.obstacles.push_back(coph_shape::segment(v[0], v[1], v[2], v[3], v[4]));
      else
        sc.obstacles.push_back(coph_shape::disk(v.at(0), v.at(1), v.at(2)));
    }
    const coph_vec2 box = vec2(s.get_child("policy_bounds"));
    sc.policy_bounds = {box[0], box[1]};
    out.push_back(std::move(sc));
  }
  return out;
}

TEST(NavCophPotential, ReproducesG3) {
  const pt::ptree &pot = golden().get_child("potential");
  const tier t(pot);
  const double width = num(pot.get_child("rbf_width"));
  const double limit = num(pot.get_child("limit"));
  const std::vector<scene> all = scenes();
  ASSERT_EQ(all.size(), 5u);
  int cases = 0, progress = 0;
  for (const auto &[_, c] : pot.get_child("cases")) {
    const std::string name = c.get<std::string>("name");
    const scene &sc = all.at(c.get<std::size_t>("scene"));
    coph_config cfg;
    cfg.rbf_width = width;
    cfg.policy_bounds = sc.policy_bounds;
    cfg.shaping_k = num(pot.get_child("shaping.k"));
    cfg.shaping_ell = num(pot.get_child("shaping.ell"));
    cfg.shaping_eta = num(pot.get_child("shaping.eta"));
    cfg.shaping_mode = c.get<std::string>("mode");
    coph_field_view f;
    f.centers = sc.centers;
    f.weights = sc.weights;
    f.width = width;
    f.obstacles = sc.obstacles;
    const std::optional<coph_shaping> shaping = coph_shaping_for(f, cfg);
    ASSERT_EQ(shaping.has_value(), cfg.shaping_mode == "progress") << name;

    const coph_vec2 q = vec2(c.get_child("q")), v = vec2(c.get_child("v"));
    const coph_vec2 target = vec2(c.get_child("target")), policy = vec2(c.get_child("policy"));
    const pt::ptree &out = c.get_child("out");
    EXPECT_AT_TIER(t, coph_risk<double>(q, f), num(out.get_child("risk")), name + " risk");
    EXPECT_AT_TIER(t, coph_potential<double>(q, target, policy, f, shaping),
                   num(out.get_child("potential")), name + " potential");
    const coph_vec2 grad = coph_grad_potential<double>(q, target, policy, f, shaping);
    const coph_command_of<double> cmd =
        coph_command<double>({q[0], q[1], v[0], v[1]}, target, policy, f, shaping, limit);
    const std::pair<const char *, coph_vec2> vectors[] = {
        {"grad", grad}, {"proposed", cmd.proposed}, {"applied", cmd.applied}};
    for (const auto &[key, mine] : vectors) {
      const coph_vec2 gold = vec2(out.get_child(key));
      for (int i = 0; i < 2; ++i)
        EXPECT_AT_TIER(t, mine[i], gold[i], name + " " + key + "[" + std::to_string(i) + "]");
    }
    if (shaping) {
      EXPECT_AT_TIER(t, shaping->alpha, num(out.get_child("alpha")), name + " alpha");
      EXPECT_AT_TIER(t, shaping->bound, num(out.get_child("bound")), name + " bound");
      ++progress;
    }
    ++cases;
  }
  EXPECT_EQ(cases, 50);
  EXPECT_EQ(progress, 25);
  t.report("G3");
}

// The analytic gradient against central differences of V, away from the golden's points too.
TEST(NavCophPotential, GradientMatchesFiniteDifferences) {
  const std::vector<scene> all = scenes();
  coph_config cfg;
  const double h = 1e-6;
  for (const scene &sc : all) {
    coph_field_view f;
    f.centers = sc.centers;
    f.weights = sc.weights;
    f.width = cfg.rbf_width;
    f.obstacles = sc.obstacles;
    for (const char *mode : {"legacy", "progress"}) {
      cfg.shaping_mode = mode;
      cfg.policy_bounds = sc.policy_bounds;
      const std::optional<coph_shaping> shaping = coph_shaping_for(f, cfg);
      const coph_vec2 c = sc.centers[5];
      for (const coph_vec2 &q :
           {coph_vec2{c[0] + 0.31, c[1] - 0.17}, coph_vec2{c[0] - 0.4, c[1] + 0.9}}) {
        const coph_vec2 target{c[0] + 2.0, c[1]}, policy{2.5, 1.5};
        const coph_vec2 g = coph_grad_potential<double>(q, target, policy, f, shaping);
        for (int i = 0; i < 2; ++i) {
          coph_vec2 lo = q, hi = q;
          lo[i] -= h;
          hi[i] += h;
          const double fd = (coph_potential<double>(hi, target, policy, f, shaping) -
                             coph_potential<double>(lo, target, policy, f, shaping)) /
                            (2 * h);
          EXPECT_NEAR(g[i], fd, 1e-6 * (1 + std::fabs(fd))) << sc.name << " " << mode << " " << i;
        }
      }
    }
  }
}

TEST(NavCophPotential, ShapingForFollowsTheMode) {
  const std::vector<scene> all = scenes();
  coph_field_view f;
  f.weights = all[0].weights;
  f.centers = all[0].centers;
  f.obstacles = all[0].obstacles;
  coph_config cfg;
  EXPECT_FALSE(coph_shaping_for(f, cfg).has_value()); // the default, 'legacy'
  cfg.shaping_mode = "progress";
  ASSERT_TRUE(coph_shaping_for(f, cfg).has_value());
  // No weights and no obstacles: bound 0, alpha 1.
  coph_field_view empty;
  const coph_shaping s = coph_shaping_parameters(empty, cfg);
  EXPECT_EQ(s.bound, 0.0);
  EXPECT_EQ(s.alpha, 1.0);
}

// ---------------------------------------------------------------------------
// G11: the allocation
// ---------------------------------------------------------------------------

TEST(NavCophAllocation, ReproducesG11) {
  const pt::ptree &alloc = golden().get_child("allocation");
  const tier t(alloc);
  int cases = 0;
  for (const auto &[_, c] : alloc.get_child("cases")) {
    const std::string name = c.get<std::string>("name");
    const coph_allocation a = coph_allocate(
        vec2(c.get_child("u")), num(c.get_child("heading")), num(c.get_child("speed")),
        num(c.get_child("wheelbase")), num(c.get_child("steer_max")), num(c.get_child("accel_max")),
        num(c.get_child("speed_floor")));
    const pt::ptree &out = c.get_child("out");
    EXPECT_AT_TIER(t, a.accel, num(out.get_child("accel")), name + " accel");
    EXPECT_AT_TIER(t, a.steer, num(out.get_child("steer")), name + " steer");
    EXPECT_AT_TIER(t, a.kappa, num(out.get_child("kappa")), name + " kappa");
    const std::pair<const char *, coph_vec2> vectors[] = {{"used", a.used},
                                                          {"unallocated", a.unallocated}};
    for (const auto &[key, mine] : vectors) {
      const coph_vec2 gold = vec2(out.get_child(key));
      for (int i = 0; i < 2; ++i)
        EXPECT_AT_TIER(t, mine[i], gold[i], name + " " + key + "[" + std::to_string(i) + "]");
    }
    ++cases;
  }
  EXPECT_EQ(cases, 60);
  t.report("G11");
}

TEST(NavCophAllocation, AtRestTheLateralForceIsUnallocated) {
  const double heading = 0.7;
  const coph_vec2 n{-std::sin(heading), std::cos(heading)};
  const coph_allocation a = coph_allocate(n, heading, 0.0, 0.33, 0.4189, 3.0);
  EXPECT_DOUBLE_EQ(a.kappa, 1.0 / (coph_speed_floor * coph_speed_floor));
  EXPECT_EQ(a.steer, 0.4189); // turned to the lock, toward the force
  EXPECT_LT(std::hypot(a.used[0], a.used[1]), 1e-12);
  EXPECT_NEAR(a.unallocated[0] * n[0] + a.unallocated[1] * n[1], 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// The energy account
// ---------------------------------------------------------------------------

// 2,000 steps of command and flow on the crossing scene, at the profile's damping and step: the
// summed residual stays at round-off next to the work and the heat, and the balance closes.
TEST(NavCophEnergy, ClosesOverATwoThousandStepRun) {
  const std::vector<scene> all = scenes();
  const scene &sc = all[0];
  const coph_config cfg = coph_profile();
  coph_field_view f;
  f.centers = sc.centers;
  f.weights = sc.weights;
  f.width = cfg.rbf_width;
  f.obstacles = sc.obstacles;
  const std::optional<coph_shaping> shaping = coph_shaping_for(f, cfg);
  ASSERT_TRUE(shaping.has_value()); // the profile shapes by progress
  coph_state x{sc.centers[0][0], sc.centers[0][1], 0.0, 0.0};
  const coph_vec2 target{sc.centers[15][0], sc.centers[15][1]};
  const coph_vec2 policy{3.5, 0.5};
  const double d = cfg.damping_initial, dt = cfg.dt;
  coph_energy e;
  double moved = 0.0;
  for (int i = 0; i < 2000; ++i) {
    const coph_command_of<double> c = coph_command<double>(x, target, policy, f, shaping, 3.0);
    e.add(coph_balance(x, c.applied, d, dt));
    // The allocation's e for a car heading along +x: what it would not take of u.
    const coph_allocation a =
        coph_allocate(c.applied, 0.0, std::hypot(x[2], x[3]), 0.33, 0.4189, 3.0);
    const coph_state next = coph_flow<double>(x, c.applied, d, dt);
    e.add_unallocated(a.unallocated, {x[0], x[1]}, {next[0], next[1]});
    moved += std::hypot(next[0] - x[0], next[1] - x[1]);
    x = next;
  }
  EXPECT_EQ(e.steps, 2000);
  std::printf("2,000 steps: work %.6g, heat %.6g, kinetic change %.6g, summed residual %.3g, "
              "moved %.4g m\n",
              e.work, e.heat, e.kinetic_change, e.residual, moved);
  EXPECT_GT(moved, 1.0);
  const double scale = std::fabs(e.work) + std::fabs(e.heat) + std::fabs(e.kinetic_change);
  EXPECT_GT(scale, 1.0);
  EXPECT_LT(std::fabs(e.residual), 1e-12 * scale); // 2e-16 against 7.6 on Windows
  EXPECT_NEAR(e.kinetic_change, e.work - e.heat, 1e-12 * scale);
  EXPECT_TRUE(std::isfinite(e.unallocated_work));
  e.add_shaping_jump(0.25);
  e.add_shaping_jump(-0.5);
  EXPECT_EQ(e.shaping_jumps, -0.25);
}

} // namespace
