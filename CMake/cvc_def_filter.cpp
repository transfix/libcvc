/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// cvc_def_filter <in.def> <out.def>
//
// Copies the module-definition file that `cmake -E __create_def` wrote for a
// DLL's objects, leaving out every symbol defined in namespace std. A pre-link
// step of cvc.dll; CvcMsvcExportDef.cmake says why.
//
// A symbol is in namespace std when its undecorated name is: a member of a std
// class template (std::vector<cvc::shape>::_Emplace_reallocate), a std function
// template or inline function (std::_Destroy_range<...>, std::operator==), a std
// type's RTTI descriptor, or a local of a std function (`std::...'::`2'::_Static).
// dbghelp undecorates; a name it cannot undecorate is kept.

#include <cstdio>
#include <fstream>
#include <string>
#include <string_view>

// clang-format off: dbghelp.h needs windows.h first.
#include <windows.h>
#include <dbghelp.h>
// clang-format on

namespace {

// The PE export table's name count is 16 bits.
constexpr std::size_t max_exports = 65535;

bool in_std(const std::string &symbol) {
  if (symbol.empty() || symbol[0] != '?') // extern "C": no namespace
    return false;
  // Only the start of the name matters, so a long one may be cut short.
  char name[1024];
  if (!UnDecorateSymbolName(symbol.c_str(), name, sizeof name, UNDNAME_NAME_ONLY))
    return false;
  std::string_view n(name);
  // A function's local scope opens with a backtick, one per level of nesting.
  while (!n.empty() && n.front() == '`')
    n.remove_prefix(1);
  return n.substr(0, 5) == "std::";
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: cvc_def_filter <in.def> <out.def>\n");
    return 2;
  }
  std::ifstream in(argv[1]);
  if (!in) {
    std::fprintf(stderr, "cvc_def_filter: cannot read %s\n", argv[1]);
    return 1;
  }

  // "EXPORTS", then one symbol per line, a data symbol followed by DATA.
  std::string out, line;
  std::size_t kept = 0, dropped = 0;
  while (std::getline(in, line)) {
    const std::size_t b = line.find_first_not_of(" \t\r");
    if (b == std::string::npos)
      continue;
    const std::string symbol = line.substr(b, line.find_first_of(" \t\r", b) - b);
    if (symbol != "EXPORTS") {
      if (in_std(symbol)) {
        ++dropped;
        continue;
      }
      ++kept;
    }
    out += line;
    out += '\n';
  }

  std::ofstream os(argv[2], std::ios::binary | std::ios::trunc);
  os << out;
  if (!os.flush()) {
    std::fprintf(stderr, "cvc_def_filter: cannot write %s\n", argv[2]);
    return 1;
  }
  std::printf("cvc_def_filter: %zu exports, %zu std:: symbols left out\n", kept, dropped);
  if (kept > max_exports) {
    std::fprintf(stderr, "cvc_def_filter: %zu exports is over the PE limit of %zu names\n", kept,
                 max_exports);
    return 1;
  }
  return 0;
}
