/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

#ifndef __BOUNDARY_SLICE_CHORDS_H__
#define __BOUNDARY_SLICE_CHORDS_H__

#include "Boundary_slice_chord.h"

namespace cvc::tiler {

/// Stores boundary slice chords such that if a chord is added
/// and it already exists, it is removed.  This way, interior
/// chords are removed as new boundaries are added.
class Boundary_slice_chords {
private:
  typedef boost::unordered_map<Boundary_slice_chord, bool> Container;

public:
  Boundary_slice_chords() {}

  ~Boundary_slice_chords() {}

  void put(const Boundary_slice_chord &chord);

  void erase(const Boundary_slice_chord &chord);

  size_t size() const;

  bool contains(const Boundary_slice_chord &chord);

  bool retired(const Segment_3 &chord);

  template <typename OutputIterator> void all(OutputIterator chords) {
    for (Container::iterator it = _chords.begin(); it != _chords.end(); ++it) {
      *chords = it->first;
      ++chords;
    }
  }

  template <typename OutputIterator> void all(OutputIterator chords) const {
    for (Container::const_iterator it = _chords.begin(); it != _chords.end(); ++it) {
      *chords = it->first;
      ++chords;
    }
  }

  const Boundary_slice_chord &first() const {
    for (Container::const_iterator it = _chords.begin(); it != _chords.end(); ++it) {
      const Segment_3 &segment = it->first.segment();
      if (!xy_equal(segment.source(), segment.target()))
        return it->first;
    }
    return _chords.begin()->first;
  }

  bool contains(const Point_3 &p, const Point_3 &q) const {
    Segment_3 s(p, q);
    Container::const_iterator it =
        _chords.find(Boundary_slice_chord(s, Walk_direction::FORWARD, 1));
    if (it != _chords.end())
      return true;

    s = Segment_3(q, p);
    it = _chords.find(Boundary_slice_chord(s, Walk_direction::FORWARD, 1));
    if (it != _chords.end())
      return true;

    return false;
  }

  const Boundary_slice_chord &get_chord(const Point_3 &p, const Point_3 &q) const {
    Segment_3 s(p, q);
    Container::const_iterator it =
        _chords.find(Boundary_slice_chord(s, Walk_direction::FORWARD, 1));
    if (it != _chords.end())
      return it->first;

    s = Segment_3(q, p);
    it = _chords.find(Boundary_slice_chord(s, Walk_direction::FORWARD, 1));
    if (it != _chords.end())
      return it->first;

    throw logic_error("Chord not found");
  }

  /// If the given vertex lies on any boundary slice chord, then
  /// all boundary slice chords containing it are returned.
  std::vector<Boundary_slice_chord> on_vertex(const Point_3 &vertex) const {
    std::vector<Boundary_slice_chord> chords;
    for (Container::const_iterator it = _chords.begin(); it != _chords.end(); ++it) {
      const Boundary_slice_chord &chord = it->first;
      if (chord.is_source(vertex))
        chords.push_back(chord);
    }
    return chords;
  }

private:
  void dump() const;

private:
  Container _chords;
  boost::unordered_map<Segment_3, bool> _retired;
};

} // namespace cvc::tiler

#endif
