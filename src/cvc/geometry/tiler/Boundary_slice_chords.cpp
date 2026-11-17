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

#include "Boundary_slice_chords.h"
#include "print_utils.h"

namespace cvc::tiler {

void Boundary_slice_chords::put(const Boundary_slice_chord& chord)
{
  if (!contains(chord))
  {
    TILER_TRACE(_logger, "Adding (" << chord.segment().source().id() << "--" 
		    << chord.segment().target().id() << " - " << chord.seg_z() << ")");
    _chords[chord] = true;
  }
  else
  {
    TILER_TRACE(_logger, "Removing (" << chord.segment().source().id() << "--" 
		    << chord.segment().target().id() << " - " << chord.seg_z() << ")");
    erase(chord);
  }
  dump();
}

void Boundary_slice_chords::erase(const Boundary_slice_chord& chord)
{
  _chords.erase(chord);
  _chords.erase(chord.opposite());

  _retired[chord.segment()] = true;
}

bool Boundary_slice_chords::retired(const Segment_3& chord)
{
  return (_retired.find(chord) != _retired.end() || _retired.find(chord.opposite()) != _retired.end());
}

size_t Boundary_slice_chords::size() const
{ return _chords.size(); }

bool Boundary_slice_chords::contains(const Boundary_slice_chord& chord)
{
  return (_chords.find(chord) != _chords.end() || _chords.find(chord.opposite()) != _chords.end());
}

void Boundary_slice_chords::dump() const
{
  stringstream ss;
  ss << "Boundary slice chords: [";
  for (Container::const_iterator it = _chords.begin(); it != _chords.end(); ++it)
  {
    if (it->second)
      ss << " (" << it->first.segment().source().id() << "--" << it->first.segment().target().id() << ")";
  }
  ss << " ]";
  TILER_TRACE(_logger, ss.str());
}

} // namespace cvc::tiler
