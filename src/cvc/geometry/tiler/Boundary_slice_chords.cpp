#include <ContourTiler/Boundary_slice_chords.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

void Boundary_slice_chords::put(const Boundary_slice_chord& chord)
{
  if (!contains(chord))
  {
    LOG4CPLUS_TRACE(_logger, "Adding (" << chord.segment().source().id() << "--" 
		    << chord.segment().target().id() << " - " << chord.seg_z() << ")");
    _chords[chord] = true;
  }
  else
  {
    LOG4CPLUS_TRACE(_logger, "Removing (" << chord.segment().source().id() << "--" 
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
  LOG4CPLUS_TRACE(_logger, ss.str());
}

CONTOURTILER_END_NAMESPACE
