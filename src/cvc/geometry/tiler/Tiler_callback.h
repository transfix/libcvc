#ifndef __TILER_CALLBACK__
#define __TILER_CALLBACK__

CONTOURTILER_BEGIN_NAMESPACE

class Tiler_workspace;
class Untiled_region;

class Tiler_callback
{
public:
  Tiler_callback() {}
  virtual ~Tiler_callback() {}

  virtual bool untiled_region(const Untiled_region& r) { return true; }
  virtual bool tile_added(const Tiler_workspace& w) { return true; }
};

CONTOURTILER_END_NAMESPACE

#endif
