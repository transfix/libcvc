#ifndef __TILER_OUTPUT__
#define __TILER_OUTPUT__

#include <ContourTiler/Tiler_callback.h>

CONTOURTILER_BEGIN_NAMESPACE

//------------------------------------------------------------------------------
// Callback class - does intermediate output.
//------------------------------------------------------------------------------
class Callback : public Tiler_callback
{
public:
  Callback(const Tiler_options& options) 
    : _options(options), logger(log4cplus::Logger::getInstance("tiler.intermediate_callback"))
  {
  }

  virtual ~Callback() {}

  virtual bool untiled_region(const Untiled_region& r);

  virtual bool tile_added(const Tiler_workspace& workspace);

private:
  Tiler_options _options;
  log4cplus::Logger logger;
};

template <typename Contour_iter>
void output_pre_pre_tile(Contour_iter bottom_begin, Contour_iter bottom_end,
			 Contour_iter top_begin, Contour_iter top_end,
			 const Tiler_options& options)
{
  string out_dir(options.output_dir());
  string base(options.base_name());
  if (options.output_bottom())
  {
    ofstream out((out_dir + "/" + base + "_bottom.g").c_str());
    gnuplot_print_contours(out, bottom_begin, bottom_end);
    out.close();
  }

  if (options.output_top())
  {
    ofstream out((out_dir + "/" + base + "_top.g").c_str());
    gnuplot_print_contours(out, top_begin, top_end);
    out.close();
  }
}

void output_pre_tile(const Tiler_workspace& w, const Tiler_options& options);
void output_phase1(const Tiler_workspace& w, const Tiler_options& options);
void output_phase2(const Tiler_workspace& w, const Tiler_options& options);

CONTOURTILER_END_NAMESPACE

#endif
