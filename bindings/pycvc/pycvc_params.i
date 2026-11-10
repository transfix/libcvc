// pycvc_params.i — params structs from their cvc::param_table (inc/cvc/core/param_table.h,
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.5). A port %includes its header, so
// the struct's fields are attributes, and then gives the table to Python once:
//
//   %include "cvc/volume/segmentation.h"
//   PYCVC_PARAM_TABLE(seeded_segmentation_params, cvc::seeded_segmentation_params)
//
// which adds, generated from the table:
//   pycvc.seeded_segmentation_params(t_low=40, t_high=200)
//                              keywords over the defaults; an unknown keyword raises TypeError
//                              naming the valid ones (a keyword is the field's name with '.' as
//                              '_', which is its member's name)
//   P.fields()                 the keywords, in table order
//   P.from_dict(d), p.to_dict()
//   p.validate()               a list of what is wrong (out of range, no such choice, the
//                              table's own check); [] when nothing is
//   p.to_state(node), P.from_state(node)
//                              one key per field under a pycvc.State; from_state seeds missing
//                              keys with their defaults and raises ValueError naming every key
//                              that does not parse or is out of range
//   p.set_text(name, text), p.get_text(name)
//                              a field as text (param_table.h's forms; ValueError if it does not
//                              parse)
//   P.markdown()               the table as the Markdown a port's docs include

%{
#include <cvc/core/param_table.h>
#include <stdexcept>
%}

%pythoncode %{
def _pycvc_install_params(cls):
    """Give a params class from PYCVC_PARAM_TABLE keyword construction, from_dict and to_dict."""
    init = cls.__init__

    def __init__(self, *args, **kw):
        init(self, *args)
        if kw:
            valid = list(cls.fields())
            for k, v in kw.items():
                if k not in valid:
                    raise TypeError("%s: unknown keyword '%s'; valid: %s"
                                    % (cls.__name__, k, ", ".join(valid)))
                setattr(self, k, v)

    def from_dict(c, d):
        return c(**dict(d))

    def to_dict(self):
        return {k: getattr(self, k) for k in cls.fields()}

    def __repr__(self):
        return "%s(%s)" % (cls.__name__,
                           ", ".join("%s=%r" % (k, getattr(self, k)) for k in cls.fields()))

    # Lists, not the wrapped std::vector's tuple, so `p.validate() == []` reads as it should.
    fields = cls.fields
    validate = cls.validate
    cls.fields = staticmethod(lambda: list(fields()))
    cls.validate = lambda self: list(validate(self))
    cls.__init__ = __init__
    cls.from_dict = classmethod(from_dict)
    cls.to_dict = to_dict
    cls.__repr__ = __repr__
    return cls
%}

%define PYCVC_PARAM_TABLE(PYNAME, P)
%exception P::from_state {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception P::set_text {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception P::get_text {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%extend P {
  static std::vector<std::string> fields() {
    std::vector<std::string> out;
    for (const std::string &n : cvc::param_names< P >())
      out.push_back(cvc::param_keyword_name(n));
    return out;
  }
  std::vector<std::string> validate() const { return cvc::params_validate(*$self); }
  void to_state(cvc::state &node) const { cvc::params_to_state(*$self, node); }
  static P from_state(cvc::state &node) {
    cvc::param_result< P > r = cvc::params_from_state< P >(node);
    if (!r.ok())
      throw std::invalid_argument(r.message());
    return r.value;
  }
  void set_text(const std::string &name, const std::string &text) {
    std::string why;
    if (!cvc::param_set_text(*$self, name, text, why))
      throw std::invalid_argument(why);
  }
  std::string get_text(const std::string &name) const {
    std::string out;
    if (!cvc::param_get_text(*$self, name, out))
      throw std::invalid_argument("unknown parameter '" + name + "'");
    return out;
  }
  static std::string markdown() { return cvc::params_markdown< P >(); }
}
%pythoncode %{
_pycvc_install_params(PYNAME)
%}
%enddef
