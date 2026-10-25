// arigl's gallery: a Demos menu over a catalog of demos, put in every document arigl runs (as an
// AppRuntime host menu). A demo that is a .ari document loads in place, in this same window; any
// other demo opens on its own -- its page in the browser, its program natively. The catalog is
// the JSON file named by $CVC_ARIGL_GALLERY:
//
//   {"demos": [
//     {"slug": "fem_gallery_ari", "title": "fem_gallery", "desc": "...",
//      "doc": "documents/fem_gallery.ari"},
//     {"slug": "terrain_lab", "desc": "...", "href": "../terrain_lab/", "program": "terrain_lab"}
//   ]}
//
// `doc` is relative to the catalog's directory. `href` is where the browser goes; `program` is a
// program natively started from arigl's own directory. An entry this build can't open is left out.
#pragma once

#include <cvc/ariadne/widget.h>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
class AppRuntime;
}
} // namespace cvc

namespace arigl {

class gallery {
public:
  // Read the catalog. Empty when `path` is empty; on a bad catalog, empty with *error set.
  static gallery load(const std::string &path, std::string *error);
  bool empty() const { return entries_.empty(); }

  // The Demos menu, with the entry whose document is `current` greyed out. Its items raise
  // "gallery:<n>", which install() handles.
  cvc::ariadne::Widget menu(const std::string &current) const;

  // Handle the menu's events on `rt`: a document is left in *pending for the host to load between
  // frames; a page is opened in the browser; a program is started natively, and *quit set.
  void install(cvc::ariadne::AppRuntime &rt, std::string *pending, bool *quit) const;

  // In the browser, name the document now showing in the page's URL (?doc=<name>), so a reload or a
  // shared link opens it again. Nothing natively.
  static void show_in_url(const std::string &doc);

private:
  struct entry {
    std::string slug, title, desc;
    std::string doc;                  // an absolute .ari path, loaded in place
    std::string href;                 // the browser: the page to go to
    std::vector<std::string> program; // natively: the program to start
  };
  std::vector<entry> entries_;
};

} // namespace arigl
