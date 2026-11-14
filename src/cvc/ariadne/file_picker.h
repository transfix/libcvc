#ifndef CVC_ARIADNE_FILE_PICKER_H
#define CVC_ARIADNE_FILE_PICKER_H

// The "Open file" window the Runtime draws for (file-pick) where the host has no picker of its own
// (docs/ARIADNE.md §10.8): ari's FTXUI and tvision backends, a GL window without SDL, a test's
// mock. It is made of the backend's own widgets (a window, a message field, a radio list, buttons),
// so it takes the keys and the pointer as the document's widgets do there, and tvision shows it as
// one of its windows. Internal to the Runtime (ariadne.cpp); not installed.

#include <cstdint>
#include <cvc/ariadne/files.h>
#include <filesystem>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

class Backend;

class ComposedFilePicker {
public:
  // Show it for `req`. `done` gets one file, or none when the user cancels (the window's close
  // button, Cancel). A picker shown already is answered as cancelled first.
  void open(const FilePickRequest &req, FilePickDone done);
  bool active() const { return active_; }
  // Close it without answering (the program that asked is gone).
  void close();
  // Draw it, after the document's tree, and act on what the user did in it last frame.
  void emit(Backend &b);

  // The window's id while it is open: "ariadne.file_pick.N", N counting this picker's openings.
  // Each opening is a new window, so a backend places it afresh, over the document's windows,
  // rather than where (and as far back as) the last one was.
  const std::string &window_id() const { return window_id_; }

private:
  struct Entry {
    std::string name;
    bool dir = false;
    std::uint64_t size = 0;
  };
  void list();                      // read dir_ into entries_
  void go(const std::string &text); // Open: a folder enters it, a file picks it
  void finish(std::vector<std::shared_ptr<FileSource>> picked);

  bool active_ = false;
  unsigned openings_ = 0;
  std::string window_id_;
  bool fresh_ = false; // its first frame: raise the window and focus the name field
  FilePickRequest req_;
  FilePickDone done_;
  std::filesystem::path dir_;
  std::vector<Entry> entries_; // "..", then folders, then the files `accept` takes, by name
  std::size_t hidden_ = 0;     // entries past the cap, not listed
  std::string field_;          // the name field's text
  int selected_ = -1;          // the entry chosen in the list
  std::string error_;          // why the last Open did nothing
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_FILE_PICKER_H
