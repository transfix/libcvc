// The Runtime's own "Open file" window (file_picker.h, docs/ARIADNE.md §10.8).

#include "file_picker.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/backend.h>
#include <system_error>
#include <utility>

namespace cvc {
namespace ariadne {

namespace {

namespace fs = std::filesystem;

// A folder lists at most this many entries; the rest are counted, and a name typed reaches them.
constexpr std::size_t kMaxEntries = 500;

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string human_size(std::uint64_t n) {
  char buf[32];
  if (n < 1024)
    std::snprintf(buf, sizeof buf, "%llu B", static_cast<unsigned long long>(n));
  else if (n < 1024ull * 1024)
    std::snprintf(buf, sizeof buf, "%.1f KB", n / 1024.0);
  else if (n < 1024ull * 1024 * 1024)
    std::snprintf(buf, sizeof buf, "%.1f MB", n / (1024.0 * 1024.0));
  else
    std::snprintf(buf, sizeof buf, "%.1f GB", n / (1024.0 * 1024.0 * 1024.0));
  return buf;
}

// `text` as a path: "~" and "~/..." are the home folder; a relative one is under `dir`.
fs::path resolve(const fs::path &dir, std::string text) {
  if (!text.empty() && text[0] == '~' && (text.size() == 1 || text[1] == '/' || text[1] == '\\')) {
#ifdef _WIN32
    const char *home = std::getenv("USERPROFILE");
#else
    const char *home = std::getenv("HOME");
#endif
    if (home && *home)
      text = std::string(home) + text.substr(1);
  }
  fs::path p = utf8_path(text);
  if (p.is_relative())
    p = dir / p;
  return p.lexically_normal();
}

} // namespace

void ComposedFilePicker::open(const FilePickRequest &req, FilePickDone done) {
  if (active_)
    finish({});
  req_ = req;
  done_ = std::move(done);
  active_ = true;
  fresh_ = true;
  window_id_ = "ariadne.file_pick." + std::to_string(++openings_);
  field_.clear();
  error_.clear();
  selected_ = -1;
  std::error_code ec;
  dir_ = req.dir.empty() ? fs::current_path(ec) : fs::absolute(utf8_path(req.dir), ec);
  if (ec || !fs::is_directory(dir_, ec))
    dir_ = fs::current_path(ec);
  list();
}

void ComposedFilePicker::close() {
  active_ = false;
  done_ = nullptr;
  entries_.clear();
}

void ComposedFilePicker::finish(std::vector<std::shared_ptr<FileSource>> picked) {
  FilePickDone done = std::move(done_);
  close();
  if (done)
    done(std::move(picked), std::string());
}

void ComposedFilePicker::list() {
  entries_.clear();
  hidden_ = 0;
  std::vector<Entry> dirs, files;
  std::error_code ec;
  for (fs::directory_iterator it(dir_, fs::directory_options::skip_permission_denied, ec), end;
       !ec && it != end; it.increment(ec)) {
    const std::string name = path_utf8(it->path().filename());
    if (name.empty() || name[0] == '.')
      continue; // hidden files: a name typed still reaches them
    std::error_code e2;
    if (it->is_directory(e2)) {
      dirs.push_back({name, true, 0});
    } else if (it->is_regular_file(e2) && file_accepted(req_.accept, name, guess_mime_type(name))) {
      const std::uintmax_t size = it->file_size(e2);
      files.push_back({name, false, e2 ? 0 : static_cast<std::uint64_t>(size)});
    }
  }
  if (ec)
    error_ = path_utf8(dir_) + ": " + ec.message();
  const auto by_name = [](const Entry &a, const Entry &b) { return lower(a.name) < lower(b.name); };
  std::sort(dirs.begin(), dirs.end(), by_name);
  std::sort(files.begin(), files.end(), by_name);
  if (dir_.has_parent_path() && dir_.parent_path() != dir_)
    entries_.push_back({"..", true, 0});
  for (std::vector<Entry> *group : {&dirs, &files})
    for (Entry &e : *group) {
      if (entries_.size() >= kMaxEntries) {
        ++hidden_;
        continue;
      }
      entries_.push_back(std::move(e));
    }
  // The list shows one entry chosen (a radio list always does): the first file, else the first
  // folder, else "..", which Open takes while the name field is empty.
  selected_ = entries_.empty() ? -1 : 0;
  for (std::size_t i = 0; i < entries_.size(); ++i)
    if (!entries_[i].dir) {
      selected_ = static_cast<int>(i);
      break;
    }
  if (selected_ == 0 && entries_.size() > 1 && entries_[0].name == "..")
    selected_ = 1;
}

void ComposedFilePicker::go(const std::string &text) {
  error_.clear();
  if (text.empty()) {
    error_ = "Choose a file, or type its name";
    return;
  }
  const fs::path p = resolve(dir_, text);
  std::error_code ec;
  if (fs::is_directory(p, ec)) {
    dir_ = p;
    field_.clear();
    list();
    return;
  }
  if (!fs::exists(p, ec)) {
    error_ = "No such file or folder: " + text;
    return;
  }
  std::string why;
  std::shared_ptr<FileSource> f = local_file_source(path_utf8(p), &why);
  if (!f) {
    error_ = why;
    return;
  }
  if (!file_accepted(req_.accept, f->name(), f->type())) {
    std::string kinds;
    for (const std::string &a : req_.accept)
      kinds += (kinds.empty() ? "" : ", ") + a;
    error_ = f->name() + " is not one of: " + kinds;
    return;
  }
  finish({std::move(f)});
}

void ComposedFilePicker::emit(Backend &b) {
  if (!active_)
    return;
  const std::string title = req_.title.empty() ? std::string("Open file") : req_.title;
  Size size;
  size.w = {Unit::Percent, 90.0f};
  size.h = {Unit::Percent, 80.0f};
  bool open = true;
  if (fresh_)
    b.focus_next_window(); // over the document's windows, and the one the keys go to
  b.push_id(window_id_.c_str());
  const bool shown = b.begin_window(title.c_str(), window_id_.c_str(), size, -1.0f, &open);
  // What the user did is acted on after the window is drawn, so nothing it draws changes midway.
  enum class Act { none, go, pick_entry, cancel } act = Act::none;
  std::string go_text;
  if (shown) {
    b.text_value("Folder", path_utf8(dir_));
    if (fresh_)
      b.focus_next_widget(); // the name field: type a name or a path, then Enter
    MessageInputOptions o;
    o.hint = "a name or a path, then Enter";
    o.multiline = false;
    o.max_lines = 1;
    const MessageEdit e = b.message_input("Name", "ariadne.file_pick.name", field_, o);
    if (e.submitted) {
      act = Act::go;
      go_text = e.value;
    } else if (e.changed || e.committed) {
      field_ = e.value;
    }
    if (!error_.empty()) {
      const float red[3] = {0.9f, 0.35f, 0.3f};
      b.text_line_colored(error_.c_str(), red);
    }
    b.separator();
    std::vector<std::string> labels;
    labels.reserve(entries_.size());
    for (const Entry &en : entries_)
      labels.push_back(en.dir ? en.name + "/" : en.name + "  (" + human_size(en.size) + ")");
    if (labels.empty())
      b.text_line(req_.accept.empty() ? "(an empty folder)" : "(nothing here it takes)");
    else {
      const IndexEdit pick = b.radio("##entries", selected_, labels, false);
      if (pick.changed && pick.index >= 0 && pick.index < static_cast<int>(entries_.size())) {
        selected_ = pick.index;
        field_ = entries_[static_cast<std::size_t>(selected_)].name;
      }
    }
    if (hidden_ > 0)
      b.text_line(("(" + std::to_string(hidden_) + " more not listed: type a name)").c_str());
    b.separator();
    Layout row;
    row.kind = LayoutKind::Horizontal;
    row.col_widths = {Track{}, Track{}};
    if (b.begin_grid(row, "ariadne.file_pick.buttons")) {
      b.grid_next_cell();
      if (b.button("Open") && act == Act::none)
        act = Act::pick_entry;
      b.grid_next_cell();
      if (b.button("Cancel"))
        act = Act::cancel;
      b.end_grid();
    } else {
      if (b.button("Open") && act == Act::none)
        act = Act::pick_entry;
      if (b.button("Cancel"))
        act = Act::cancel;
    }
  }
  b.end_window();
  b.pop_id();
  fresh_ = false;
  if (!open || act == Act::cancel) {
    finish({});
    return;
  }
  if (act == Act::go)
    go(go_text);
  else if (act == Act::pick_entry)
    go(!field_.empty() || selected_ < 0 || selected_ >= static_cast<int>(entries_.size())
           ? field_
           : entries_[static_cast<std::size_t>(selected_)].name);
}

} // namespace ariadne
} // namespace cvc
