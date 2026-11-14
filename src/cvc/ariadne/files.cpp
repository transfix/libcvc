// Ariadne — files the user picked. See files.h.

#include <algorithm>
#include <cctype>
#include <cvc/ariadne/files.h>
#include <cvc/core/app.h>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

#ifdef __EMSCRIPTEN__
#include <chrono>
#include <cstdlib>
#include <emscripten.h>
#include <emscripten/eventloop.h>
#include <thread>
#ifdef __EMSCRIPTEN_PTHREADS__
#include <emscripten/proxying.h> // a pool thread's read, run on the page's thread
#include <emscripten/threading.h>
#endif
#endif

namespace cvc {
namespace ariadne {

namespace {

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

} // namespace

std::string path_utf8(const std::filesystem::path &p) {
  const auto u8 = p.u8string();
  return std::string(u8.begin(), u8.end());
}
std::filesystem::path utf8_path(const std::string &utf8) {
  return std::filesystem::path(std::u8string(utf8.begin(), utf8.end()));
}

namespace {

// The extension of `name`, lowercase, without the dot ("" when it has none).
std::string extension(const std::string &name) {
  const std::size_t slash = name.find_last_of("/\\");
  const std::size_t dot = name.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash) ||
      dot + 1 >= name.size())
    return std::string();
  return lower(name.substr(dot + 1));
}

// The common types, by extension: what a picker filters on and an upload labels a file with.
struct MimeEntry {
  const char *ext;
  const char *type;
};
constexpr MimeEntry kMimeTypes[] = {
    {"png", "image/png"},         {"jpg", "image/jpeg"},        {"jpeg", "image/jpeg"},
    {"gif", "image/gif"},         {"webp", "image/webp"},       {"avif", "image/avif"},
    {"bmp", "image/bmp"},         {"svg", "image/svg+xml"},     {"tif", "image/tiff"},
    {"tiff", "image/tiff"},       {"heic", "image/heic"},       {"ico", "image/x-icon"},
    {"mp4", "video/mp4"},         {"m4v", "video/mp4"},         {"webm", "video/webm"},
    {"mov", "video/quicktime"},   {"mkv", "video/x-matroska"},  {"ogv", "video/ogg"},
    {"mp3", "audio/mpeg"},        {"m4a", "audio/mp4"},         {"ogg", "audio/ogg"},
    {"oga", "audio/ogg"},         {"opus", "audio/ogg"},        {"wav", "audio/wav"},
    {"flac", "audio/flac"},       {"txt", "text/plain"},        {"md", "text/markdown"},
    {"csv", "text/csv"},          {"html", "text/html"},        {"htm", "text/html"},
    {"json", "application/json"}, {"pdf", "application/pdf"},   {"zip", "application/zip"},
    {"gz", "application/gzip"},   {"tar", "application/x-tar"}, {"xml", "application/xml"},
    {"yaml", "application/yaml"}, {"yml", "application/yaml"},  {"ari", "application/yaml"},
};

constexpr const char *kOctets = "application/octet-stream";

// A file on disk: opened for each read, so nothing is held open between them.
class LocalFile : public FileSource {
public:
  LocalFile(std::filesystem::path path, std::uint64_t size)
      : path_(std::move(path)), size_(size), name_(path_utf8(path_.filename())),
        type_(guess_mime_type(name_)) {}
  std::string name() const override { return name_; }
  std::uint64_t size() const override { return size_; }
  std::string type() const override { return type_; }
  void read(std::uint64_t offset, std::size_t length, ReadDone done) override {
    std::error_code ec;
    const std::uintmax_t now = std::filesystem::file_size(path_, ec);
    if (ec) {
      done(std::string(), name_ + ": " + ec.message());
      return;
    }
    if (now != size_) {
      done(std::string(), name_ + ": the file changed since it was picked");
      return;
    }
    if (offset >= size_) {
      done(std::string(), std::string());
      return;
    }
    std::ifstream in(path_, std::ios::binary);
    if (!in) {
      done(std::string(), name_ + ": can't open it");
      return;
    }
    const std::uint64_t n = std::min<std::uint64_t>(length, size_ - offset);
    std::string bytes(static_cast<std::size_t>(n), '\0');
    in.seekg(static_cast<std::streamoff>(offset));
    in.read(bytes.data(), static_cast<std::streamsize>(n));
    if (static_cast<std::uint64_t>(in.gcount()) != n) {
      done(std::string(), name_ + ": the file changed since it was picked");
      return;
    }
    done(std::move(bytes), std::string());
  }

private:
  std::filesystem::path path_;
  std::uint64_t size_;
  std::string name_;
  std::string type_;
};

class MemoryFile : public FileSource {
public:
  MemoryFile(std::string name, std::string bytes, std::string type)
      : name_(std::move(name)), bytes_(std::move(bytes)), type_(std::move(type)) {}
  std::string name() const override { return name_; }
  std::uint64_t size() const override { return bytes_.size(); }
  std::string type() const override { return type_; }
  void read(std::uint64_t offset, std::size_t length, ReadDone done) override {
    if (offset >= bytes_.size()) {
      done(std::string(), std::string());
      return;
    }
    done(bytes_.substr(static_cast<std::size_t>(offset), length), std::string());
  }

private:
  std::string name_;
  std::string bytes_;
  std::string type_;
};

} // namespace

std::string guess_mime_type(const std::string &name) {
  const std::string ext = extension(name);
  for (const MimeEntry &e : kMimeTypes)
    if (ext == e.ext)
      return e.type;
  return kOctets;
}

std::shared_ptr<FileSource> local_file_source(const std::string &path, std::string *error) {
  std::error_code ec;
  const std::filesystem::path p = utf8_path(path);
  if (!std::filesystem::is_regular_file(p, ec)) {
    if (error)
      *error = path + ": " + (ec ? ec.message() : std::string("not a file"));
    return nullptr;
  }
  const std::uintmax_t size = std::filesystem::file_size(p, ec);
  if (ec) {
    if (error)
      *error = path + ": " + ec.message();
    return nullptr;
  }
  std::ifstream probe(p, std::ios::binary);
  if (!probe) {
    if (error)
      *error = path + ": can't open it";
    return nullptr;
  }
  return std::make_shared<LocalFile>(std::filesystem::absolute(p, ec), size);
}

std::shared_ptr<FileSource> memory_file_source(std::string name, std::string bytes,
                                               std::string type) {
  if (type.empty())
    type = guess_mime_type(name);
  return std::make_shared<MemoryFile>(std::move(name), std::move(bytes), std::move(type));
}

bool file_accepted(const std::vector<std::string> &accept, const std::string &name,
                   const std::string &type) {
  if (accept.empty())
    return true;
  const std::string ext = extension(name);
  const std::string t = lower(type);
  for (const std::string &raw : accept) {
    const std::string a = lower(raw);
    if (a.empty())
      continue;
    if (a[0] == '.') {
      if (a.substr(1) == ext)
        return true;
    } else if (a.size() > 2 && a.compare(a.size() - 2, 2, "/*") == 0) {
      if (t.compare(0, a.size() - 1, a, 0, a.size() - 1) == 0)
        return true;
    } else if (a == t) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> accepted_extensions(const std::vector<std::string> &accept) {
  std::vector<std::string> out;
  const auto add = [&](const std::string &e) {
    if (!e.empty() && std::find(out.begin(), out.end(), e) == out.end())
      out.push_back(e);
  };
  for (const std::string &raw : accept) {
    const std::string a = lower(raw);
    if (a.empty())
      continue;
    if (a[0] == '.') {
      add(a.substr(1));
      continue;
    }
    const bool wild = a.size() > 2 && a.compare(a.size() - 2, 2, "/*") == 0;
    for (const MimeEntry &e : kMimeTypes) {
      const std::string type = e.type;
      if (wild ? type.compare(0, a.size() - 1, a, 0, a.size() - 1) == 0 : type == a)
        add(e.ext);
    }
  }
  return out;
}

std::string PickedFiles::add(std::shared_ptr<FileSource> file) {
  std::lock_guard<std::mutex> lock(m_);
  if (files_.size() >= kMaxFiles)
    return std::string();
  std::string handle = "file:" + std::to_string(next_++);
  files_.emplace(handle, std::move(file));
  return handle;
}

std::shared_ptr<FileSource> PickedFiles::find(const std::string &handle) const {
  std::lock_guard<std::mutex> lock(m_);
  const auto it = files_.find(handle);
  return it == files_.end() ? nullptr : it->second;
}

bool PickedFiles::forget(const std::string &handle) {
  std::lock_guard<std::mutex> lock(m_);
  return files_.erase(handle) != 0;
}

std::size_t PickedFiles::size() const {
  std::lock_guard<std::mutex> lock(m_);
  return files_.size();
}

PickedFiles &picked_files(cvc::app &app) { return app.slot<PickedFiles>(); }

#ifdef __EMSCRIPTEN__

// The page's side: Module.cvcFiles holds the File objects picked ({files: {id: File}}) and the
// operations out ({ops: {op: {done, ...}}}), numbered by the page, so nothing on this side is kept
// between calls. An operation's result is polled from a timer on the browser's event loop
// (emscripten_set_timeout) until it is done.
// clang-format off
EM_JS(int, cvc_files_pick, (const char *accept, int multiple), {
  if (typeof document === 'undefined' || !document.body) return 0;
  var reg = Module.cvcFiles = Module.cvcFiles || {next: 1, op: 1, files: {}, ops: {}};
  var op = reg.op++;
  var st = reg.ops[op] = {done: false, ids: []};
  try {
    var input = document.createElement('input');
    input.type = 'file';
    var a = UTF8ToString(accept);
    if (a) input.accept = a;
    input.multiple = !!multiple;
    input.style.position = 'fixed';
    input.style.left = '-10000px';
    input.style.opacity = '0';
    input.setAttribute('aria-hidden', 'true');
    var finish = function() {
      if (st.done) return;
      st.done = true;
      for (var i = 0; input.files && i < input.files.length; ++i) {
        var id = reg.next++;
        reg.files[id] = input.files[i];
        st.ids.push(id);
      }
      if (input.parentNode) input.parentNode.removeChild(input);
    };
    input.addEventListener('change', finish);
    input.addEventListener('cancel', finish); // no files: the user closed the dialog
    document.body.appendChild(input);
    input.click();
  } catch (e) {
    st.done = true;
    st.error = String((e && e.message) || e);
  }
  return op;
});

EM_JS(int, cvc_files_op_done, (int op), {
  var st = Module.cvcFiles && Module.cvcFiles.ops[op];
  return !st ? -1 : st.done ? 1 : 0;
});

EM_JS(int, cvc_files_op_count, (int op), {
  var st = Module.cvcFiles.ops[op];
  return st.ids ? st.ids.length : st.n || 0;
});

EM_JS(int, cvc_files_op_id, (int op, int i), { return Module.cvcFiles.ops[op].ids[i]; });

EM_JS(char *, cvc_files_op_error, (int op), {
  var st = Module.cvcFiles.ops[op];
  var s = (st && st.error) || '';
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});

EM_JS(void, cvc_files_op_free, (int op), { if (Module.cvcFiles) delete Module.cvcFiles.ops[op]; });

EM_JS(char *, cvc_files_name, (int id), {
  var f = Module.cvcFiles.files[id];
  var s = (f && f.name) || '';
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});

EM_JS(char *, cvc_files_type, (int id), {
  var f = Module.cvcFiles.files[id];
  var s = (f && f.type) || '';
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});

EM_JS(double, cvc_files_size, (int id), {
  var f = Module.cvcFiles.files[id];
  return f ? f.size : -1;
});

EM_JS(void, cvc_files_release, (int id), { if (Module.cvcFiles) delete Module.cvcFiles.files[id]; });

// Read [offset, offset + length) of file `id` into `dst` (length bytes the caller keeps until the
// operation is done). The count read lands in the op's `n`.
EM_JS(int, cvc_files_read, (int id, double offset, int length, unsigned char *dst), {
  var reg = Module.cvcFiles;
  var op = reg.op++;
  var st = reg.ops[op] = {done: false, n: 0};
  var f = reg.files[id];
  if (!f) {
    st.done = true;
    st.error = 'the file is gone';
    return op;
  }
  f.slice(offset, offset + length).arrayBuffer().then(function(buf) {
    var bytes = new Uint8Array(buf);
    HEAPU8.set(bytes, dst);
    st.n = bytes.length;
    st.done = true;
  }, function(e) {
    st.error = String((e && e.message) || e);
    st.done = true;
  });
  return op;
});
// clang-format on

namespace {

std::string take_js_string(char *p) {
  std::string s = p ? p : "";
  std::free(p);
  return s;
}

// Whether this is the page's own thread, where its File objects are (always, without threads).
bool on_page_thread() {
#ifdef __EMSCRIPTEN_PTHREADS__
  return emscripten_is_main_runtime_thread() != 0;
#else
  return true;
#endif
}

// Run `f` on the page's thread and wait for it: in a build with threads, an upload's reads come
// from the app's background pool (net_intrinsics.cpp's kernel), and the File is the page's.
template <class F> void at_page(F &&f) {
#ifdef __EMSCRIPTEN_PTHREADS__
  if (!on_page_thread()) {
    struct Call {
      F *f;
    } call{&f};
    emscripten_proxy_sync(
        emscripten_proxy_get_system_queue(), emscripten_main_runtime_thread_id(),
        [](void *p) { (*static_cast<Call *>(p)->f)(); }, &call);
    return;
  }
#endif
  f();
}

// A file the page picked: its File stays on the page, by id, until this goes.
class BrowserFile : public FileSource {
public:
  explicit BrowserFile(int id)
      : id_(id), name_(take_js_string(cvc_files_name(id))),
        size_(static_cast<std::uint64_t>(std::max(0.0, cvc_files_size(id)))) {
    type_ = take_js_string(cvc_files_type(id));
    if (type_.empty())
      type_ = guess_mime_type(name_);
  }
  ~BrowserFile() override {
    const int id = id_;
    at_page([id] { cvc_files_release(id); });
  }
  std::string name() const override { return name_; }
  std::uint64_t size() const override { return size_; }
  std::string type() const override { return type_; }
  // From the page's thread, a read answers later, from its event loop; from a pool thread (a
  // build with threads) it waits for the page and answers before it returns.
  bool reads_async() const override { return true; }

  void read(std::uint64_t offset, std::size_t length, ReadDone done) override {
    if (offset >= size_) {
      done(std::string(), std::string());
      return;
    }
    const std::size_t n = static_cast<std::size_t>(std::min<std::uint64_t>(length, size_ - offset));
    if (!on_page_thread()) {
      // A pool thread (a build with threads): the page reads, this thread waits for it.
      std::string bytes(n, '\0');
      int op = 0;
      at_page([&] {
        op = cvc_files_read(id_, static_cast<double>(offset), static_cast<int>(n),
                            reinterpret_cast<unsigned char *>(bytes.data()));
      });
      int state = 0;
      while (state == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        at_page([&] { state = cvc_files_op_done(op); });
      }
      std::string error;
      int got = 0;
      at_page([&] {
        error = take_js_string(cvc_files_op_error(op));
        got = error.empty() ? cvc_files_op_count(op) : 0;
        cvc_files_op_free(op);
      });
      if (!error.empty()) {
        done(std::string(), "the browser could not read the file: " + error);
        return;
      }
      bytes.resize(static_cast<std::size_t>(std::max(0, got)));
      done(std::move(bytes), std::string());
      return;
    }
    auto *r = new Read;
    r->bytes.resize(n);
    r->done = std::move(done);
    r->op = cvc_files_read(id_, static_cast<double>(offset), static_cast<int>(r->bytes.size()),
                           reinterpret_cast<unsigned char *>(r->bytes.data()));
    poll(r);
  }

private:
  // A read out: its buffer lives until the page has written it (or failed), whatever the reader
  // does meanwhile.
  struct Read {
    int op = 0;
    std::string bytes;
    ReadDone done;
  };
  static void poll(void *p) {
    auto *r = static_cast<Read *>(p);
    if (cvc_files_op_done(r->op) == 0) {
      emscripten_set_timeout(&BrowserFile::poll, 4, r);
      return;
    }
    std::string error = take_js_string(cvc_files_op_error(r->op));
    const int n = error.empty() ? cvc_files_op_count(r->op) : 0;
    cvc_files_op_free(r->op);
    std::unique_ptr<Read> owned(r);
    owned->bytes.resize(static_cast<std::size_t>(std::max(0, n)));
    if (error.empty())
      owned->done(std::move(owned->bytes), std::string());
    else
      owned->done(std::string(), "the browser could not read the file: " + error);
  }

  int id_;
  std::string name_;
  std::uint64_t size_;
  std::string type_;
};

struct PendingPick {
  int op = 0;
  FilePickDone done;
};

void poll_pick(void *p) {
  auto *pick = static_cast<PendingPick *>(p);
  if (cvc_files_op_done(pick->op) == 0) {
    emscripten_set_timeout(&poll_pick, 50, pick);
    return;
  }
  std::unique_ptr<PendingPick> owned(pick);
  std::string error = take_js_string(cvc_files_op_error(pick->op));
  std::vector<std::shared_ptr<FileSource>> files;
  const int n = error.empty() ? cvc_files_op_count(pick->op) : 0;
  for (int i = 0; i < n; ++i)
    files.push_back(std::make_shared<BrowserFile>(cvc_files_op_id(pick->op, i)));
  cvc_files_op_free(pick->op);
  owned->done(std::move(files), error.empty() ? error : "the browser's file picker: " + error);
}

} // namespace

bool browser_pick_files(const FilePickRequest &req, FilePickDone done) {
  std::string accept;
  for (const std::string &a : req.accept)
    accept += (accept.empty() ? "" : ",") + a;
  const int op = cvc_files_pick(accept.c_str(), req.multiple ? 1 : 0);
  if (op == 0)
    return false;
  auto *pick = new PendingPick{op, std::move(done)};
  emscripten_set_timeout(&poll_pick, 50, pick);
  return true;
}

#else

bool browser_pick_files(const FilePickRequest &, FilePickDone) { return false; }

#endif

} // namespace ariadne
} // namespace cvc
