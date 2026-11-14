#ifndef CVC_ARIADNE_FILES_H
#define CVC_ARIADNE_FILES_H

// Ariadne — files the user picked (docs/ARIADNE.md §10.8).
//
// (file-pick [OPTS]) asks the user for a file and gives the program a handle to it, "file:N":
// never its bytes, never its path. So a document sends only what the user chose
// ((http-upload URL H ...), net_intrinsics.h) and reads nothing of the disk by itself. A handle
// names a FileSource the app keeps (PickedFiles, one per app), good across ticks and programs
// until (file-forget H) or the app's end.
//
// Who asks the user, in this order:
//   - the backend's own picker, where it has one (Backend::pick_files): arigl's is the system's
//     file dialog, through SDL;
//   - in a browser build, the page's <input type=file> (browser_pick_files), whatever draws the
//     document (arigl-web's canvas, ari.html's terminal);
//   - anywhere else (ari's FTXUI and tvision backends, a GL window without SDL), an "Open file"
//     window the Runtime draws with the backend's own widgets: the folder's listing, a name field,
//     Open and Cancel.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

// A file the user picked: its name, size and type, and its bytes read a slice at a time. Natively
// read() answers before it returns, and any thread may call it. In a browser it reads the page's
// File and answers later, from the browser's event loop: call it on the main thread there.
class FileSource {
public:
  virtual ~FileSource() = default;
  virtual std::string name() const = 0; // the file's own name, without its folder
  virtual std::uint64_t size() const = 0;
  virtual std::string type() const = 0; // a MIME type; "application/octet-stream" when unknown
  // Up to `length` bytes at `offset` (fewer only at the end of the file), or `error` when they
  // could not be read: the file was changed, moved or removed since it was picked. Called once.
  using ReadDone = std::function<void(std::string bytes, std::string error)>;
  virtual void read(std::uint64_t offset, std::size_t length, ReadDone done) = 0;
  // Whether read() answers after it returns (a browser's File).
  virtual bool reads_async() const { return false; }
};

// A file on disk, as a native picker gives it. Its size is taken now, and a read finds the file
// changed when its size has moved since. Null, with `error` set, when `path` is not a regular file
// this process can open.
std::shared_ptr<FileSource> local_file_source(const std::string &path,
                                              std::string *error = nullptr);
// Bytes in memory, as a file named `name` of type `type` ("" = guessed from the name): a host's
// own (a pasted picture, a dropped file) or a test's.
std::shared_ptr<FileSource> memory_file_source(std::string name, std::string bytes,
                                               std::string type = std::string());

// A path's text as UTF-8, and a path from UTF-8 text (std::filesystem's own text is not UTF-8 on
// Windows, and C++20's u8string is a std::u8string).
std::string path_utf8(const std::filesystem::path &p);
std::filesystem::path utf8_path(const std::string &utf8);

// The MIME type a file name's extension says; "application/octet-stream" when it is not one of the
// common ones (pictures, video, audio, text, PDF, archives).
std::string guess_mime_type(const std::string &name);

// What a program asked a picker for (file-pick's options).
struct FilePickRequest {
  // What may be picked: extensions (".png") and MIME types ("image/png", "image/*"), as an HTML
  // <input accept> lists them. Empty: any file.
  std::vector<std::string> accept;
  bool multiple = false; // the user may pick more than one
  std::string title;     // the picker's title; "" = "Open file"
  std::string dir;       // the folder a native or composed picker starts in; "" = the working one
};

// Whether a file named `name` of MIME type `type` is one `accept` takes; an empty `accept` takes
// any file.
bool file_accepted(const std::vector<std::string> &accept, const std::string &name,
                   const std::string &type);
// The extensions `accept` takes, lowercase, without the dot: ".PNG" as "png", "image/*" as every
// picture extension guess_mime_type knows. Empty: any (or `accept` names only types it can't map).
std::vector<std::string> accepted_extensions(const std::vector<std::string> &accept);

// What a picker answers with: the files picked (none when the user cancelled), or `error`, why it
// could not ask.
using FilePickDone =
    std::function<void(std::vector<std::shared_ptr<FileSource>> picked, std::string error)>;

// The page's <input type=file>, in a browser build: true when the browser was asked, and then
// `done` is called once, later, on the main thread, when the user picks or cancels. A browser
// opens it only within a user's gesture: an action run by a click or a key has one. False where
// there is no page to ask (a native build, or node).
bool browser_pick_files(const FilePickRequest &req, FilePickDone done);

// The files an app's programs hold handles to (app.slot<PickedFiles>(), picked_files(app)).
class PickedFiles {
public:
  static constexpr const char *app_slot = "cvc.ariadne.picked_files";
  static constexpr std::size_t kMaxFiles = 256;

  // A new handle for `file`, "file:N"; "" when the app holds kMaxFiles already.
  std::string add(std::shared_ptr<FileSource> file);
  // The file `handle` names, or null.
  std::shared_ptr<FileSource> find(const std::string &handle) const;
  // Let go of `handle`; true when it named a file.
  bool forget(const std::string &handle);
  std::size_t size() const;

private:
  mutable std::mutex m_;
  std::map<std::string, std::shared_ptr<FileSource>> files_;
  std::uint64_t next_ = 1;
};

PickedFiles &picked_files(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_FILES_H
