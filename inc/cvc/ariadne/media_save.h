#ifndef CVC_ARIADNE_MEDIA_SAVE_H
#define CVC_ARIADNE_MEDIA_SAVE_H

// Ariadne — saving a picture or a video the user sees (docs/ARIADNE.md §5.29).
//
// An image: whose picture came from a URL or a file offers "Save image..." in its context menu, and
// a video: (with controls) "Save video...". (save-url URL [OPTS]) does the same for a document's
// own Save button. The core fetches the bytes as the widget did (the host's network, its allowlist
// and the widget's `auth:` credential, through the app's HTTP cache where the widget's fetch went
// through it), names the file from the URL with the extension of what the bytes are, and then,
// in this order:
//   - the backend's own Save dialog, where it has one (Backend::ask_save_path): arigl's is the
//     system's, through SDL; the core writes the file where the user chose;
//   - in a browser build, a download of the bytes from the page (a Blob and an <a download>);
//   - anywhere else (ari's FTXUI and tvision backends, a GL window without SDL), a new file in the
//     user's Downloads folder, never over one that is there, and a notification naming its path.

#include <functional>
#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {

// What to save: a picture or a video a context menu offers, or what (save-url) names.
struct MediaSave {
  std::string url;    // its source, as the widget names it (http(s), file, cvc, state)
  std::string auth;   // the credential NAME the widget fetches it with (never a token); "" none
  std::string name;   // the file name to give it; "" = from the URL
  std::string mime;   // its MIME type; "" = what the bytes are
  bool video = false; // a video's ("Save video..."), not a picture's
};

// The name a browser page stages a save's bytes under (browser_save_stage): its URL and credential
// name, which a context menu and the core agree on.
inline std::string media_save_key(const MediaSave &m) { return m.url + '\n' + m.auth; }

// What a host's Save dialog asks with (Backend::ask_save_path).
struct SavePathRequest {
  std::string name;  // the file name it suggests, with its extension
  std::string mime;  // the file's MIME type (a dialog's filter)
  std::string title; // "Save image", "Save video" or "Save file"
};
// The dialog's answer: the path the user chose (UTF-8), "" when they cancelled; or `error`, why it
// could not ask.
using SavePathDone = std::function<void(std::string path, std::string error)>;

// What the bytes are, from their first bytes: a picture's, a video's or a sound's MIME type
// ("image/png", "video/mp4", ...), or "" when they are none of those this knows.
std::string sniff_mime(std::string_view bytes);

// The name to save a file as: `name` when given, else the last part of the URL's path
// (percent-decoded, without its query), else "image" or "video". Only its last path part is kept,
// characters a file name can't hold are replaced with '_', and a name Windows reserves (CON, NUL,
// COM1...) gets a '_' in front. With a `mime` that has an extension (mime_extension), the name ends
// in it: an extension of another known type is replaced ("a.jpg" holding a PNG is "a.png"), and
// one that isn't a known type is kept and the type's added ("a.php" -> "a.php.png").
std::string save_file_name(const std::string &url, const std::string &name, const std::string &mime,
                           bool video);

// The user's Downloads folder (UTF-8), made when it isn't there: on Windows the Downloads known
// folder; elsewhere $XDG_DOWNLOAD_DIR, else XDG_DOWNLOAD_DIR in user-dirs.dirs (in
// $XDG_CONFIG_HOME, or ~/.config), else ~/Downloads. "" with `error` when there is no home folder
// to find it in.
std::string downloads_dir(std::string *error = nullptr);

// Write `bytes` to a new file named `name` in `dir`, never over a file there: when `name` is taken,
// "stem (1).ext", "stem (2).ext" and so on. The path written (UTF-8), or "" with `error`.
std::string save_new_file(const std::string &dir, const std::string &name, std::string_view bytes,
                          std::string *error = nullptr);

// Write `bytes` to `path`, replacing what is there (a Save dialog asked the user). False with
// `error` when it could not; a file left half written is removed.
bool save_file_at(const std::string &path, std::string_view bytes, std::string *error = nullptr);

// ---- a browser page -----------------------------------------------------------------------------
//
// In a browser build the page downloads a file: a Blob of the bytes, and a click on an
// <a download> for it. A browser counts a download as the user's inside a click; Safari, only
// inside the click itself. So a context menu stages the bytes when it opens (KEY names them), and
// the page's own pointer release over its Save item (armed) clicks the link there, before the
// program sees the click a frame later. A save the page hasn't staged clicks when its bytes come.

// Hand `bytes` to the page as the download `name` (of type `mime`) staged as KEY, replacing the one
// staged before. When the pointer released over the Save item armed for KEY before the bytes came,
// it is clicked now. False where there is no page (a native build, node).
bool browser_save_stage(const std::string &key, const std::string &name, const std::string &mime,
                        std::string_view bytes);
// Download what KEY staged, unless the pointer release over its item has already: true when it is
// downloaded (now, or by that release). False when nothing is staged as KEY.
bool browser_save_commit(const std::string &key);
// Let what KEY staged go, if it is still what is staged.
void browser_save_drop(const std::string &key);
// The Save item under the pointer this frame (null: none): a pointer release over it downloads
// what KEY staged, inside the click.
void browser_save_arm(const std::string *key);
// The page downloads `bytes` as `name` now (stage and commit): false where there is no page.
bool browser_save_file(const std::string &name, const std::string &mime, std::string_view bytes);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_MEDIA_SAVE_H
