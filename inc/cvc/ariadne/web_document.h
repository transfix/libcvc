// Ariadne — a runtime's document named by URL, in a browser. The browser pages of arigl
// (arigl-web, src/arigl/web/arigl-web.html) and ari (src/ari/web/ari.html) pass the document's
// URL where a path would go: `arigl run [flags] https://<origin>/<path>/main.ari`. Before the load
// the runtime calls stage_url_document, which fetches the document and the files it names
// (doc_fetch.h) through the page's fetch() into MEMFS at their URL paths, changes to the
// document's directory and points o.doc at the local copy, so its relative imports resolve as
// they would on disk. Only the page's own origin is allowed unless the page lists others.
//
// Emscripten only: it awaits each fetch under Asyncify, so the runtime links with -sASYNCIFY.
// Natively a URL is refused; a runtime runs a document from a path there.
#pragma once

#include <cvc/ariadne/host_cli.h>
#include <string>

namespace cvc {
namespace ariadne {

// Whether `doc` names a document by an http:// or https:// URL.
bool is_url_document(const std::string &doc);

// Fetch the URL document in o.doc and point o.doc at its local copy. Returns -1 to carry on, else
// the exit status (1: it could not be loaded, and stderr says why, each line prefixed with
// `runtime`). Natively a URL is refused with status 2.
//
// The page passes its config through the environment:
//   CVC_ARIADNE_SRC_ORIGINS  other origins the document may come from, space-separated ("*": any)
//   CVC_ARIADNE_PRELOAD      more files to fetch with it, relative to it, one per line
int stage_url_document(HostOptions &o, const std::string &runtime);

// Tell the page the document is loaded and the runtime is about to run it
// (Module.ariadneRunning). A no-op natively, and on a page that does not listen.
void report_running();

// Tell the page about a warning while the document runs (Module.ariadneWarning(text)), e.g. a
// resident that stopped, so it can show it then rather than only in the console. A no-op natively,
// and on a page that does not listen.
void report_warning(const std::string &text);

// Tell the page the runtime stopped with exit `status` (Module.ariadneExit); its stderr says why.
// A no-op natively, and on a page that does not listen.
void report_exit(int status);

} // namespace ariadne
} // namespace cvc
