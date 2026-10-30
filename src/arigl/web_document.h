// arigl-web: a document named by URL. The browser shell (web/arigl-web.html) runs
// `arigl run [flags] https://<origin>/<path>/main.ari`; before the load, arigl fetches that
// document and the files it names (cvc/ariadne/doc_fetch.h) into MEMFS at their URL paths, runs
// in the document's directory, and loads the local copy, so its relative imports resolve as they
// would on disk. The page's own origin is the only one allowed unless the page lists others.
#pragma once

#include <cvc/ariadne/host_cli.h>
#include <string>

namespace arigl {

// Whether `doc` names a document by an http:// or https:// URL.
bool is_url_document(const std::string &doc);

// Fetch the URL document in o.doc and point o.doc at its local copy. Returns -1 to carry on, else
// the exit status (1: it could not be loaded, and stderr says why). Natively a URL is refused:
// arigl runs a document from a path there.
//
// The page passes its config through the environment:
//   CVC_ARIGL_SRC_ORIGINS  other origins the document may come from, space-separated ("*": any)
//   CVC_ARIGL_PRELOAD      more files to fetch with it, relative to it, one per line
int stage_url_document(cvc::ariadne::HostOptions &o);

// Tell the page the document is loaded and its loop is about to run. A no-op natively, and on a
// page that does not listen.
void report_running();

// Tell the page that arigl stopped before or while running its document with exit `status` (its
// stderr says why). A no-op natively, and on a page that does not listen.
void report_exit(int status);

} // namespace arigl
