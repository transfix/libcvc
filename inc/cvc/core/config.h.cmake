#ifndef CONFIG_H
#define CONFIG_H

#cmakedefine LOG4CPLUS_DISABLE_TRACE
#cmakedefine CVC_HDF5_DISABLED

// Installed data directory for cvc::nav (the .cvcnav navigation policy weights
// live at ${CVC_NAV_DATADIR}/coef_mlp.cvcnav). Baked from the install prefix at
// configure time; a relocated install resolves relative to the loaded library
// instead (see coef_mlp::default_weights_path).
#define CVC_NAV_DATADIR "@CVC_NAV_DATADIR@"

// libcvc version — the single source of truth is the top-level
// project(libcvc VERSION ...). types.h includes this header, so this supersedes
// its stale "3.0.0" CVC_VERSION_STRING fallback and the real version reaches
// every consumer (HDF5 provenance stamps, the Ariadne .ari min_libcvc load gate).
#define CVC_VERSION_MAJOR @PROJECT_VERSION_MAJOR@
#define CVC_VERSION_MINOR @PROJECT_VERSION_MINOR@
#define CVC_VERSION_PATCH @PROJECT_VERSION_PATCH@
#define CVC_VERSION_STRING "@PROJECT_VERSION@"

#define NOMINMAX

#ifdef __WINDOWS__
#include <WinSock2.h>
#endif

#endif // CONFIG_H
