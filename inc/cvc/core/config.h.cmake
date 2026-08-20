#ifndef CONFIG_H
#define CONFIG_H

#cmakedefine LOG4CPLUS_DISABLE_TRACE
#cmakedefine CVC_HDF5_DISABLED

// Installed data directory for cvc::nav (the .cvcnav navigation policy weights
// live at ${CVC_NAV_DATADIR}/coef_mlp.cvcnav). Baked from the install prefix at
// configure time; a relocated install resolves relative to the loaded library
// instead (see coef_mlp::default_weights_path).
#define CVC_NAV_DATADIR "@CVC_NAV_DATADIR@"

#define NOMINMAX

#ifdef __WINDOWS__
#include <WinSock2.h>
#endif

#endif // CONFIG_H
