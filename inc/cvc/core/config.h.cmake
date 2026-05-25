#ifndef CONFIG_H
#define CONFIG_H

#cmakedefine LOG4CPLUS_DISABLE_TRACE
#cmakedefine CVC_HDF5_DISABLED

#define NOMINMAX

#ifdef __WINDOWS__
#include <WinSock2.h>
#endif

#endif // CONFIG_H
