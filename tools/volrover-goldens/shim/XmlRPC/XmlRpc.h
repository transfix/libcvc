// legacy GenSeg's header declares its XML-RPC entry, generalSegmentation(XmlRpcValue &, ...), which
// the goldens never build (genseg_driver.cpp calls what it called). A declaration is all genseg.h
// needs. Part of tools/volrover-goldens: never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_XMLRPC_H
#define VOLROVER_GOLDENS_SHIM_XMLRPC_H

namespace XmlRpc {
class XmlRpcValue;
}

#endif
