#ifndef CVC_NET_HTTP_BACKENDS_H
#define CVC_NET_HTTP_BACKENDS_H

// cvc::net internal — the compiled-in default-backend factory. NOT a shipped header (lives under
// src/, not inc/): it wires the dispatcher (http_client.cpp) to whichever backend TU is active.
//
// make_default_backend() is defined EXACTLY ONCE across the build by the mutually-exclusive backend
// translation units: http_client_curl.cpp under CVC_NET_HAVE_CURL, http_client_fetch.cpp under
// CVC_NET_HAVE_FETCH, and http_client.cpp itself (a NullHttpClient) when neither is compiled. Each
// of the three TUs is always compiled; the inactive ones #ifdef to empty, so the linker sees one
// definition. It always returns a non-null client.

#include <cvc/net/http_client.h>
#include <memory>

namespace cvc {
namespace net {
namespace detail {

std::unique_ptr<HttpClient> make_default_backend();

} // namespace detail
} // namespace net
} // namespace cvc

#endif // CVC_NET_HTTP_BACKENDS_H
