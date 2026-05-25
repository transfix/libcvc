#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <cvc/app.h>
#include <cvc/utility.h>
#include <xmlrpc/XmlRpc.h>

namespace cvc {
// --------------------
// xmlrpc_client_thread
// --------------------
// Purpose:
//   Performs a client xmlrpc call to the specified method.
// ---- Change History ----
// 01/12/2014 -- Joe R. -- Creation.
class xmlrpc_client_thread {
public:
  xmlrpc_client_thread(
      app &ctx, const std::string &host, int port, const std::string &method_name,
      const XmlRpc::XmlRpcValue &params,
      boost::posix_time::ptime mod_time = boost::posix_time::microsec_clock::universal_time())
      : _app(&ctx), _host(host), _port(port), _method_name(method_name), _params(params),
        _mod_time(mod_time) {
    _thread_name =
        boost::str(boost::format("xmlrpc_client_thread_%s_%d_%s") % host % port % method_name);
  }

  xmlrpc_client_thread(const xmlrpc_client_thread &t) = default;
  xmlrpc_client_thread &operator=(const xmlrpc_client_thread &t) = default;

  void operator()() const {
    using namespace boost;
    using namespace std;

    cvc::thread_feedback feedback(*_app);

    XmlRpc::XmlRpcClient c(_host.c_str(), _port);
    XmlRpc::XmlRpcValue params, result;

    params = _params;

    // params[0] = _params;

    // this might be ignored if the server has a newer value
    //       params[1] = posix_time::to_simple_string(_mod_time);

    for (int i = 0; i < 3; i++)
      _app->log(6, str(boost::format("%s :: params[%d] = %s\n") % BOOST_CURRENT_FUNCTION % i %
                       string(params[i])));

    c.execute(_method_name.c_str(), params, result);

    _app->data(thread_name(), result);
  }

  const std::string &thread_name() const { return _thread_name; }

protected:
  app *_app;
  std::string _thread_name;
  std::string _host;
  int _port;
  std::string _method_name;
  XmlRpc::XmlRpcValue _params;
  boost::posix_time::ptime _mod_time;
};

// --------
// rpc_call
// --------
// Purpose:
//  Does an xmlrpc method call on the specified host and port.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
XmlRpc::XmlRpcValue rpc_call(app &ctx, const std::string &host, int port,
                             const std::string &method_name, const XmlRpc::XmlRpcValue &params,
                             bool sync, boost::posix_time::ptime mod_time) {
  if (sync) {
    xmlrpc_client_thread xct(ctx, host, port, method_name, params, mod_time);
    xct();
    return ctx.data<XmlRpc::XmlRpcValue>(xct.thread_name());
  } else // async calls are broken at the moment ... 01/13/2014
  {
    xmlrpc_client_thread xct(ctx, host, port, method_name, params, mod_time);
    ctx.data(xct.thread_name(), xct);
    return XmlRpc::XmlRpcValue();
  }
}

// --------
// rpc_call
// --------
// Purpose:
//  Does an xmlrpc method call on the specified host and port.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
XmlRpc::XmlRpcValue rpc_call(app &ctx, const std::string &host, int port,
                             const std::string &method_name, const std::vector<std::string> &params,
                             bool sync, boost::posix_time::ptime mod_time) {
  XmlRpc::XmlRpcValue vals;
  int i = 0;
  BOOST_FOREACH (const std::string &p, params)
    vals[i++] = p;
  return rpc_call(ctx, host, port, method_name, vals, sync, mod_time);
}

// ---
// rpc
// ---
// Purpose:
//  Does an xmlrpc method call on the specified host and port.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
XmlRpc::XmlRpcValue rpc(app &ctx, const std::string &host_and_port, const std::string &method_name,
                        const std::vector<std::string> &params, bool sync,
                        boost::posix_time::ptime mod_time) {
  std::string host;
  int port = -1;
  boost::tie(host, port) = get_xmlrpc_host_and_port(host_and_port);
  return rpc_call(ctx, host, port, method_name, params, sync, mod_time);
}
} // namespace cvc
