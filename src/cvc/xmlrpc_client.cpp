#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/utility.h>
#include <xmlrpc/XmlRpc.h>

namespace CVC_NAMESPACE {
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
      const std::string &host, int port, const std::string &method_name,
      const XmlRpc::XmlRpcValue &params,
      boost::posix_time::ptime mod_time = boost::posix_time::microsec_clock::universal_time())
      : _host(host), _port(port), _method_name(method_name), _params(params), _mod_time(mod_time) {
    _thread_name =
        boost::str(boost::format("xmlrpc_client_thread_%s_%d_%s") % host % port % method_name);
  }

  xmlrpc_client_thread(const xmlrpc_client_thread &t)
      : _thread_name(t._thread_name), _host(t._host), _port(t._port), _method_name(t._method_name),
        _params(t._params), _mod_time(t._mod_time) {}

  xmlrpc_client_thread &operator=(const xmlrpc_client_thread &t) {
    if (this == &t)
      return *this;
    _thread_name = t._thread_name;
    _host = t._host;
    _port = t._port;
    _method_name = t._method_name;
    _params = t._params;
    _mod_time = t._mod_time;
    return *this;
  }

  void operator()() const {
    using namespace boost;
    using namespace std;

    using namespace boost;
    using namespace std;

    CVC_NAMESPACE::thread_feedback feedback;

    XmlRpc::XmlRpcClient c(_host.c_str(), _port);
    XmlRpc::XmlRpcValue params, result;

    params = _params;

    // params[0] = _params;

    // this might be ignored if the server has a newer value
    //       params[1] = posix_time::to_simple_string(_mod_time);

    for (int i = 0; i < 3; i++)
      cvcapp.log(6, str(boost::format("%s :: params[%d] = %s\n") % BOOST_CURRENT_FUNCTION % i %
                        string(params[i])));

    c.execute(_method_name.c_str(), params, result);

    cvcapp.data(thread_name(), result);
  }

  const std::string &thread_name() const { return _thread_name; }

protected:
  std::string _thread_name;
  std::string _host;
  int _port;
  std::string _method_name;
  XmlRpc::XmlRpcValue _params;
  boost::posix_time::ptime _mod_time;
};

// --------------------------
// queue_xmlrpc_client_thread
// --------------------------
// Purpose:
//   Queues up a thread on the data map to set a remote state value at a later time.
//   process_xmlrpc_client_threads is in charge of actually launching the thread.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
class queue_xmlrpc_client_thread {
public:
  queue_xmlrpc_client_thread(const xmlrpc_client_thread &xct) : _xct(xct) {}

  queue_xmlrpc_client_thread(const queue_xmlrpc_client_thread &q) : _xct(q._xct) {}

  queue_xmlrpc_client_thread &operator=(const queue_xmlrpc_client_thread &rhs) { _xct = rhs._xct; }

  void operator()() const {
    CVC_NAMESPACE::thread_feedback feedback;

    // Stick the thread on the datamap.  There is another thread that will actually launch
    // these threads some time later.  Doing this, we won't flood the network with xmlrpc
    // requests if data gets old before it is sent out over the network.
    cvcapp.data(_xct.thread_name(), _xct);
  }

private:
  xmlrpc_client_thread _xct;
};

// -----------------------------
// process_xmlrpc_client_threads
// -----------------------------
// Purpose:
//   Starts all the xmlrpc client threads that are on the data map.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- creation.
class process_xmlrpc_client_threads {
public:
  void operator()() const {
    CVC_NAMESPACE::thread_feedback feedback;

    while (1) {
      cvcapp.sleep(cvcstate("__system.xmlrpc_client.interval").value<double>());

      std::vector<std::string> keys = cvcapp.data<xmlrpc_client_thread>();
      std::vector<xmlrpc_client_thread> threads = cvcapp.data<xmlrpc_client_thread>(keys);
      BOOST_FOREACH (xmlrpc_client_thread &thread, threads) {
        if (cvcapp.hasThread(thread.thread_name()))
          continue;
        cvcapp.data(thread.thread_name(), boost::any()); // this will be replaced by the result
        cvcapp.startThread(thread.thread_name(), thread);
      }
    }
  }
};

// --------
// rpc_call
// --------
// Purpose:
//  Does an xmlrpc method call on the specified host and port.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
XmlRpc::XmlRpcValue rpc_call(const std::string &host, int port, const std::string &method_name,
                             const XmlRpc::XmlRpcValue &params, bool sync,
                             boost::posix_time::ptime mod_time) {
  if (sync) {
    xmlrpc_client_thread xct(host, port, method_name, params, mod_time);
    xct();
    return cvcapp.data<XmlRpc::XmlRpcValue>(xct.thread_name());
  } else // async calls are broken at the moment ... 01/13/2014
  {
    xmlrpc_client_thread xct(host, port, method_name, params, mod_time);
    cvcapp.data(xct.thread_name(), xct);
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
XmlRpc::XmlRpcValue rpc_call(const std::string &host, int port, const std::string &method_name,
                             const std::vector<std::string> &params, bool sync,
                             boost::posix_time::ptime mod_time) {
  XmlRpc::XmlRpcValue vals;
  int i = 0;
  BOOST_FOREACH (const std::string &p, params)
    vals[i++] = p;
  return rpc_call(host, port, method_name, vals, sync, mod_time);
}

// ---
// rpc
// ---
// Purpose:
//  Does an xmlrpc method call on the specified host and port.
// ---- Change History ----
// 01/13/2014 -- Joe R. -- Creation.
XmlRpc::XmlRpcValue rpc(const std::string &host_and_port, const std::string &method_name,
                        const std::vector<std::string> &params, bool sync,
                        boost::posix_time::ptime mod_time) {
  std::string host;
  int port = -1;
  boost::tie(host, port) = get_xmlrpc_host_and_port(host_and_port);
  return rpc_call(host, port, method_name, params, sync, mod_time);
}
} // namespace CVC_NAMESPACE

namespace {
class process_xmlrpc_client_threads_init {
public:
  static void init() {
    // Sleep for 200ms before each iteration by default.
    cvcstate("__system.xmlrpc_client.interval").value(double(200.0));
    cvcapp.startThread("process_xmlrpc_client_threads",
                       CVC_NAMESPACE::process_xmlrpc_client_threads());
  }

  process_xmlrpc_client_threads_init() {
    // Need to fix async calls.  For now, disabling this thread.
    // CVC_NAMESPACE::state::on_startup(init);
  }
} static_init;
} // namespace
