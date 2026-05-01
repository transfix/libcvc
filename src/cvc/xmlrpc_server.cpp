#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/utility.h>
#include <xmlrpc/XmlRpc.h>

namespace CVC_NAMESPACE {
CVC_DEF_EXCEPTION(xmlrpc_server_error);
CVC_DEF_EXCEPTION(xmlrpc_server_error_listen);
CVC_DEF_EXCEPTION(xmlrpc_server_terminate);

#define XMLRPC_METHOD_PROTOTYPE(name, description)                                                 \
  class name : public XmlRpc::XmlRpcServerMethod {                                                 \
  public:                                                                                          \
    name(XmlRpc::XmlRpcServer *s) : XmlRpc::XmlRpcServerMethod(#name, s) {}                        \
    void execute(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result);                        \
    std::string help() { return std::string(description); }                                        \
  };

#define XMLRPC_METHOD_DEFINITION(name)                                                             \
  void xmlrpc_server_thread::name::execute(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result)

// --------------------
// xmlrpc_server_thread
// --------------------
// Purpose:
//   The thread that manages the XmlRpcServer instance.
// ---- Change History ----
// 02/20/2012 -- Joe R. -- Creation.
// 02/24/2012 -- Joe R. -- Moving default initilization here to avoid deadlock
// 03/02/2012 -- Joe R. -- Running a thread to sync up with other hosts.
// 03/10/2012 -- Joe R. -- Starting process_notify_xmlrpc_threads.
// 01/12/2014 -- Joe R. -- Now looping to establish a listen port.
// 01/13/2014 -- Joe R. -- No more process_notify_xmlrpc_threads.
class xmlrpc_server_thread {
public:
  xmlrpc_server_thread() {}

  void operator()() const {
    CVC_NAMESPACE::thread_feedback feedback;

    // document the tree regarding the xmlrpc server
    cvcstate("__system.xmlrpc.port").comment("The port used by the xmlrpc server.");

    try {
      using namespace boost;
      std::string host = asio::ip::host_name();
      std::string ipaddr = CVC_NAMESPACE::get_local_ip_address();

      // Useful info to have
      cvcstate("__system.xmlrpc.hostname")
          .value(host)
          .comment("The hostname of the host running the xmlrpc server thread.");
      cvcstate("__system.xmlrpc.ipaddr")
          .value(ipaddr)
          .comment("The ip address bound by the xmlrpc server.");

      // We are looping here so that we try to establish a listen port by incrementing
      // the number starting from the default until we don't throw an exception.
      int port = -1;
      while (1) {
        try {
          port = cvcstate("__system.xmlrpc.port").value<int>();
        } catch (bad_lexical_cast &) {
          // throw xmlrpc_server_error("invalid port");

          // use the default
          port = XMLRPC_DEFAULT_PORT;
          cvcstate("__system.xmlrpc.port").value(XMLRPC_DEFAULT_PORT);
        }
        std::string portstr = cvcstate("__system.xmlrpc.port");

        try {
          // instantiate the server and its methods.
          XmlRpc::XmlRpcServer s;
          cvcstate_set_value set_value(&s);
          cvcstate_get_value get_value(&s);
          cvcstate_get_children get_children(&s);
          cvcstate_get_num_children get_num_children(&s);
          cvcstate_get_json get_json(&s);
          cvcstate_set_json set_json(&s);
          cvcstate_get_lastmod lastmod(&s);
          cvcstate_touch touch(&s);
          cvcstate_reset reset(&s);
          cvcstate_terminate terminate(&s);

          // Start the server, and run it indefinitely.
          // For some reason, time_from_string and boost_regex creashes if the main thread is
          // waiting in atexit(). So, make sure main() has a cvcapp.wait_for_threads() call at the
          // end.
          XmlRpc::setVerbosity(0);
          if (!s.bindAndListen(port))
            throw xmlrpc_server_error_listen(
                str(boost::format("could not bind to port %d") % port));
          s.enableIntrospection(true);
          // s.work(-1.0);

          cvcapp.log(1, str(boost::format("%s :: \n%s\n") % BOOST_CURRENT_FUNCTION %
                            cvcstate("__system").json()));

          // loop with interruption points so we can gracefully terminate
          while (1) {
            boost::this_thread::interruption_point();
            s.work(200.0); // work for 200ms
          }
        } catch (xmlrpc_server_error_listen &) {
          port++;
          cvcstate("__system.xmlrpc.port").value(port);
        } catch (std::exception &e) {
          using namespace boost;
          cvcapp.log(1, str(boost::format(
                                "%s :: restarting server on xmlrpc_server_thread exception: %s\n") %
                            BOOST_CURRENT_FUNCTION % e.what()));
        }
      }
    } catch (boost::thread_interrupted &) {
      using namespace boost;
      cvcapp.log(1, str(boost::format("%s :: xmlrpc_server_thread interrupted, shutting down\n") %
                        BOOST_CURRENT_FUNCTION));
    }
  }

  static void shutdown() {
    cvcapp.sleep(5000.0);
    cvcstate("__system.xmlrpc").value(int(0));
    cvcapp.log(3, boost::str(boost::format("%s :: shutting down\n") % BOOST_CURRENT_FUNCTION));
  }

private:
  // our exported methods
  XMLRPC_METHOD_PROTOTYPE(cvcstate_set_value, "Sets a state object's value.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_get_value, "Gets a state object's value.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_get_children,
                          "Get a list of root's children using a PERL regular expression.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_get_num_children,
                          "Returns the number of children of the requested cvcstate object.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_get_json,
                          "Get a json representation of the requested cvcstate object.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_set_json, "Set a cvcstate object from a json representation.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_get_lastmod, "Returns a state's last modified time.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_touch,
                          "Touches the state, triggering listeners as if it was written to.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_reset, "Resets the state.");
  XMLRPC_METHOD_PROTOTYPE(cvcstate_terminate, "Quits the server.");
};

XMLRPC_METHOD_DEFINITION(cvcstate_set_value) { cvcstate(params[0]).value(std::string(params[1])); }

XMLRPC_METHOD_DEFINITION(cvcstate_get_value) { result = cvcstate(params[0]).value(); }

XMLRPC_METHOD_DEFINITION(cvcstate_get_children) {
  using namespace std;
  vector<string> ret = cvcstate().children(params[0]);
  for (size_t i = 0; i < ret.size(); i++)
    result[i] = ret[i];
}

XMLRPC_METHOD_DEFINITION(cvcstate_get_num_children) {
  result = int(cvcstate(params[0]).numChildren());
}

XMLRPC_METHOD_DEFINITION(cvcstate_get_json) { result = cvcstate(params[0]).json(); }

XMLRPC_METHOD_DEFINITION(cvcstate_set_json) { cvcstate(params[0]).json(params[1]); }

XMLRPC_METHOD_DEFINITION(cvcstate_get_lastmod) {
  result = boost::posix_time::to_simple_string(cvcstate(params[0]).lastMod());
}

XMLRPC_METHOD_DEFINITION(cvcstate_touch) { cvcstate(params[0]).touch(); }

XMLRPC_METHOD_DEFINITION(cvcstate_reset) { cvcstate(params[0]).reset(); }

XMLRPC_METHOD_DEFINITION(cvcstate_terminate) {
  cvcapp.startThread("xmlrpc_server_thread_shutdown", shutdown);
}
} // namespace CVC_NAMESPACE

namespace {
class xmlrpc_server_thread_init {
public:
  static void monitor() {
    try {
      if (!cvcstate("__system.xmlrpc").value().empty() &&
          boost::lexical_cast<int>(cvcstate("__system.xmlrpc").value())) {
        // Create a new XMLRPC thread to handle IPC
        if (cvcapp.hasThread("xmlrpc_server_thread"))
          cvcapp.threads("xmlrpc_server_thread")->interrupt();
        cvcapp.startThread("xmlrpc_server_thread", CVC_NAMESPACE::xmlrpc_server_thread(), false);
      } else {
        if (cvcapp.hasThread("xmlrpc_server_thread")) {
          cvcapp.threads("xmlrpc_server_thread")->interrupt();
        }
      }
    } catch (boost::bad_lexical_cast &) {
      cvcapp.log(3, boost::str(boost::format("%s :: error parsing __system.xmlrpc\n") %
                               BOOST_CURRENT_FUNCTION));
    }
  }

  // sets a monitor function to observe the value of __system.xmlrpc.
  // If it is set to anything that evaluates to true, the xmlrpc server thread will be started.
  // If it is set to false, the running xmlrpc server will be terminated.
  static void init() {
    cvcstate("__system.xmlrpc").valueChanged.connect(monitor);
    monitor();
  }

  xmlrpc_server_thread_init() { CVC_NAMESPACE::state::on_startup(init); }
} static_init;
} // namespace
