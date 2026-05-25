#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/utility/utility.h>
#include <xmlrpc/XmlRpc.h>

namespace cvc {
CVC_DEF_EXCEPTION(xmlrpc_server_error);
CVC_DEF_EXCEPTION(xmlrpc_server_error_listen);
CVC_DEF_EXCEPTION(xmlrpc_server_terminate);

#define XMLRPC_METHOD_PROTOTYPE(name, description)                                                 \
  class name : public XmlRpc::XmlRpcServerMethod {                                                 \
  public:                                                                                          \
    name(app &ctx, XmlRpc::XmlRpcServer *s) : XmlRpc::XmlRpcServerMethod(#name, s), _app(&ctx) {}  \
    void execute(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result);                        \
    std::string help() { return std::string(description); }                                        \
                                                                                                   \
  private:                                                                                         \
    app *_app;                                                                                     \
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
  explicit xmlrpc_server_thread(app &ctx) : _app(&ctx) {}

  void operator()() const {
    cvc::thread_feedback feedback(*_app);

    // document the tree regarding the xmlrpc server
    state::instance (*_app)("__system.xmlrpc.port").comment("The port used by the xmlrpc server.");

    try {
      using namespace boost;
      std::string host = asio::ip::host_name();
      std::string ipaddr = cvc::get_local_ip_address();

      // Useful info to have
      state::instance (*_app)("__system.xmlrpc.hostname")
          .value(host)
          .comment("The hostname of the host running the xmlrpc server thread.");
      state::instance (*_app)("__system.xmlrpc.ipaddr")
          .value(ipaddr)
          .comment("The ip address bound by the xmlrpc server.");

      // We are looping here so that we try to establish a listen port by incrementing
      // the number starting from the default until we don't throw an exception.
      int port = -1;
      while (1) {
        try {
          port = state::instance(*_app)("__system.xmlrpc.port").value<int>();
        } catch (bad_lexical_cast &) {
          // throw xmlrpc_server_error("invalid port");

          // use the default
          port = XMLRPC_DEFAULT_PORT;
          state::instance (*_app)("__system.xmlrpc.port").value(XMLRPC_DEFAULT_PORT);
        }
        std::string portstr = state::instance(*_app)("__system.xmlrpc.port");

        try {
          // instantiate the server and its methods.
          XmlRpc::XmlRpcServer s;
          cvcstate_set_value set_value(*_app, &s);
          cvcstate_get_value get_value(*_app, &s);
          cvcstate_get_children get_children(*_app, &s);
          cvcstate_get_num_children get_num_children(*_app, &s);
          cvcstate_get_json get_json(*_app, &s);
          cvcstate_set_json set_json(*_app, &s);
          cvcstate_get_lastmod lastmod(*_app, &s);
          cvcstate_touch touch(*_app, &s);
          cvcstate_reset reset(*_app, &s);
          cvcstate_terminate terminate(*_app, &s);

          // Start the server, and run it indefinitely.
          // For some reason, time_from_string and boost_regex creashes if the main thread is
          // waiting in atexit(). So, make sure main() has a wait_for_threads() call at the
          // end.
          XmlRpc::setVerbosity(0);
          if (!s.bindAndListen(port))
            throw xmlrpc_server_error_listen(
                str(boost::format("could not bind to port %d") % port));
          s.enableIntrospection(true);
          // s.work(-1.0);

          _app->log(1, str(boost::format("%s :: \n%s\n") % BOOST_CURRENT_FUNCTION %
                           state::instance(*_app)("__system").json()));

          // loop with interruption points so we can gracefully terminate
          while (1) {
            boost::this_thread::interruption_point();
            s.work(200.0); // work for 200ms
          }
        } catch (xmlrpc_server_error_listen &) {
          port++;
          state::instance (*_app)("__system.xmlrpc.port").value(port);
        } catch (std::exception &e) {
          using namespace boost;
          _app->log(1, str(boost::format(
                               "%s :: restarting server on xmlrpc_server_thread exception: %s\n") %
                           BOOST_CURRENT_FUNCTION % e.what()));
        }
      }
    } catch (boost::thread_interrupted &) {
      using namespace boost;
      _app->log(1, str(boost::format("%s :: xmlrpc_server_thread interrupted, shutting down\n") %
                       BOOST_CURRENT_FUNCTION));
    }
  }

  static void shutdown(app &ctx) {
    ctx.sleep(5000.0);
    state::instance(ctx)("__system.xmlrpc").value(int(0));
    ctx.log(3, boost::str(boost::format("%s :: shutting down\n") % BOOST_CURRENT_FUNCTION));
  }

private:
  app *_app;

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

XMLRPC_METHOD_DEFINITION(cvcstate_set_value) {
  state::instance (*_app)(params[0]).value(std::string(params[1]));
}

XMLRPC_METHOD_DEFINITION(cvcstate_get_value) { result = state::instance(*_app)(params[0]).value(); }

XMLRPC_METHOD_DEFINITION(cvcstate_get_children) {
  using namespace std;
  vector<string> ret = state::instance(*_app).children(params[0]);
  for (size_t i = 0; i < ret.size(); i++)
    result[i] = ret[i];
}

XMLRPC_METHOD_DEFINITION(cvcstate_get_num_children) {
  result = int(state::instance(*_app)(params[0]).numChildren());
}

XMLRPC_METHOD_DEFINITION(cvcstate_get_json) { result = state::instance(*_app)(params[0]).json(); }

XMLRPC_METHOD_DEFINITION(cvcstate_set_json) { state::instance (*_app)(params[0]).json(params[1]); }

XMLRPC_METHOD_DEFINITION(cvcstate_get_lastmod) {
  result = boost::posix_time::to_simple_string(state::instance(*_app)(params[0]).lastMod());
}

XMLRPC_METHOD_DEFINITION(cvcstate_touch) { state::instance (*_app)(params[0]).touch(); }

XMLRPC_METHOD_DEFINITION(cvcstate_reset) { state::instance (*_app)(params[0]).reset(); }

XMLRPC_METHOD_DEFINITION(cvcstate_terminate) {
  app *a = _app;
  _app->startThread("xmlrpc_server_thread_shutdown", [a]() { xmlrpc_server_thread::shutdown(*a); });
}
} // namespace cvc

namespace {
class xmlrpc_server_thread_init {
public:
  static void monitor(cvc::app &ctx) {
    try {
      if (!cvc::state::instance(ctx)("__system.xmlrpc").value().empty() &&
          boost::lexical_cast<int>(cvc::state::instance(ctx)("__system.xmlrpc").value())) {
        // Create a new XMLRPC thread to handle IPC
        if (ctx.hasThread("xmlrpc_server_thread"))
          ctx.threads("xmlrpc_server_thread")->interrupt();
        ctx.startThread("xmlrpc_server_thread", cvc::xmlrpc_server_thread(ctx), false);
      } else {
        if (ctx.hasThread("xmlrpc_server_thread")) {
          ctx.threads("xmlrpc_server_thread")->interrupt();
        }
      }
    } catch (boost::bad_lexical_cast &) {
      ctx.log(3, boost::str(boost::format("%s :: error parsing __system.xmlrpc\n") %
                            BOOST_CURRENT_FUNCTION));
    }
  }

  // sets a monitor function to observe the value of __system.xmlrpc.
  // If it is set to anything that evaluates to true, the xmlrpc server thread will be started.
  // If it is set to false, the running xmlrpc server will be terminated.
  static void init(cvc::app &ctx) {
    cvc::state::instance(ctx)("__system.xmlrpc").valueChanged.connect([&ctx]() { monitor(ctx); });
    monitor(ctx);
  }

  xmlrpc_server_thread_init() { cvc::state::on_startup(cvc::state::app_init_func(init)); }
} static_init;
} // namespace
