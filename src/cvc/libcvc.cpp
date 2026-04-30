#include <cvc/algorithm.h>
#include <cvc/app.h>
#include <cvc/geometry_file_io.h>
#include <cvc/state.h>
#include <cvc/utility.h>
#include <cvc/volume_file_info.h>

// #include <boost/tuple/tuple.hpp>
#include <boost/current_function.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

/*
 * Commands
 */

typedef boost::function<void(const std::vector<std::string> &)> command_func;
typedef boost::tuple<command_func, std::string> command;
typedef std::map<std::string, command> command_map;
command_map commands;

void help(const std::vector<std::string> &args) {
  using namespace std;
  cout << "Usage: libcvc <command> <command args>" << endl << endl;
  for (command_map::iterator i = commands.begin(); i != commands.end(); ++i) {
    cout << " - " << i->first << endl;
    cout << i->second.get<1>() << endl << endl;
  }
}

void copy(const std::vector<std::string> &args) {
  using namespace std;
  using namespace boost;
  using namespace CVC_NAMESPACE;
  thread_info ti(BOOST_CURRENT_FUNCTION);

  if (args.empty())
    throw command_line_error("Missing input filename.");
  if (args.size() < 2)
    throw command_line_error("Missing output volume filename");
  save(load(args[0]), args[1]);
}

void info(const std::vector<std::string> &args) {
  using namespace std;
  using namespace boost;
  using namespace CVC_NAMESPACE;
  thread_info ti(BOOST_CURRENT_FUNCTION);

  if (args.empty())
    throw command_line_error("Missing input filename.");

  if (is_geometry_filename(args[0])) {
    geometry geo(args[0]);
    if (geo.num_points() > 0)
      cout << "Num vertices: " << geo.num_points() << endl;
    if (geo.num_lines() > 0)
      cout << "Num lines: " << geo.num_lines() << endl;
    if (geo.num_tris() > 0)
      cout << "Num tris: " << geo.num_tris() << endl;
    if (geo.num_quads() > 0)
      cout << "Num quads: " << geo.num_quads() << endl;

    point_t min_pt = geo.min_point();
    point_t max_pt = geo.max_point();
    cout << "min_point: " << str(boost::format("%1%,%2%,%3%") % min_pt[0] % min_pt[1] % min_pt[2])
         << endl;
    cout << "max_point: " << str(boost::format("%1%,%2%,%3%") % max_pt[0] % max_pt[1] % max_pt[2])
         << endl;
  } else if (is_volume_filename(args[0])) {
    volume_file_info volinfo;
    volinfo.read(args[0]);
    cout << volinfo.filename() << ":" << endl;
    cout << "Num Variables: " << volinfo.numVariables() << endl;
    cout << "Num Timesteps: " << volinfo.numTimesteps() << endl;
    cout << "Dimension: " << volinfo.XDim() << "x" << volinfo.YDim() << "x" << volinfo.ZDim()
         << endl;
    cout << "Bounding box: ";
    cout << "(" << volinfo.boundingBox().minx << "," << volinfo.boundingBox().miny << ","
         << volinfo.boundingBox().minz << ") ";
    cout << "(" << volinfo.boundingBox().maxx << "," << volinfo.boundingBox().maxy << ","
         << volinfo.boundingBox().maxz << ") ";
    cout << endl;
    cout << "Span: " << "(" << volinfo.XSpan() << "," << volinfo.YSpan() << "," << volinfo.ZSpan()
         << ") " << endl;
    double volmin = volinfo.min(), volmax = volinfo.max();
    for (unsigned int i = 0; i < volinfo.numVariables(); i++) {
      cout << "Name of var " << i << ": " << volinfo.name(i) << endl;
      cout << "Voxel type of var " << i << ": " << volinfo.voxelTypeStr(i) << endl;
      for (unsigned int j = 0; j < volinfo.numTimesteps(); j++) {
        if (volmin > volinfo.min(i, j))
          volmin = volinfo.min(i, j);
        if (volmax < volinfo.max(i, j))
          volmax = volinfo.max(i, j);
        cout << "Min voxel value of var " << i << ", timestep " << j << ": " << volinfo.min(i, j)
             << endl;
        cout << "Max voxel value of var " << i << ", timestep " << j << ": " << volinfo.max(i, j)
             << endl;
      }
    }
    cout << "Min voxel value (of whole dataset): " << volmin << endl;
    cout << "Max voxel value (of whole dataset): " << volmax << endl;
  } else
    throw read_error("Unknown data");
}

// 01/11/2014 - Joe R. - moved bounding box to last argument and made optional.
void sdf(const std::vector<std::string> &args) {
  using namespace std;
  using namespace CVC_NAMESPACE;
  thread_info ti(BOOST_CURRENT_FUNCTION);
  if (args.empty())
    throw command_line_error("Missing geometry filename");
  if (args.size() < 2)
    throw command_line_error("Missing dimension");
  if (args.size() < 3)
    throw command_line_error("Missing output volume filename");

  geometry geom(args[0]);
  bounding_box bbox = geom.extents();
  if (args.size() >= 4)
    bbox = bounding_box(args[3]);

  sdf(geom, dimension(args[1]), bbox).write(args[2]);
}

void iso(const std::vector<std::string> &args) {
  using namespace std;
  using namespace boost;
  using namespace CVC_NAMESPACE;
  thread_info ti(BOOST_CURRENT_FUNCTION);
  if (args.empty())
    throw command_line_error("Missing volume filename");
  if (args.size() < 2)
    throw command_line_error("Missing isovalue");
  if (args.size() < 3)
    throw command_line_error("Missing output geometry filename");
  iso(volume(args[0]), lexical_cast<double>(args[1])).write(args[2]);
}

#ifdef USING_XMLRPC
void server(const std::vector<std::string> &args) {
  using namespace std;
  using namespace boost;
  using namespace CVC_NAMESPACE;
  thread_info ti(BOOST_CURRENT_FUNCTION);
  int port = XMLRPC_DEFAULT_PORT;
  if (!args.empty())
    port = lexical_cast<int>(args[0]);
  cvcstate("__system.xmlrpc.port").value(port);
  cvcstate("__system.xmlrpc").value(int(1)); // start the server
  cvcapp.wait();                             // wait for the server thread to quit
}

void client(const std::vector<std::string> &args) {
  using namespace std;
  using namespace boost;
  using namespace CVC_NAMESPACE;
  thread_info ti(BOOST_CURRENT_FUNCTION);
  if (args.empty())
    throw command_line_error("Missing host:port");
  if (args.size() < 2)
    throw command_line_error("Missing method name");
  std::string host_and_port = args[0];
  std::string method_name = args[1];
  vector<string> rpc_args = args;
  rpc_args.erase(rpc_args.begin(), rpc_args.begin() + 2);
  string result = rpc(host_and_port, method_name, rpc_args);
  if (!result.empty())
    cout << result << endl;
}
#endif

class init_commands {
public:
  init_commands() {
    using namespace std;
    using boost::format;
    using boost::make_tuple;
    using boost::str;

    commands["copy"] =
        boost::make_tuple(command_func(copy),
                          string("copy <input filename> <output filename>\n"
                                 "Copies input to output file, possibly converting file formats."));
    commands["help"] = boost::make_tuple(command_func(help), string("Prints command list."));
    commands["iso"] = boost::make_tuple(
        command_func(iso), string("iso <volume filename> <isovalue> <output geometry filename>\n"
                                  "Computes isosurface geometry for a given volume and isovalue.\n"
                                  "<isovalue> - value describing the surface to be computed."));
    commands["sdf"] = boost::make_tuple(
        command_func(sdf),
        string(
            "sdf <geometry filename> <dimension> <output volume filename> [bounding_box]\n"
            "Computes a signed distance function volume of the geometry specified.\n"
            "<dimension> - comma separated list of 3 integers specifying output volume dimension\n"
            "[bounding_box] - comma separated list of 6 floats specifying output volume bounding "
            "box\n"
            "                 If unspecified, defaults to the extents of the input geometry."));
    commands["info"] =
        boost::make_tuple(command_func(info), string("info <filename>\n"
                                                     "Prints info about the specified file."));
#ifdef USING_XMLRPC
    commands["server"] = boost::make_tuple(
        command_func(server),
        str(boost::format(string("server [port]\n"
                                 "Starts an xmlrpc server at the specified port. Defaults to %d")) %
            CVC_NAMESPACE::XMLRPC_DEFAULT_PORT));
    commands["client"] = boost::make_tuple(
        command_func(client), string("client <host:port> <xmlrpc_method> [method args]\n"
                                     "Calls an rpc method on the target host:port."));
#endif
  }
} static_init;
} // namespace

int main(int argc, char **argv) {
  using namespace std;
  using namespace CVC_NAMESPACE;

  vector<string> args;
  for (int i = 0; i < argc; i++)
    args.push_back(argv[i]);

  try {
    args.erase(args.begin()); // no need for the first arg
    if (args.empty())
      throw command_line_error("Missing command string");
    string cmd = args[0];
    args.erase(args.begin()); // erase the command string

    if (commands.find(cmd) == commands.end())
      throw command_line_error("Invalid command.");

    commands[cmd].get<0>()(args);
  } catch (command_line_error &e) {
    if (!e.what_str().empty())
      cout << "Error: " << e.what_str() << endl;
    cout << "Usage: " << argv[0] << " <command> <command args>" << endl;
    return EXIT_FAILURE;
  } catch (CVC_NAMESPACE::exception &e) {
    cerr << "Exception: " << e.what() << endl;
  }

  return EXIT_SUCCESS;
}
