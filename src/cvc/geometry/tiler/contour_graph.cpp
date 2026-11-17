#include <ContourTiler/contour_graph.h>
#include <ContourTiler/reader_ser.h>
#include <ContourTiler/reader_csv.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

class Apical_synapse
{
public:
  Apical_synapse() {}
  Apical_synapse(const std::string& name, double z) : _name(name), _z(z) {}
  ~Apical_synapse() {}

  friend bool operator<(const Apical_synapse& a, const Apical_synapse& b)
  { return a.z() < b.z(); }

  const std::string& name() const { return _name; }
  double z() const { return _z; }

  std::string to_string() const
  { return _name + ", " + boost::lexical_cast<string>(_z); }

private:
  std::string _name;
  double _z;
};

template <typename OutputIterator>
void read_apical_synapses(OutputIterator synapses)
{
  using namespace std;
  typedef boost::tokenizer<boost::escaped_list_separator<char> > tokenizer;

  string filename("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_tiled/synapse_z_values.csv");
  ifstream in(filename.c_str());
  string line;

  getline(in, line);
  while (line != "")
  {
    tokenizer tok(line);
    int index = 0;
    string name;
    double z;
    for(tokenizer::iterator it=tok.begin(); it!=tok.end(); ++it, ++index)
    {
      if (index == 0) name = *it;
      else z = boost::lexical_cast<double>(*it);
    }
    *synapses = Apical_synapse(name, z);
    ++synapses;

    getline(in, line);
  }

  in.close();
}

void test_graph()
{
  using namespace std;

  string ser_filename("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/Volumejosef.ser");
  list<string> csv_files;
  csv_files.push_back("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/cross_sectioned_shaft.csv");
  csv_files.push_back("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/partially_oblique_shaft.csv");
  csv_files.push_back("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/cross_sectioned_spine.csv");
  csv_files.push_back("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/oblique_spine.csv");
  csv_files.push_back("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/partially_oblique_spine.csv");
  csv_files.push_back("/org/centers/cvc/data/neuropil/CA1/20070926/beth_bell_traces/en_face_spine.csv");

  list<Contour_handle> contours;

  cout << "Testing graph" << endl;

  // Get the synapses from the csv files
  typedef list<boost::tuple<string, string, string> > Synapses;
  Synapses synapses;
  for (list<string>::const_iterator it = csv_files.begin(); it != csv_files.end(); ++it)
    read_synapses_csv(*it, back_inserter(synapses));
  for (Synapses::const_iterator it = synapses.begin(); it != synapses.end(); ++it)
    cout << boost::get<0>(*it) << endl;

  // Get the contours from the ser files
  list<Contour_exception> exceptions;
  read_contours_ser(ser_filename, -1, back_inserter(contours), back_inserter(exceptions));

  print_contours(contours.begin(), contours.end());
  cout << "Failed contours: " << exceptions.size() << endl;

  list<Apical_synapse> ap_syn;
  read_apical_synapses(std::back_inserter(ap_syn));
  ap_syn.sort();

  // Build the graph
  test_graph(contours.begin(), contours.end(), synapses, ap_syn.begin(), ap_syn.end());

  contours.clear();
}

CONTOURTILER_END_NAMESPACE
