#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/utility/utility.h>
#include <fstream>

namespace cvc {
CVC_DEF_EXCEPTION(invalid_cvcraw_file);

struct cvcraw_io : public geometry_file_io {
  // --------------------
  // cvcraw_io::cvcraw_io
  // --------------------
  // Purpose:
  //   Initializes the extension list and id.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  cvcraw_io() : _id("cvcraw_io : v1.0") {
    _extensions.push_back(".raw");
    _extensions.push_back(".rawn");
    _extensions.push_back(".rawnc");
    _extensions.push_back(".rawc");
  }

  // -------------
  // cvcraw_io::id
  // -------------
  // Purpose:
  //   Returns a string that identifies this geometry_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual const std::string &id() const { return _id; }

  // ---------------------
  // cvcraw_io::extensions
  // ---------------------
  // Purpose:
  //   Returns a list of extensions that this geometry_file_io object supports.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual const extension_list &extensions() const { return _extensions; }

  // ---------------
  // cvcraw_io::read
  // ---------------
  // Purpose:
  //   Reads a file and outputs a geometry object.
  // ---- Change History ----
  // 04/02/2010 -- Joe R. -- Creation.
  // 01/11/2014 -- Joe R. -- Adapted for geometry_file_io.
  virtual geometry read(const std::string &filename) const {
    using namespace std;
    using namespace boost;

    geometry ret_geom;

    ifstream inf(filename.c_str());
    if (!inf)
      throw read_error(string("Could not open ") + filename);

    unsigned int line_num = 0;
    string line;
    vector<string> split_line;
    unsigned int num_verts, num_elems;

    getline(inf, line);
    line_num++;
    if (!inf)
      throw read_error(
          str(boost::format("Error reading file %1%, line %2%") % filename % line_num));
    trim(line);
    split(split_line, line, is_any_of(" "), token_compress_on);
    if (split_line.size() != 1 && split_line.size() != 2)
      throw read_error(str(boost::format("Not a cvc-raw file (wrong number of tokens: [%1%])") %
                           split_line.size()));

    try {
      num_verts = lexical_cast<unsigned int>(split_line[0]);
      num_elems = split_line.size() > 1 ? lexical_cast<unsigned int>(split_line[1]) : 0;

      for (unsigned int vt = 0; vt < num_verts; vt++) {
        getline(inf, line);
        line_num++;
        if (!inf)
          throw read_error(
              str(boost::format("Error reading file %1%, line %2%") % filename % line_num));
        trim(line);
        split(split_line, line, is_any_of(" "), token_compress_on);

        switch (split_line.size()) {
        case 3: // raw
        case 4: // raw with boundary
        {
          point_t point;
          for (int i = 0; i < 3; i++)
            point[i] = lexical_cast<double>(split_line[i]);
          ret_geom.points().push_back(point);
          if (split_line.size() == 4)
            ret_geom.boundary().push_back(lexical_cast<int>(split_line[3]));
        } break;
        case 6: // rawn or rawc
        case 7: // rawn or rawc with boundary
        {
          bool is_rawn = ends_with(filename, ".rawn");
          point_t point;
          for (int i = 0; i < 3; i++)
            point[i] = lexical_cast<double>(split_line[i]);
          ret_geom.points().push_back(point);

          if (is_rawn) {
            vector_t normal;
            for (int i = 3; i < 6; i++)
              normal[i - 3] = lexical_cast<double>(split_line[i]);
            ret_geom.normals().push_back(normal);
          } else {
            color_t color;
            for (int i = 3; i < 6; i++)
              color[i - 3] = lexical_cast<double>(split_line[i]);
            ret_geom.colors().push_back(color);
          }

          if (split_line.size() == 7)
            ret_geom.boundary().push_back(lexical_cast<int>(split_line[6]));
        } break;
        case 9:  // rawnc
        case 10: // rawnc with boundary
        {
          point_t point;
          vector_t normal;
          color_t color;

          for (int i = 0; i < 3; i++)
            point[i - 0] = lexical_cast<double>(split_line[i]);
          ret_geom.points().push_back(point);

          for (int i = 3; i < 6; i++)
            normal[i - 3] = lexical_cast<double>(split_line[i]);
          ret_geom.normals().push_back(normal);

          for (int i = 6; i < 9; i++)
            color[i - 6] = lexical_cast<double>(split_line[i]);
          ret_geom.colors().push_back(color);

          if (split_line.size() == 10)
            ret_geom.boundary().push_back(lexical_cast<int>(split_line[9]));
        } break;
        default: {
          throw read_error(str(boost::format("Not a cvc-raw file (wrong number of tokens: [%1%])") %
                               split_line.size()));
        } break;
        }
      }

      for (unsigned int tri = 0; tri < num_elems; tri++) {
        getline(inf, line);
        line_num++;
        if (!inf)
          throw read_error(
              str(boost::format("Error reading file %1%, line %2%") % filename % line_num));
        trim(line);
        split(split_line, line, is_any_of(" "), token_compress_on);
        switch (split_line.size()) {
        case 2: // lines
        {
          line_t line;
          for (unsigned int i = 0; i < split_line.size(); i++)
            line[i] = lexical_cast<unsigned int>(split_line[i]);
          ret_geom.lines().push_back(line);
        } break;
        case 3: // tris
        {
          tri_t tri;
          for (unsigned int i = 0; i < split_line.size(); i++)
            tri[i] = lexical_cast<unsigned int>(split_line[i]);
          ret_geom.tris().push_back(tri);
        } break;
        case 4: // quads or tetrahedrons
        {
          // if we didn't collect boundary information earlier, then we have quads
          if (ret_geom.boundary().size() != ret_geom.points().size()) {
            quad_t quad;
            for (unsigned int i = 0; i < split_line.size(); i++)
              quad[i] = lexical_cast<unsigned int>(split_line[i]);
            ret_geom.quads().push_back(quad);
          } else {
            unsigned int t[4];
            for (unsigned int i = 0; i < 4; i++)
              t[i] = lexical_cast<unsigned int>(split_line[i]);

            tri_t tet_tris[4];
            tet_tris[0][0] = t[0];
            tet_tris[0][1] = t[2];
            tet_tris[0][2] = t[1];
            tet_tris[1][0] = t[1];
            tet_tris[1][1] = t[2];
            tet_tris[1][2] = t[3];
            tet_tris[2][0] = t[0];
            tet_tris[2][1] = t[3];
            tet_tris[2][2] = t[2];
            tet_tris[3][0] = t[0];
            tet_tris[3][1] = t[1];
            tet_tris[3][2] = t[3];

            for (unsigned int i = 0; i < 4; i++)
              ret_geom.tris().push_back(tet_tris[i]);
          }
        } break;
        case 8: // hexahedrons
        {
          // if we don't have boundary information, then something is wrong!
          if (ret_geom.boundary().size() != ret_geom.points().size())
            throw read_error("Incorrect cvc-raw file: missing boundary info for hex verts");

          unsigned int t[8];
          for (unsigned int i = 0; i < 8; i++)
            t[i] = lexical_cast<unsigned int>(split_line[i]);
          // TODO: investigate hex and tet index ordering... somehow I had to do this different than
          // geoframe
          quad_t hex_quads[6];
#if 0
		    hex_quads[0][0] = t[0]; hex_quads[0][1] = t[3]; hex_quads[0][2] = t[2]; hex_quads[0][3] = t[1];
		    hex_quads[1][0] = t[4]; hex_quads[1][1] = t[5]; hex_quads[1][2] = t[6]; hex_quads[1][3] = t[7];
		    hex_quads[2][0] = t[0]; hex_quads[2][1] = t[4]; hex_quads[2][2] = t[7]; hex_quads[2][3] = t[3];
		    hex_quads[3][0] = t[1]; hex_quads[3][1] = t[2]; hex_quads[3][2] = t[6]; hex_quads[3][3] = t[5];
		    hex_quads[4][0] = t[0]; hex_quads[4][1] = t[1]; hex_quads[4][2] = t[5]; hex_quads[4][3] = t[4];
		    hex_quads[5][0] = t[2]; hex_quads[5][1] = t[3]; hex_quads[5][2] = t[7]; hex_quads[5][3] = t[6];
#endif

          hex_quads[0][0] = t[0];
          hex_quads[0][1] = t[1];
          hex_quads[0][2] = t[2];
          hex_quads[0][3] = t[3];
          hex_quads[1][0] = t[4];
          hex_quads[1][1] = t[5];
          hex_quads[1][2] = t[6];
          hex_quads[1][3] = t[7];
          hex_quads[2][0] = t[0];
          hex_quads[2][1] = t[4];
          hex_quads[2][2] = t[7];
          hex_quads[2][3] = t[3];
          hex_quads[3][0] = t[1];
          hex_quads[3][1] = t[2];
          hex_quads[3][2] = t[6];
          hex_quads[3][3] = t[5];
          hex_quads[4][0] = t[0];
          hex_quads[4][1] = t[1];
          hex_quads[4][2] = t[5];
          hex_quads[4][3] = t[4];
          hex_quads[5][0] = t[2];
          hex_quads[5][1] = t[3];
          hex_quads[5][2] = t[7];
          hex_quads[5][3] = t[6];

          for (unsigned int i = 0; i < 6; i++)
            ret_geom.quads().push_back(hex_quads[i]);
        } break;
        default: {
          throw read_error(str(boost::format("Not a cvc-raw file {num components found: %1%}") %
                               split_line.size()));
        } break;
        }
      }
    } catch (std::exception &e) {
      throw read_error(
          str(boost::format("Error reading file %1%, line %2%, contents: '%3%', reason: %4%") %
              filename % line_num % line % string(e.what())));
    }

    // adjust indices if they start from 1 rather than 0.  (i.e. search for a 0 in each index
    // list. If there are no zeros, decrement each index by one)
#ifdef CVC_GEOMETRY_CORRECT_INDEX_START
    {
      bool lines_has_zero = false;
      bool tris_has_zero = false;
      bool quads_has_zero = false;

      for (typename lines_t::const_iterator i = ret_geom.lines().begin();
           i != ret_geom.lines().end(); i++)
        if (find(i->begin(), i->end(), 0) != i->end()) {
          lines_has_zero = true;
          break;
        }

      for (typename tris_t::const_iterator i = ret_geom.tris().begin(); i != ret_geom.tris().end();
           i++)
        if (find(i->begin(), i->end(), 0) != i->end()) {
          tris_has_zero = true;
          break;
        }

      for (typename quads_t::const_iterator i = ret_geom.quads().begin();
           i != ret_geom.quads().end(); i++)
        if (find(i->begin(), i->end(), 0) != i->end()) {
          quads_has_zero = true;
          break;
        }

      if (!lines_has_zero)
        for (typename lines_t::iterator i = ret_geom.lines().begin(); i != ret_geom.lines().end();
             i++)
          for_each(i->begin(), i->end(), --_1);

      if (!tris_has_zero)
        for (typename tris_t::iterator i = ret_geom.tris().begin(); i != ret_geom.tris().end(); i++)
          for_each(i->begin(), i->end(), --_1);

      if (!quads_has_zero)
        for (typename quads_t::iterator i = ret_geom.quads().begin(); i != ret_geom.quads().end();
             i++)
          for_each(i->begin(), i->end(), --_1);
    }
#endif

    return ret_geom;
  }

  // ----------------
  // cvcraw_io::write
  // ----------------
  // Purpose:
  //   Writes geometry to file.
  // ---- Change History ----
  // 12/27/2013 -- Joe R. -- Adapted from old io.h header in VolumeRover.
  // 01/11/2014 -- Joe R. -- Adapted to work with geometry_file_io.
  virtual void write(const geometry &geom, const std::string &filename) const {
    using namespace std;
    using namespace boost;

    ofstream outf(filename.c_str());
    if (!outf)
      throw write_error(string("Could not open ") + filename);

    unsigned int num_elems;
    if (geom.boundary().size() != geom.points().size()) // if we don't have boundary information
    {
      num_elems = geom.lines().size() > 0   ? geom.lines().size()
                  : geom.tris().size() > 0  ? geom.tris().size()
                  : geom.quads().size() > 0 ? geom.quads().size()
                                            : 0;
    } else {
      num_elems = geom.tris().size() > 0    ? geom.tris().size() / 4
                  : geom.quads().size() > 0 ? geom.quads().size() / 6
                                            : 0;
    }
    outf << geom.points().size() << " " << num_elems << endl;
    if (!outf)
      throw write_error("Could not write number of points or number of tris to file!");

    // arand: 4-21-2011
    // changed to check file extension
    // normals and colors are only printed if .rawn/.rawnc/.rawc
    // have been specified...
    bool haveNormals = (geom.normals().size() == geom.points().size());
    bool printNormals = (ends_with(filename, ".rawn") || ends_with(filename, ".rawnc"));
    bool haveColors = (geom.colors().size() == geom.points().size());
    bool printColors = (ends_with(filename, ".rawc") || ends_with(filename, ".rawnc"));

    if (printNormals && !haveNormals) {
      std::cerr << "WARNING: file with normals requested but not available." << std::endl;
    }

    if (printColors && !haveColors) {
      std::cerr << "WARNING: file with colors requested but not available." << std::endl;
    }

    for (points_t::const_iterator i = geom.points().begin(); i != geom.points().end(); i++) {
      outf << (*i)[0] << " " << (*i)[1] << " " << (*i)[2];

      if (haveNormals && printNormals)
        outf << " " << geom.normals()[std::distance(geom.points().begin(), i)][0] << " "
             << geom.normals()[std::distance(geom.points().begin(), i)][1] << " "
             << geom.normals()[std::distance(geom.points().begin(), i)][2];
      if (haveColors && printColors)
        outf << " " << geom.colors()[std::distance(geom.points().begin(), i)][0] << " "
             << geom.colors()[std::distance(geom.points().begin(), i)][1] << " "
             << geom.colors()[std::distance(geom.points().begin(), i)][2];
      if (geom.boundary().size() == geom.points().size())
        outf << " " << geom.boundary()[std::distance(geom.points().begin(), i)];
      outf << endl;
      if (!outf)
        throw write_error(str(boost::format("Error writing vertex %1%") %
                              std::distance(geom.points().begin(), i)));
    }

    if (geom.lines().size() != 0) {
      for (lines_t::const_iterator i = geom.lines().begin(); i != geom.lines().end(); i++) {
        typedef lines_t::value_type cell_type;
        for (cell_type::const_iterator j = i->begin(); j != i->end(); j++) {
          outf << *j;
          if (boost::next(j) == i->end())
            outf << endl;
          else
            outf << " ";
        }

        if (!outf)
          throw write_error(str(boost::format("Error writing line %1%") %
                                std::distance(geom.lines().begin(), i)));
      }
    } else if (geom.tris().size() != 0) {
      if (geom.boundary().size() !=
          geom.points()
              .size()) // if we don't have boundary info, don't treat these tris as part of a tet
      {
        for (tris_t::const_iterator i = geom.tris().begin(); i != geom.tris().end(); i++) {
          typedef tris_t::value_type cell_type;
          for (cell_type::const_iterator j = i->begin(); j != i->end(); j++) {
            outf << *j;
            if (boost::next(j) == i->end())
              outf << endl;
            else
              outf << " ";
          }

          if (!outf)
            throw write_error(str(boost::format("Error writing triangle %1%") %
                                  std::distance(geom.tris().begin(), i)));
        }
      } else {
        for (unsigned int i = 0; i < geom.tris().size() / 4; i++) {
          outf << geom.tris()[4 * i][0] << " " << geom.tris()[4 * i][1] << " "
               << geom.tris()[4 * i][2] << " " << geom.tris()[4 * i + 1][2] << endl;
          if (!outf)
            throw write_error(str(boost::format("Error writing tetrahedron %1%") % i));
        }
      }
    } else if (geom.quads().size() != 0) {
      if (geom.boundary().size() !=
          geom.points()
              .size()) // if we don't have boundary info, don't tread these quads as part of a hexa
      {
        for (quads_t::const_iterator i = geom.quads().begin(); i != geom.quads().end(); i++) {
          typedef quads_t::value_type cell_type;
          for (cell_type::const_iterator j = i->begin(); j != i->end(); j++) {
            outf << *j;
            if (boost::next(j) == i->end())
              outf << endl;
            else
              outf << " ";
          }

          if (!outf)
            throw write_error(str(boost::format("Error writing quad %1%") %
                                  std::distance(geom.quads().begin(), i)));
        }
      } else {
        for (unsigned int i = 0; i < geom.quads().size() / 6; i++) {
#if 0
		  outf << geom.quads()[6*i][0] << " " << geom.quads()[6*i][1] << " "
		       << geom.quads()[6*i][2] << " " << geom.quads()[6*i][3] << " "
		       << geom.quads()[6*i+1][1] << " " << geom.quads()[6*i+1][0] << " "
		       << geom.quads()[6*i+1][3] << " " << geom.quads()[6*i+1][2] << endl;
#endif
          outf << geom.quads()[6 * i][0] << " " << geom.quads()[6 * i][1] << " "
               << geom.quads()[6 * i][2] << " " << geom.quads()[6 * i][3] << " "
               << geom.quads()[6 * i + 1][0] << " " << geom.quads()[6 * i + 1][1] << " "
               << geom.quads()[6 * i + 1][2] << " " << geom.quads()[6 * i + 1][3] << endl;

          if (!outf)
            throw write_error(str(boost::format("Error writing hexahedron %1%") % i));
        }
      }
    }
  }

protected:
  std::string _id;
  extension_list _extensions;
};
} // namespace cvc

namespace cvc {
void register_cvcraw_io(app & /*ctx*/) {
  geometry_file_io::insert_handler(geometry_file_io::ptr(new cvcraw_io));
}

void register_cvcraw_io() {
  geometry_file_io::insert_handler(geometry_file_io::ptr(new cvcraw_io));
}
} // namespace cvc
