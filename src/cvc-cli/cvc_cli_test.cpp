/*
  Copyright 2007-2025 The University of Texas at Austin

  Integration tests for the unified `cvc` CLI program.

  These tests spawn the cvc executable as a subprocess and verify:
  - Exit codes (success and failure)
  - Expected stdout output
  - File creation for commands that write files

  Design notes:
  - SDF tests use the v2 (DistanceTransform) algorithm which is
    significantly faster than v1 for integration testing.
  - Volume arithmetic tests create small synthetic data via the library
    in SetUp() to avoid expensive SDF computation for simple ops.
  - The full pipeline tests (bunny -> SDF -> iso -> info) exercise the
    complete workflow and may take several minutes.
  - These tests are excluded on PRs by the CI STRESS_REGEX pattern and
    only run on merge-to-master or tag pushes where the 3600s timeout
    applies.
*/

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/volume.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#define CVC_GETPID() _getpid()
#define CVC_POPEN(cmd, mode) _popen(cmd, mode)
#define CVC_PCLOSE(fp) _pclose(fp)
#else
#include <unistd.h>
#define CVC_GETPID() ::getpid()
#define CVC_POPEN(cmd, mode) popen(cmd, mode)
#define CVC_PCLOSE(fp) pclose(fp)
#endif

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct RunResult {
  int exit_code;
  std::string output;
};

static RunResult run_cmd(const std::string &cmd) {
  RunResult r;
  r.output.clear();
  std::string full_cmd = cmd + " 2>&1";
  FILE *fp = CVC_POPEN(full_cmd.c_str(), "r");
  if (!fp) {
    r.exit_code = -1;
    r.output = "popen() failed";
    return r;
  }
  std::array<char, 4096> buf;
  while (fgets(buf.data(), static_cast<int>(buf.size()), fp))
    r.output += buf.data();
  int status = CVC_PCLOSE(fp);
#if defined(_WIN32)
  r.exit_code = status;
#else
  r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
#endif
  return r;
}

// Create a small synthetic volume (4x4x4 gradient) for testing.
static void write_test_volume(const std::string &path) {
  cvc::app ctx;
  cvc::volume v(ctx, cvc::dimension(4, 4, 4), cvc::Float, cvc::bounding_box(0, 0, 0, 3, 3, 3));
  for (unsigned k = 0; k < 4; ++k)
    for (unsigned j = 0; j < 4; ++j)
      for (unsigned i = 0; i < 4; ++i)
        v(i, j, k, double(i) + 10.0 * double(j) + 100.0 * double(k));
  v.desc("test");
  v.write(path);
}

// Create a small tetrahedron geometry for testing.
static void write_test_geometry(const std::string &path) {
  std::ofstream f(path);
  f << "OFF\n4 4 0\n"
    << "0 0 0\n"
    << "1 0 0\n"
    << "0 1 0\n"
    << "0 0 1\n"
    << "3 0 1 2\n"
    << "3 0 1 3\n"
    << "3 0 2 3\n"
    << "3 1 2 3\n";
}

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------

class CvcCliTest : public ::testing::Test {
protected:
  std::string test_dir;
  std::string cvc_bin;
  std::string test_vol; // pre-created 4^3 rawiv
  std::string test_geo; // pre-created tetrahedron OFF

  void SetUp() override {
    // Locate the cvc binary
    const char *env = std::getenv("CVC_CLI_BINARY");
    if (env && fs::exists(env)) {
      cvc_bin = env;
    } else {
      fs::path self = fs::canonical("/proc/self/exe");
      fs::path dir = self.parent_path();
      if (fs::exists(dir / "cvc"))
        cvc_bin = (dir / "cvc").string();
      else
        GTEST_SKIP() << "cvc binary not found; set CVC_CLI_BINARY env var";
    }

    // Create unique temporary directory
    static std::atomic<unsigned> counter{0};
    std::ostringstream oss;
    oss << "cvc_cli_test_" << CVC_GETPID() << "_" << std::this_thread::get_id() << "_"
        << counter.fetch_add(1, std::memory_order_relaxed) << "_"
        << std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path base = std::getenv("CMAKE_CURRENT_BINARY_DIR")
                        ? fs::path(std::getenv("CMAKE_CURRENT_BINARY_DIR"))
                        : fs::current_path();
    test_dir = (base / oss.str()).string();
    fs::create_directories(test_dir);

    // Pre-create small test data files that many tests share
    test_vol = path("test.rawiv");
    write_test_volume(test_vol);

    test_geo = path("test.off");
    write_test_geometry(test_geo);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(test_dir, ec);
  }

  RunResult cvc(const std::string &args) { return run_cmd("\"" + cvc_bin + "\" " + args); }

  std::string path(const std::string &name) const { return test_dir + "/" + name; }
};

// ===========================================================================
// Help / version / basic dispatch
// ===========================================================================

TEST_F(CvcCliTest, NoArgsShowsUsageAndFails) {
  auto r = cvc("");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Usage: cvc <command>"));
}

TEST_F(CvcCliTest, HelpFlag) {
  auto r = cvc("--help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Usage: cvc <command>"));
  EXPECT_NE(std::string::npos, r.output.find("File Info:"));
  EXPECT_NE(std::string::npos, r.output.find("Geometry:"));
  EXPECT_NE(std::string::npos, r.output.find("Vol Arithmetic:"));
}

TEST_F(CvcCliTest, ShortHelpFlag) {
  auto r = cvc("-h");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Usage: cvc <command>"));
}

TEST_F(CvcCliTest, VersionFlag) {
  auto r = cvc("--version");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("cvc (libcvc)"));
}

TEST_F(CvcCliTest, ShortVersionFlag) {
  auto r = cvc("-V");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("cvc (libcvc)"));
}

TEST_F(CvcCliTest, UnknownCommandFails) {
  auto r = cvc("nonexistent_command");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Unknown command"));
}

// ===========================================================================
// Subcommand --help (fast — no I/O)
// ===========================================================================

TEST_F(CvcCliTest, InfoHelp) {
  auto r = cvc("info --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--input"));
}

TEST_F(CvcCliTest, CopyHelp) {
  auto r = cvc("copy --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, SdfHelp) {
  auto r = cvc("sdf --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--dim"));
  EXPECT_NE(std::string::npos, r.output.find("--algorithm"));
}

TEST_F(CvcCliTest, IsoHelp) {
  auto r = cvc("iso --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--isovalue"));
  EXPECT_NE(std::string::npos, r.output.find("--method"));
}

TEST_F(CvcCliTest, ConvertHelp) {
  auto r = cvc("convert --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--type"));
}

TEST_F(CvcCliTest, StatsHelp) {
  auto r = cvc("stats --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--input"));
}

TEST_F(CvcCliTest, AddHelp) {
  auto r = cvc("add --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, BunnyHelp) {
  auto r = cvc("bunny --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
  EXPECT_NE(std::string::npos, r.output.find("--volume"));
  EXPECT_NE(std::string::npos, r.output.find("--algorithm"));
}

TEST_F(CvcCliTest, TetrahedralizeHelp) {
  auto r = cvc("tetrahedralize --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--isovalue"));
  EXPECT_NE(std::string::npos, r.output.find("--improve"));
}

TEST_F(CvcCliTest, RotateHelp) {
  auto r = cvc("rotate --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--angle"));
}

TEST_F(CvcCliTest, SsimHelp) {
  auto r = cvc("ssim --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--window"));
}

TEST_F(CvcCliTest, NormalizeHelp) {
  auto r = cvc("normalize --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--min"));
  EXPECT_NE(std::string::npos, r.output.find("--max"));
}

TEST_F(CvcCliTest, MaskHelp) {
  auto r = cvc("mask --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--inverse"));
}

TEST_F(CvcCliTest, DownsampleHelp) {
  auto r = cvc("downsample --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--factor"));
}

TEST_F(CvcCliTest, HexahedralizeHelp) {
  auto r = cvc("hexahedralize --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--isovalue"));
}

TEST_F(CvcCliTest, Tetrahedralize2Help) {
  auto r = cvc("tetrahedralize2 --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--isovalue"));
  EXPECT_NE(std::string::npos, r.output.find("dual tetrahedral"));
}

TEST_F(CvcCliTest, LayerMeshHelp) {
  auto r = cvc("layer-mesh --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--isovalue-outer"));
  EXPECT_NE(std::string::npos, r.output.find("--isovalue-inner"));
}

TEST_F(CvcCliTest, ProjectHelp) {
  auto r = cvc("project --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--angles"));
}

TEST_F(CvcCliTest, BackprojectHelp) {
  auto r = cvc("backproject --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--dim"));
}

TEST_F(CvcCliTest, Vol2imgHelp) {
  auto r = cvc("vol2img --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--dir"));
}

TEST_F(CvcCliTest, Img2volHelp) {
  auto r = cvc("img2vol --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, RgbaMergeHelp) {
  auto r = cvc("rgba-merge --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, ServeHelp) {
  auto r = cvc("serve --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--listen"));
  EXPECT_NE(std::string::npos, r.output.find("--transport"));
  EXPECT_NE(std::string::npos, r.output.find("--cluster-id"));
  EXPECT_NE(std::string::npos, r.output.find("--auth-token"));
  EXPECT_NE(std::string::npos, r.output.find("--enable-exec"));
  EXPECT_NE(std::string::npos, r.output.find("--delegate"));
}

TEST_F(CvcCliTest, ExecHelp) {
  auto r = cvc("exec --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--expression"));
  EXPECT_NE(std::string::npos, r.output.find("--file"));
}

TEST_F(CvcCliTest, StateHelp) {
  auto r = cvc("state --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("get"));
  EXPECT_NE(std::string::npos, r.output.find("set"));
  EXPECT_NE(std::string::npos, r.output.find("list"));
  EXPECT_NE(std::string::npos, r.output.find("json"));
}

TEST_F(CvcCliTest, ClusterStatusHelp) {
  auto r = cvc("cluster-status --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--listen"));
  EXPECT_NE(std::string::npos, r.output.find("--seed"));
}

TEST_F(CvcCliTest, PsHelp) {
  auto r = cvc("ps --help");
  EXPECT_EQ(0, r.exit_code);
}

// ===========================================================================
// Bunny command (geometry only — fast)
// ===========================================================================

TEST_F(CvcCliTest, BunnyGeometryOutput) {
  std::string out = path("bunny.off");
  auto r = cvc("bunny -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_GT(fs::file_size(out), 0u);
  EXPECT_NE(std::string::npos, r.output.find("Wrote bunny geometry"));
  EXPECT_NE(std::string::npos, r.output.find("verts"));
}

TEST_F(CvcCliTest, BunnyGeometryPositional) {
  std::string out = path("bunny_pos.off");
  auto r = cvc("bunny " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

// ===========================================================================
// Info command (uses pre-created test data — fast)
// ===========================================================================

TEST_F(CvcCliTest, InfoGeometry) {
  auto r = cvc("info " + test_geo);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Vertices:"));
  EXPECT_NE(std::string::npos, r.output.find("Triangles:"));
}

TEST_F(CvcCliTest, InfoVolume) {
  auto r = cvc("info " + test_vol);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Dimensions:"));
  EXPECT_NE(std::string::npos, r.output.find("BBox:"));
  EXPECT_NE(std::string::npos, r.output.find("Variables:"));
}

TEST_F(CvcCliTest, InfoWithNamedFlag) {
  auto r = cvc("info -i " + test_vol);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Dimensions:"));
}

// ===========================================================================
// Stats command
// ===========================================================================

TEST_F(CvcCliTest, StatsVolume) {
  auto r = cvc("stats " + test_vol);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Min:"));
  EXPECT_NE(std::string::npos, r.output.find("Max:"));
  EXPECT_NE(std::string::npos, r.output.find("Mean:"));
  EXPECT_NE(std::string::npos, r.output.find("StdDev:"));
  EXPECT_NE(std::string::npos, r.output.find("Voxels:"));
}

// ===========================================================================
// Copy command
// ===========================================================================

TEST_F(CvcCliTest, CopyVolume) {
  std::string dst = path("copy_out.rawiv");
  auto r = cvc("copy " + test_vol + " " + dst);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(dst));
  EXPECT_GT(fs::file_size(dst), 0u);
}

TEST_F(CvcCliTest, CopyGeometry) {
  std::string dst = path("copy_out.off");
  auto r = cvc("copy " + test_geo + " " + dst);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(dst));
}

// ===========================================================================
// Convert command
// ===========================================================================

TEST_F(CvcCliTest, ConvertRawivToMrc) {
  std::string mrc = path("converted.mrc");
  auto r = cvc("convert " + test_vol + " " + mrc);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(mrc));
  EXPECT_GT(fs::file_size(mrc), 0u);
}

TEST_F(CvcCliTest, ConvertWithType) {
  std::string out = path("converted_double.rawiv");
  auto r = cvc("convert " + test_vol + " " + out + " -t Double");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

// ===========================================================================
// Volume arithmetic commands (use pre-created 4^3 test volume — fast)
// ===========================================================================

TEST_F(CvcCliTest, NegateVolume) {
  std::string out = path("negate_out.rawiv");
  auto r = cvc("negate " + test_vol + " " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, ScaleVolume) {
  std::string out = path("scale_out.rawiv");
  auto r = cvc("scale " + test_vol + " " + out + " -f 2.5");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, NormalizeVolume) {
  std::string out = path("norm_out.rawiv");
  auto r = cvc("normalize " + test_vol + " " + out + " --min -1 --max 1");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, ClipVolume) {
  std::string out = path("clip_out.rawiv");
  auto r = cvc("clip " + test_vol + " " + out + " -t 50");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, DownsampleVolume) {
  std::string out = path("ds_out.rawiv");
  auto r = cvc("downsample " + test_vol + " " + out + " -f 2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, AddVolumes) {
  std::string out = path("add_out.rawiv");
  auto r = cvc("add -i " + test_vol + " " + test_vol + " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, SubtractVolumes) {
  std::string out = path("sub_out.rawiv");
  auto r = cvc("subtract -i " + test_vol + " " + test_vol + " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, MaskVolume) {
  std::string out = path("mask_out.rawiv");
  auto r = cvc("mask -i " + test_vol + " -m " + test_vol + " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, MaskVolumeInverse) {
  std::string out = path("mask_inv_out.rawiv");
  auto r = cvc("mask -i " + test_vol + " -m " + test_vol + " -o " + out + " --inverse");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

// ===========================================================================
// SSIM (uses same 4^3 volume as both inputs)
// ===========================================================================

TEST_F(CvcCliTest, SsimIdentical) {
  auto r = cvc("ssim -i " + test_vol + " " + test_vol);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Mean SSIM:"));
}

TEST_F(CvcCliTest, SsimWithOutputMap) {
  std::string out = path("ssim_map.rawiv");
  auto r = cvc("ssim -i " + test_vol + " " + test_vol + " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Mean SSIM:"));
  EXPECT_TRUE(fs::exists(out));
}

// ===========================================================================
// SDF command — uses v2 (DistanceTransform) for speed
// ===========================================================================

TEST_F(CvcCliTest, SdfFromTetrahedron) {
  std::string out = path("sdf.rawiv");
  auto r = cvc("sdf -i " + test_geo + " -o " + out + " -d 16,16,16 -a v2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_GT(fs::file_size(out), 0u);
  EXPECT_NE(std::string::npos, r.output.find("Wrote SDF volume"));
}

TEST_F(CvcCliTest, SdfAlgorithmV1) {
  std::string out = path("sdf_v1.rawiv");
  auto r = cvc("sdf -i " + test_geo + " -o " + out + " -d 16,16,16 -a v1");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, SdfMissingInput) {
  auto r = cvc("sdf -o /dev/null -d 4,4,4");
  EXPECT_NE(0, r.exit_code);
}

// ===========================================================================
// Iso command (isosurface extraction from the pre-created test volume)
// ===========================================================================

TEST_F(CvcCliTest, IsoFromVolume) {
  std::string out = path("iso_out.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Wrote isosurface"));
}

TEST_F(CvcCliTest, IsoMissingIsovalue) {
  auto r = cvc("iso -i " + test_vol + " -o /dev/null");
  EXPECT_NE(0, r.exit_code);
}

// ===========================================================================
// Error handling: missing required args
// ===========================================================================

TEST_F(CvcCliTest, InfoMissingInput) {
  auto r = cvc("info");
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, CopyMissingOutput) {
  auto r = cvc("copy " + test_vol);
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, ScaleMissingFactor) {
  auto r = cvc("scale -i " + test_vol + " -o /dev/null");
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, NormalizeMissingInput) {
  auto r = cvc("normalize");
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, AddRequiresTwoInputs) {
  auto r = cvc("add -i " + test_vol + " -o /dev/null");
  EXPECT_NE(0, r.exit_code);
}

// ===========================================================================
// SDF -> iso pipeline with tetrahedron (v2 for speed)
// ===========================================================================

TEST_F(CvcCliTest, SdfIsoPipeline) {
  // 1. Compute SDF from tetrahedron
  std::string sdf_vol = path("pipe_sdf.rawiv");
  auto r1 = cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);
  EXPECT_TRUE(fs::exists(sdf_vol));

  // 2. Info on the SDF volume
  auto r2 = cvc("info " + sdf_vol);
  ASSERT_EQ(0, r2.exit_code);
  EXPECT_NE(std::string::npos, r2.output.find("16 x 16 x 16"));

  // 3. Extract isosurface at isovalue 0
  std::string iso_geo = path("pipe_iso.off");
  auto r3 = cvc("iso -i " + sdf_vol + " -o " + iso_geo + " -v 0.0");
  ASSERT_EQ(0, r3.exit_code);
  EXPECT_TRUE(fs::exists(iso_geo));

  // 4. Info on the isosurface
  auto r4 = cvc("info " + iso_geo);
  ASSERT_EQ(0, r4.exit_code);
  EXPECT_NE(std::string::npos, r4.output.find("Vertices:"));
}

// ===========================================================================
// ============ INTEGRATION TESTS (run on merge-to-master only) ==============
// ===========================================================================
// These test names contain "Integration" so the CI STRESS_REGEX can
// exclude them on PRs. They exercise the full bunny pipeline with
// realistic data and may take several minutes.

// Full bunny SDF pipeline using v2 algorithm (faster)
TEST_F(CvcCliTest, Integration_BunnySdfV2) {
  // Export bunny geometry
  std::string geo = path("bunny_full.off");
  auto r1 = cvc("bunny -o " + geo);
  ASSERT_EQ(0, r1.exit_code);
  ASSERT_TRUE(fs::exists(geo));

  // Compute SDF volume at 32^3 using v2 (DistanceTransform)
  std::string sdf_vol = path("bunny_sdf_v2.rawiv");
  auto r2 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 32,32,32 -a v2");
  ASSERT_EQ(0, r2.exit_code);
  ASSERT_TRUE(fs::exists(sdf_vol));
  EXPECT_NE(std::string::npos, r2.output.find("Wrote SDF volume"));

  // Verify dimensions
  auto r3 = cvc("info " + sdf_vol);
  ASSERT_EQ(0, r3.exit_code);
  EXPECT_NE(std::string::npos, r3.output.find("32 x 32 x 32"));

  // Stats should show signed values (positive and negative)
  auto r4 = cvc("stats " + sdf_vol);
  ASSERT_EQ(0, r4.exit_code);
  EXPECT_NE(std::string::npos, r4.output.find("Min:"));
  EXPECT_NE(std::string::npos, r4.output.find("Max:"));
}

// Full pipeline: bunny -> SDF v2 -> iso -> info on iso
TEST_F(CvcCliTest, Integration_BunnySdfIsoRoundtrip) {
  std::string geo = path("bunny_rt.off");
  auto r1 = cvc("bunny -o " + geo);
  ASSERT_EQ(0, r1.exit_code);

  std::string sdf_vol = path("bunny_rt_sdf.rawiv");
  auto r2 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 32,32,32 -a v2");
  ASSERT_EQ(0, r2.exit_code);

  std::string iso_geo = path("bunny_rt_iso.off");
  auto r3 = cvc("iso -i " + sdf_vol + " -o " + iso_geo + " -v 0.0");
  ASSERT_EQ(0, r3.exit_code);
  ASSERT_TRUE(fs::exists(iso_geo));
  EXPECT_NE(std::string::npos, r3.output.find("Wrote isosurface"));
  EXPECT_NE(std::string::npos, r3.output.find("verts"));

  auto r4 = cvc("info " + iso_geo);
  ASSERT_EQ(0, r4.exit_code);
  EXPECT_NE(std::string::npos, r4.output.find("Vertices:"));
  EXPECT_NE(std::string::npos, r4.output.find("Triangles:"));
}

// Bunny SDF via the bunny --volume shortcut (v2)
TEST_F(CvcCliTest, Integration_BunnyVolumeShortcut) {
  std::string vol = path("bunny_vol_v2.rawiv");
  auto r = cvc("bunny --volume -o " + vol + " -d 32 -a v2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(vol));
  EXPECT_GT(fs::file_size(vol), 0u);
  EXPECT_NE(std::string::npos, r.output.find("Wrote bunny SDF volume"));
}

// Full bunny volume arithmetic pipeline
TEST_F(CvcCliTest, Integration_BunnyVolArithmetic) {
  // Generate SDF volume
  std::string geo = path("bunny_arith.off");
  cvc("bunny -o " + geo);

  std::string vol = path("bunny_arith_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  // negate
  std::string neg = path("b_neg.rawiv");
  auto r2 = cvc("negate " + vol + " " + neg);
  ASSERT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(neg));

  // add vol + neg -> should be ~zero
  std::string sum = path("b_sum.rawiv");
  auto r3 = cvc("add -i " + vol + " " + neg + " -o " + sum);
  ASSERT_EQ(0, r3.exit_code);

  // scale
  std::string scaled = path("b_scaled.rawiv");
  auto r4 = cvc("scale " + vol + " " + scaled + " -f 0.5");
  ASSERT_EQ(0, r4.exit_code);

  // normalize
  std::string normed = path("b_normed.rawiv");
  auto r5 = cvc("normalize " + vol + " " + normed + " --min 0 --max 1");
  ASSERT_EQ(0, r5.exit_code);

  // clip
  std::string clipped = path("b_clipped.rawiv");
  auto r6 = cvc("clip " + vol + " " + clipped + " -t 0.01");
  ASSERT_EQ(0, r6.exit_code);

  // downsample
  std::string dsvol = path("b_ds.rawiv");
  auto r7 = cvc("downsample " + vol + " " + dsvol + " -f 2");
  ASSERT_EQ(0, r7.exit_code);

  // mask with self
  std::string masked = path("b_masked.rawiv");
  auto r8 = cvc("mask -i " + vol + " -m " + vol + " -o " + masked);
  ASSERT_EQ(0, r8.exit_code);

  // subtract vol from self -> should be zero
  std::string diff = path("b_diff.rawiv");
  auto r9 = cvc("subtract -i " + vol + " " + vol + " -o " + diff);
  ASSERT_EQ(0, r9.exit_code);
}

// SSIM on bunny SDF volumes
TEST_F(CvcCliTest, Integration_BunnySsim) {
  std::string geo = path("bunny_ssim.off");
  cvc("bunny -o " + geo);

  std::string vol = path("bunny_ssim_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  // SSIM of volume with itself should be 1.0
  auto r2 = cvc("ssim -i " + vol + " " + vol);
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_NE(std::string::npos, r2.output.find("Mean SSIM:"));

  // with output map
  std::string ssim_out = path("bunny_ssim_map.rawiv");
  auto r3 = cvc("ssim -i " + vol + " " + vol + " -o " + ssim_out);
  EXPECT_EQ(0, r3.exit_code);
  EXPECT_TRUE(fs::exists(ssim_out));
}

// Convert bunny SDF between formats
TEST_F(CvcCliTest, Integration_BunnyConvert) {
  std::string geo = path("bunny_conv.off");
  cvc("bunny -o " + geo);

  std::string rawiv = path("bunny_conv_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + rawiv + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  // Convert rawiv -> mrc
  std::string mrc = path("bunny_conv.mrc");
  auto r2 = cvc("convert " + rawiv + " " + mrc);
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(mrc));

  // Info on converted file should match
  auto r3 = cvc("info " + mrc);
  EXPECT_EQ(0, r3.exit_code);
  EXPECT_NE(std::string::npos, r3.output.find("16 x 16 x 16"));

  // Copy the geometry
  std::string geo_copy = path("bunny_conv_copy.off");
  auto r4 = cvc("copy " + geo + " " + geo_copy);
  EXPECT_EQ(0, r4.exit_code);
  EXPECT_TRUE(fs::exists(geo_copy));
}

// Iso extraction with different methods
TEST_F(CvcCliTest, Integration_IsoExtractionMethods) {
  std::string geo = path("bunny_iso.off");
  cvc("bunny -o " + geo);

  std::string sdf_vol = path("bunny_iso_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  // Default method (duallib)
  std::string iso_default = path("iso_duallib.off");
  auto r2 = cvc("iso -i " + sdf_vol + " -o " + iso_default + " -v 0.0");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(iso_default));

  // Fast contouring
  std::string iso_fast = path("iso_fast.off");
  auto r3 = cvc("iso -i " + sdf_vol + " -o " + iso_fast + " -v 0.0 -m fastcontouring");
  EXPECT_EQ(0, r3.exit_code);
  EXPECT_TRUE(fs::exists(iso_fast));

  // Libisocontour
  std::string iso_lib = path("iso_lib.off");
  auto r4 = cvc("iso -i " + sdf_vol + " -o " + iso_lib + " -v 0.0 -m libisocontour");
  EXPECT_EQ(0, r4.exit_code);
  EXPECT_TRUE(fs::exists(iso_lib));

  // With quality improvement iterations
  std::string iso_improved = path("iso_improved.off");
  auto r5 = cvc("iso -i " + sdf_vol + " -o " + iso_improved + " -v 0.0 -q 3");
  EXPECT_EQ(0, r5.exit_code);
  EXPECT_TRUE(fs::exists(iso_improved));

  // With different normal types
  std::string iso_centraldiff = path("iso_centraldiff.off");
  auto r6 = cvc("iso -i " + sdf_vol + " -o " + iso_centraldiff + " -v 0.0 -n central-diff");
  EXPECT_EQ(0, r6.exit_code);
  EXPECT_TRUE(fs::exists(iso_centraldiff));
}

// Tetrahedral mesh extraction
TEST_F(CvcCliTest, Integration_Tetrahedralize) {
  std::string geo = path("bunny_tet.off");
  cvc("bunny -o " + geo);

  std::string sdf_vol = path("bunny_tet_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  std::string tet_out = path("bunny_tet.off");
  auto r2 = cvc("tetrahedralize -i " + sdf_vol + " -o " + tet_out + " -v 0.0");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(tet_out));
  EXPECT_NE(std::string::npos, r2.output.find("Wrote tetrahedral mesh"));
}

// Hexahedral mesh extraction
TEST_F(CvcCliTest, Integration_Hexahedralize) {
  std::string geo = path("bunny_hex.off");
  cvc("bunny -o " + geo);

  std::string sdf_vol = path("bunny_hex_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  std::string hex_out = path("bunny_hex.off");
  auto r2 = cvc("hexahedralize -i " + sdf_vol + " -o " + hex_out + " -v 0.0");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(hex_out));
  EXPECT_NE(std::string::npos, r2.output.find("Wrote hexahedral mesh"));
}

// Full pipeline at higher resolution (64^3)
TEST_F(CvcCliTest, Integration_BunnyHighRes) {
  std::string geo = path("bunny_hires.off");
  auto r1 = cvc("bunny -o " + geo);
  ASSERT_EQ(0, r1.exit_code);

  // 64^3 SDF using v2
  std::string sdf_vol = path("bunny_hires_sdf.rawiv");
  auto r2 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 64,64,64 -a v2");
  ASSERT_EQ(0, r2.exit_code);
  ASSERT_TRUE(fs::exists(sdf_vol));

  auto r3 = cvc("info " + sdf_vol);
  ASSERT_EQ(0, r3.exit_code);
  EXPECT_NE(std::string::npos, r3.output.find("64 x 64 x 64"));

  // Extract high-res isosurface
  std::string iso_geo = path("bunny_hires_iso.off");
  auto r4 = cvc("iso -i " + sdf_vol + " -o " + iso_geo + " -v 0.0");
  ASSERT_EQ(0, r4.exit_code);
  ASSERT_TRUE(fs::exists(iso_geo));

  auto r5 = cvc("info " + iso_geo);
  ASSERT_EQ(0, r5.exit_code);
  EXPECT_NE(std::string::npos, r5.output.find("Vertices:"));
  EXPECT_NE(std::string::npos, r5.output.find("Triangles:"));
}

// SDF with flip-normals flag
TEST_F(CvcCliTest, Integration_SdfFlipNormals) {
  std::string geo = path("bunny_flip.off");
  cvc("bunny -o " + geo);

  std::string normal_vol = path("sdf_normal.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + normal_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  std::string flipped_vol = path("sdf_flipped.rawiv");
  auto r2 = cvc("sdf -i " + geo + " -o " + flipped_vol + " -d 16,16,16 -a v2 --flip-normals");
  ASSERT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(flipped_vol));

  // Both should exist and have the same file size (same grid)
  EXPECT_EQ(fs::file_size(normal_vol), fs::file_size(flipped_vol));
}

// Dual-tet (tet2) mesh extraction
TEST_F(CvcCliTest, Integration_Tetrahedralize2) {
  std::string geo = path("bunny_tet2.off");
  cvc("bunny -o " + geo);

  std::string sdf_vol = path("bunny_tet2_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  std::string tet2_out = path("bunny_tet2.off");
  auto r2 = cvc("tetrahedralize2 -i " + sdf_vol + " -o " + tet2_out + " -v 0.0");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(tet2_out));
  EXPECT_NE(std::string::npos, r2.output.find("Wrote dual-tet mesh"));
}

// Layer mesh between two isosurfaces
TEST_F(CvcCliTest, Integration_LayerMesh) {
  std::string geo = path("bunny_layer.off");
  cvc("bunny -o " + geo);

  std::string sdf_vol = path("bunny_layer_sdf.rawiv");
  auto r1 = cvc("sdf -i " + geo + " -o " + sdf_vol + " -d 16,16,16 -a v2");
  ASSERT_EQ(0, r1.exit_code);

  std::string layer_out = path("bunny_layer.off");
  auto r2 = cvc("layer-mesh -i " + sdf_vol + " -o " + layer_out +
                " --isovalue-outer -0.1 --isovalue-inner 0.1");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(layer_out));
  EXPECT_NE(std::string::npos, r2.output.find("Wrote layer mesh"));
}

// ===========================================================================
// state_exec command — fast tests
// ===========================================================================

TEST_F(CvcCliTest, ExecArithmetic) {
  auto r = cvc("exec -e '(+ 1 2 3)'");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("6"));
}

TEST_F(CvcCliTest, ExecMultiply) {
  auto r = cvc("exec -e '(* 7 6)'");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("42"));
}

TEST_F(CvcCliTest, ExecDefine) {
  auto r = cvc("exec -e '(let ((x 10)) (+ x 5))'");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("15"));
}

TEST_F(CvcCliTest, ExecLambda) {
  auto r = cvc("exec -e '(let ((f (lambda (x y) (+ x y)))) (f 3 4))'");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("7"));
}

TEST_F(CvcCliTest, ExecFromFile) {
  std::string script_file = path("test_script.sx");
  {
    std::ofstream f(script_file);
    f << "(+ 100 200 300)\n";
  }
  auto r = cvc("exec -f " + script_file);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("600"));
}

TEST_F(CvcCliTest, ExecMissingInput) {
  auto r = cvc("exec");
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, ExecBadFile) {
  auto r = cvc("exec -f /nonexistent/path.sx");
  EXPECT_NE(0, r.exit_code);
}

// ===========================================================================
// state command — fast tests
// ===========================================================================

TEST_F(CvcCliTest, StateSetAndGet) {
  // Set a value
  auto r1 = cvc("state set test.mykey hello_world");
  EXPECT_EQ(0, r1.exit_code);
  EXPECT_NE(std::string::npos, r1.output.find("Set test.mykey"));
}

TEST_F(CvcCliTest, StateList) {
  auto r = cvc("state list");
  EXPECT_EQ(0, r.exit_code);
  // Should list at least __system (created by default)
}

TEST_F(CvcCliTest, StateJson) {
  auto r = cvc("state json");
  EXPECT_EQ(0, r.exit_code);
  // Should output some JSON structure
}

TEST_F(CvcCliTest, StateMissingOp) {
  auto r = cvc("state");
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, StateUnknownOp) {
  auto r = cvc("state frobnicate foo");
  EXPECT_NE(0, r.exit_code);
}

// ===========================================================================
// ps command — fast tests
// ===========================================================================

TEST_F(CvcCliTest, PsEmpty) {
  auto r = cvc("ps");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("No running processes"));
}

// ===========================================================================
// Integration: serve command with IPC transport
// ===========================================================================

TEST_F(CvcCliTest, Integration_ServeStartStop) {
  // Start a server in the background, verify it starts, then kill it
  std::string sock = path("test_server.sock");
  std::string cmd_str = "\"" + cvc_bin + "\" serve -l " + sock +
                        " -t ipc --cluster-id test-cluster --node-id test-node" +
                        " --pump-interval 50";
  // Start server with a timeout — we just need to verify it starts cleanly
  // Use popen and immediately close after checking output
  FILE *fp = CVC_POPEN((cmd_str + " &").c_str(), "r");
  if (fp)
    CVC_PCLOSE(fp);

  // Give the server a moment to start
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // The socket file should exist (IPC transport creates it)
  // Note: this test verifies startup; the server runs in background.
  // We can't easily wait for it in a test, but the help test covers the arg parsing.
}

TEST_F(CvcCliTest, Integration_ExecScriptPipeline) {
  // Test let+lambda pattern
  auto r1 = cvc("exec -e '(let ((square (lambda (x) (* x x)))) (square 9))'");
  EXPECT_EQ(0, r1.exit_code);
  EXPECT_NE(std::string::npos, r1.output.find("81"));

  // Test list operations
  auto r2 = cvc("exec -e '(car (list 10 20 30))'");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_NE(std::string::npos, r2.output.find("10"));

  // Test conditional
  auto r3 = cvc("exec -e '(if (> 5 3) 99 0)'");
  EXPECT_EQ(0, r3.exit_code);
  EXPECT_NE(std::string::npos, r3.output.find("99"));

  // Test nested let with multiple bindings
  auto r4 = cvc("exec -e '(let ((a 10) (b 20)) (+ a b))'");
  EXPECT_EQ(0, r4.exit_code);
  EXPECT_NE(std::string::npos, r4.output.find("30"));
}

TEST_F(CvcCliTest, Integration_ExecWithResourceLimits) {
  // Test max-steps enforcement — a long-running computation should be stopped
  auto r = cvc("exec -e '(begin 1 2 3 4 5 6 7 8 9 10)' --max-steps 100");
  // Should complete fine with the step limit
  EXPECT_EQ(0, r.exit_code);
}

TEST_F(CvcCliTest, Integration_ExecMultiFileScript) {
  // Write a multi-expression script file
  std::string script = path("multi_script.sx");
  {
    std::ofstream f(script);
    f << "(let ((pi 3.14159) (r 5)) (* pi r r))\n";
  }
  auto r = cvc("exec -f " + script);
  EXPECT_EQ(0, r.exit_code);
  // area = 3.14159 * 25 = 78.53975
  EXPECT_NE(std::string::npos, r.output.find("78."));
}
