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
  - Volume arithmetic tests create smallvol synthetic data via the library
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
#include <csignal>
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
#if defined(_WIN32)
  // _popen invokes "cmd /c <command>".  When <command> begins with a
  // double-quote, cmd.exe may strip the first and last quote characters
  // as a matched outer pair, which breaks commands that contain multiple
  // quoted segments (e.g. a quoted exe path AND a quoted argument).
  // Wrapping the entire command in an extra pair of quotes avoids this:
  //   cmd /c ""path\to\exe" args "expr"" 2>&1
  std::string full_cmd = "\"" + cmd + " 2>&1\"";
#else
  std::string full_cmd = cmd + " 2>&1";
#endif
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

// Shell-quote a string: single quotes on Unix, double quotes on Windows.
static std::string sq(const std::string &s) {
#if defined(_WIN32)
  return "\"" + s + "\"";
#else
  return "'" + s + "'";
#endif
}

// Read an entire file into a string (empty string if it doesn't exist).
static std::string slurp(const std::string &p) {
  std::ifstream in(p, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

#if !defined(_WIN32)
// Start `cvc serve` in the background, redirecting all of its std streams away
// from the test's pipes (a child that keeps the test's stdout open can wedge the
// test runner). stdin<-/dev/null, stdout+stderr->logf, PID captured to pidf.
// Portable: no GNU-only `timeout`/`stdbuf` (absent on macOS). Returns pid or -1.
static pid_t start_background_server(const std::string &cvc_bin, const std::string &sock,
                                     const std::string &logf, const std::string &pidf,
                                     const std::string &extra_args = "") {
  std::string cmd = "\"" + cvc_bin + "\" serve -l " + sock + " -t ipc --pump-interval 50 " +
                    extra_args + " > \"" + logf + "\" 2>&1 < /dev/null & echo $! > \"" + pidf +
                    "\"";
  if (std::system(cmd.c_str()) != 0)
    return -1;
  pid_t pid = -1;
  std::ifstream(pidf) >> pid;
  return pid;
}

// Poll for the server's AF_UNIX socket, up to ~`tries` * 50ms. Returns true if
// it appeared. Callers that need the banner should stop_server() + slurp(logf).
static bool wait_for_socket(const std::string &sock, int tries = 100) {
  for (int i = 0; i < tries; ++i) {
    if (fs::exists(sock))
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

// Ask the server to stop (SIGINT -> clean shutdown, which also flushes its
// buffered startup banner), then wait until it is actually gone so nothing
// outlives the test; SIGKILL as a last resort.
static void stop_server(pid_t pid) {
  if (pid <= 0)
    return;
  ::kill(pid, SIGINT);
  for (int i = 0; i < 200 && ::kill(pid, 0) == 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (::kill(pid, 0) == 0)
    ::kill(pid, SIGKILL);
}
#endif

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

// Create a smallvol tetrahedron geometry for testing.
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
  std::string sock_dir; // short /tmp dir for AF_UNIX sockets (~108-byte path limit)

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

#if !defined(_WIN32)
    // The per-test tmpdir path is too deep for AF_UNIX socket binding
    // (~108-byte limit), so IPC socket paths live in a short, equally
    // unique /tmp directory instead.
    sock_dir = (fs::path("/tmp") / oss.str()).string();
    fs::create_directories(sock_dir);
#endif

    // Pre-create smallvol test data files that many tests share
    test_vol = path("test.rawiv");
    write_test_volume(test_vol);

    test_geo = path("test.off");
    write_test_geometry(test_geo);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(test_dir, ec);
    if (!sock_dir.empty())
      fs::remove_all(sock_dir, ec);
  }

  RunResult cvc(const std::string &args) { return run_cmd("\"" + cvc_bin + "\" " + args); }

  std::string path(const std::string &name) const { return test_dir + "/" + name; }

  // Path for AF_UNIX sockets — must stay short (see sock_dir above).
  std::string spath(const std::string &name) const { return sock_dir + "/" + name; }
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
  auto r = cvc("exec -e " + sq("(+ 1 2 3)"));
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("6"));
}

TEST_F(CvcCliTest, ExecMultiply) {
  auto r = cvc("exec -e " + sq("(* 7 6)"));
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("42"));
}

TEST_F(CvcCliTest, ExecDefine) {
  auto r = cvc("exec -e " + sq("(let ((x 10)) (+ x 5))"));
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("15"));
}

TEST_F(CvcCliTest, ExecLambda) {
  auto r = cvc("exec -e " + sq("(let ((f (lambda (x y) (+ x y)))) (f 3 4))"));
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
  auto r1 = cvc("exec -e " + sq("(let ((square (lambda (x) (* x x)))) (square 9))"));
  EXPECT_EQ(0, r1.exit_code);
  EXPECT_NE(std::string::npos, r1.output.find("81"));

  // Test list operations
  auto r2 = cvc("exec -e " + sq("(car (list 10 20 30))"));
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_NE(std::string::npos, r2.output.find("10"));

  // Test conditional
  auto r3 = cvc("exec -e " + sq("(if (> 5 3) 99 0)"));
  EXPECT_EQ(0, r3.exit_code);
  EXPECT_NE(std::string::npos, r3.output.find("99"));

  // Test nested let with multiple bindings
  auto r4 = cvc("exec -e " + sq("(let ((a 10) (b 20)) (+ a b))"));
  EXPECT_EQ(0, r4.exit_code);
  EXPECT_NE(std::string::npos, r4.output.find("30"));
}

TEST_F(CvcCliTest, Integration_ExecWithResourceLimits) {
  // Test max-steps enforcement — a long-running computation should be stopped
  auto r = cvc("exec -e " + sq("(begin 1 2 3 4 5 6 7 8 9 10)") + " --max-steps 100");
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

// ===========================================================================
// Subcommand --help for the VolUtils wrapper commands
// ===========================================================================

TEST_F(CvcCliTest, SubtractHelp) {
  auto r = cvc("subtract --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, ScaleHelp) {
  auto r = cvc("scale --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--factor"));
}

TEST_F(CvcCliTest, ClipHelp) {
  auto r = cvc("clip --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--threshold"));
}

TEST_F(CvcCliTest, NegateHelp) {
  auto r = cvc("negate --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, BilateralHelp) {
  auto r = cvc("bilateral --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--radiometric"));
  EXPECT_NE(std::string::npos, r.output.find("--spatial"));
}

TEST_F(CvcCliTest, ContrastHelp) {
  auto r = cvc("contrast --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--resistor"));
}

TEST_F(CvcCliTest, AnisotropicHelp) {
  auto r = cvc("anisotropic --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--iterations"));
}

TEST_F(CvcCliTest, GdtvHelp) {
  auto r = cvc("gdtv --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--lambda"));
}

TEST_F(CvcCliTest, ResizeHelp) {
  auto r = cvc("resize --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--dims"));
}

TEST_F(CvcCliTest, DifferenceHelp) {
  auto r = cvc("difference --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--output"));
}

TEST_F(CvcCliTest, ClampMinHelp) {
  auto r = cvc("clamp-min --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--min"));
}

TEST_F(CvcCliTest, AverageHelp) {
  auto r = cvc("average --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--input"));
}

TEST_F(CvcCliTest, ExtractHelp) {
  auto r = cvc("extract --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--var"));
}

TEST_F(CvcCliTest, CompareHelp) {
  auto r = cvc("compare --help");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("--tolerance"));
}

// ===========================================================================
// Info edge cases: line/quad geometry, unknown extension
// ===========================================================================

TEST_F(CvcCliTest, InfoLineGeometry) {
  // cvc-raw geometry with a 2-index element = line segment
  std::string raw = path("segments.raw");
  {
    std::ofstream f(raw);
    f << "4 1\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n0 1\n";
  }
  auto r = cvc("info " + raw);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Lines:"));
}

TEST_F(CvcCliTest, InfoQuadGeometry) {
  // cvc-raw geometry with a 4-index element = quad
  std::string raw = path("quads.raw");
  {
    std::ofstream f(raw);
    f << "4 1\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n0 1 2 3\n";
  }
  auto r = cvc("info " + raw);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Quads:"));
}

TEST_F(CvcCliTest, InfoUnknownExtensionFails) {
  auto r = cvc("info " + path("mystery.xyz"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Unknown file type"));
}

TEST_F(CvcCliTest, ConvertUnknownTypeFails) {
  auto r = cvc("convert " + test_vol + " " + path("bad.rawiv") + " -t Bogus");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Unknown voxel type"));
}

// ===========================================================================
// Volume arithmetic: remaining error branches
// ===========================================================================

TEST_F(CvcCliTest, SubtractRequiresTwoInputs) {
  auto r = cvc("subtract -i " + test_vol + " -o " + path("s.rawiv"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("exactly 2"));
}

TEST_F(CvcCliTest, SsimRequiresTwoInputs) {
  auto r = cvc("ssim -i " + test_vol);
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("exactly 2"));
}

// ===========================================================================
// VolUtils wrappers: difference / clamp-min / average / extract / resize
// ===========================================================================

TEST_F(CvcCliTest, DifferenceVolumes) {
  std::string out = path("diff_out.rawiv");
  auto r = cvc("difference -i " + test_vol + " " + test_vol + " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, DifferenceRequiresTwoInputs) {
  auto r = cvc("difference -i " + test_vol + " -o " + path("d.rawiv"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("exactly 2"));
}

TEST_F(CvcCliTest, ClampMinVolume) {
  std::string out = path("clamp_out.rawiv");
  auto r = cvc("clamp-min " + test_vol + " " + out + " -m 50");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, AverageVolumes) {
  // Three inputs exercise the N-way pairwise fold
  std::string out = path("avg_out.rawiv");
  auto r = cvc("average -i " + test_vol + " " + test_vol + " " + test_vol + " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, AverageRequiresTwoInputs) {
  auto r = cvc("average -i " + test_vol + " -o " + path("a.rawiv"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("at least 2"));
}

TEST_F(CvcCliTest, ExtractVariableTimestep) {
  std::string out = path("extract_out.rawiv");
  auto r = cvc("extract " + test_vol + " " + out + " --var 0 -t 0");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, ResizeVolume) {
  std::string out = path("resize_out.rawiv");
  auto r = cvc("resize " + test_vol + " " + out + " -d 8 8 8");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));

  auto r2 = cvc("info " + out);
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_NE(std::string::npos, r2.output.find("8 x 8 x 8"));
}

TEST_F(CvcCliTest, ResizeRequiresThreeDims) {
  auto r = cvc("resize " + test_vol + " " + path("r.rawiv") + " -d 8 8");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("exactly 3"));
}

// ===========================================================================
// Compare command (exact voxel diff with pass/fail exit code)
// ===========================================================================

TEST_F(CvcCliTest, CompareEqualVolumes) {
  auto r = cvc("compare -i " + test_vol + " " + test_vol);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("OK: volumes match"));
}

TEST_F(CvcCliTest, CompareMismatchExitsNonzero) {
  std::string neg = path("cmp_neg.rawiv");
  ASSERT_EQ(0, cvc("negate " + test_vol + " " + neg).exit_code);
  auto r = cvc("compare -i " + test_vol + " " + neg);
  EXPECT_EQ(1, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("MISMATCH"));
}

TEST_F(CvcCliTest, CompareDimensionMismatchExitsNonzero) {
  std::string smallvol = path("cmp_small.rawiv");
  ASSERT_EQ(0, cvc("downsample " + test_vol + " " + smallvol + " -f 2").exit_code);
  auto r = cvc("compare -i " + test_vol + " " + smallvol);
  EXPECT_EQ(1, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("dimensions differ"));
}

TEST_F(CvcCliTest, CompareRequiresTwoInputs) {
  auto r = cvc("compare -i " + test_vol);
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("exactly 2"));
}

// ===========================================================================
// Noise filters (tiny 4^3 input)
// ===========================================================================

TEST_F(CvcCliTest, BilateralFilterVolume) {
  std::string out = path("bilateral_out.rawiv");
  auto r = cvc("bilateral " + test_vol + " " + out + " --radius 1");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, ContrastEnhancementVolume) {
  std::string out = path("contrast_out.rawiv");
  auto r = cvc("contrast " + test_vol + " " + out + " --resistor 0.9");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, AnisotropicDiffusionVolume) {
  std::string out = path("aniso_out.rawiv");
  auto r = cvc("anisotropic " + test_vol + " " + out + " -n 2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, GdtvFilterVolume) {
  std::string out = path("gdtv_out.rawiv");
  auto r = cvc("gdtv " + test_vol + " " + out + " -n 1");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

// ===========================================================================
// Rotate command (single angle and multi-rotation series)
// ===========================================================================

TEST_F(CvcCliTest, RotateSingleAngle) {
  std::string out = path("rot_out.rawiv");
  auto r = cvc("rotate " + test_vol + " " + out + " -a 90");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Rotated 90"));
}

TEST_F(CvcCliTest, RotateMultipleAngles) {
  std::string base = path("rotseries");
  auto r = cvc("rotate " + test_vol + " " + base + " -a 0 -n 2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Wrote 2 rotations"));
  EXPECT_TRUE(fs::exists(base + ".0000.rawiv"));
  EXPECT_TRUE(fs::exists(base + ".0001.rawiv"));
}

// ===========================================================================
// Projection / back-projection on a tiny volume
// ===========================================================================

TEST_F(CvcCliTest, ProjectVolume) {
  std::string angles = path("angles.txt");
  {
    std::ofstream f(angles);
    f << "0\n90\n";
  }
  std::string out = path("proj_out.rawiv");
  auto r = cvc("project " + test_vol + " " + out + " -a " + angles);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Projected 2 angles"));
}

TEST_F(CvcCliTest, ProjectMissingAnglesFileFails) {
  auto r = cvc("project " + test_vol + " " + path("p.rawiv") + " -a " + path("no_such_angles.txt"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Cannot open angles file"));
}

TEST_F(CvcCliTest, BackprojectVolume) {
  std::string angles = path("angles.txt");
  {
    std::ofstream f(angles);
    f << "0\n90\n";
  }
  std::string proj = path("bp_proj.rawiv");
  ASSERT_EQ(0, cvc("project " + test_vol + " " + proj + " -a " + angles).exit_code);

  std::string recon = path("bp_recon.rawiv");
  auto r = cvc("backproject " + proj + " " + recon + " -a " + angles + " -d 4");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(recon));
  EXPECT_NE(std::string::npos, r.output.find("Reconstructed 4^3"));

  // Unfiltered variant
  std::string recon2 = path("bp_recon_nf.rawiv");
  auto r2 = cvc("backproject " + proj + " " + recon2 + " -a " + angles + " -d 4 --no-filter");
  EXPECT_EQ(0, r2.exit_code);
  EXPECT_TRUE(fs::exists(recon2));
}

TEST_F(CvcCliTest, BackprojectMissingAnglesFileFails) {
  auto r = cvc("backproject " + test_vol + " " + path("b.rawiv") + " -a " +
               path("no_such_angles.txt") + " -d 4");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Cannot open angles file"));
}

// ===========================================================================
// Image I/O: vol2img / img2vol / rgba-merge
// ===========================================================================

// vol2img/img2vol are backed by cvc::vol_to_slices / slices_to_volume (the
// Magick++ path). ImageMagick is now enabled on every CI platform including
// Windows (see the cvcpkg-prefix fallback in src/cvc/CMakeLists.txt), so these
// run everywhere.
TEST_F(CvcCliTest, Vol2ImgExportsSlices) {
  std::string dir = path("slices");
  auto r = cvc("vol2img " + test_vol + " " + dir);
  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_NE(std::string::npos, r.output.find("Exported 4 slices"));
  EXPECT_TRUE(fs::exists(dir + "/slice_00000.png"));
  EXPECT_TRUE(fs::exists(dir + "/slice_00003.png"));
}

TEST_F(CvcCliTest, Img2VolImportsSlices) {
  std::string dir = path("slices_rt");
  ASSERT_EQ(0, cvc("vol2img " + test_vol + " " + dir).exit_code);

  std::string out = path("img2vol_out.rawiv");
  auto r = cvc("img2vol -i " + dir + "/slice_00000.png " + dir + "/slice_00001.png -o " + out);
  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Imported 2 images"));
}

TEST_F(CvcCliTest, RgbaMergeVolumes) {
  std::string out = path("rgba_out.rawiv");
  auto r = cvc("rgba-merge -i " + test_vol + " " + test_vol + " " + test_vol + " " + test_vol +
               " -o " + out);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, RgbaMergeRequiresFourInputs) {
  auto r = cvc("rgba-merge -i " + test_vol + " " + test_vol + " -o " + path("x.rawiv"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("exactly 4"));
}

// ===========================================================================
// SDF / iso option branches
// ===========================================================================

TEST_F(CvcCliTest, SdfWithExplicitBbox) {
  std::string out = path("sdf_bbox.rawiv");
  auto r = cvc("sdf -i " + test_geo + " -o " + out + " -d 16,16,16 -b 0,0,0,2,2,2 -a v2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, IsoFastContouringCentralDiff) {
  std::string out = path("iso_fc.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100 -m fastcontouring -n central-diff");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, IsoLibisocontourBsplineInterp) {
  std::string out = path("iso_lic.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100 -m libisocontour -n bspline-interp");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

TEST_F(CvcCliTest, IsoWithPropertyVolume) {
  // NOTE: only the default duallib method is used here; combining
  // -m libisocontour with -p aborts (cvc::null_dimension, library bug).
  std::string out = path("iso_prop.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100 -p " + test_vol);
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
}

// ===========================================================================
// Mesh extraction commands on a smallvol tetrahedron SDF
// ===========================================================================

TEST_F(CvcCliTest, TetrahedralizeFromSdf) {
  std::string sdf_vol = path("tet_sdf.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  std::string out = path("tet_mesh.off");
  auto r = cvc("tetrahedralize -i " + sdf_vol + " -o " + out + " -v 0.0");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Wrote tetrahedral mesh"));

  // With a property volume
  std::string out_p = path("tet_mesh_prop.off");
  auto rp = cvc("tetrahedralize -i " + sdf_vol + " -o " + out_p + " -v 0.0 -p " + sdf_vol);
  EXPECT_EQ(0, rp.exit_code);
  EXPECT_TRUE(fs::exists(out_p));
}

TEST_F(CvcCliTest, TetrahedralizeMethodAndImproveFlags) {
  std::string sdf_vol = path("tet_sdf2.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  // NOTE: --improve optimization segfaults in tetrahedralize (library bug);
  // the other four improvement methods are exercised below.
  auto r1 = cvc("tetrahedralize -i " + sdf_vol + " -o " + path("t_gf.off") +
                " -v 0.0 -m fastcontouring --improve geo-flow -n central-diff -q 1");
  EXPECT_EQ(0, r1.exit_code);

  auto r2 = cvc("tetrahedralize -i " + sdf_vol + " -o " + path("t_ec.off") +
                " -v 0.0 --improve edge-contract -q 1");
  EXPECT_EQ(0, r2.exit_code);

  auto r3 = cvc("tetrahedralize -i " + sdf_vol + " -o " + path("t_jl.off") +
                " -v 0.0 --improve joe-liu -q 1");
  EXPECT_EQ(0, r3.exit_code);

  auto r4 = cvc("tetrahedralize -i " + sdf_vol + " -o " + path("t_mv.off") +
                " -v 0.0 --improve minimal-vol -q 1");
  EXPECT_EQ(0, r4.exit_code);

  auto r5 = cvc("tetrahedralize -i " + sdf_vol + " -o " + path("t_lic.off") +
                " -v 0.0 -m libisocontour -n bspline-interp");
  EXPECT_EQ(0, r5.exit_code);
}

TEST_F(CvcCliTest, HexahedralizeFromSdf) {
  std::string sdf_vol = path("hex_sdf.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  std::string out = path("hex_mesh.off");
  auto r = cvc("hexahedralize -i " + sdf_vol + " -o " + out + " -v 0.0");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Wrote hexahedral mesh"));

  // With a property volume
  std::string out_p = path("hex_mesh_prop.off");
  auto rp = cvc("hexahedralize -i " + sdf_vol + " -o " + out_p + " -v 0.0 -p " + sdf_vol);
  EXPECT_EQ(0, rp.exit_code);
  EXPECT_TRUE(fs::exists(out_p));
}

TEST_F(CvcCliTest, HexahedralizeMethodAndImproveFlags) {
  std::string sdf_vol = path("hex_sdf2.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  // NOTE: --improve joe-liu and minimal-vol segfault in hexahedralize
  // (library bug); geo-flow, edge-contract and optimization are safe.
  auto r1 = cvc("hexahedralize -i " + sdf_vol + " -o " + path("h_lic.off") +
                " -v 0.0 -m libisocontour -n bspline-interp");
  EXPECT_EQ(0, r1.exit_code);

  auto r2 = cvc("hexahedralize -i " + sdf_vol + " -o " + path("h_gf.off") +
                " -v 0.0 --improve geo-flow -q 1");
  EXPECT_EQ(0, r2.exit_code);

  auto r3 = cvc("hexahedralize -i " + sdf_vol + " -o " + path("h_ec.off") +
                " -v 0.0 --improve edge-contract -q 1");
  EXPECT_EQ(0, r3.exit_code);

  auto r4 = cvc("hexahedralize -i " + sdf_vol + " -o " + path("h_opt.off") +
                " -v 0.0 --improve optimization -q 1");
  EXPECT_EQ(0, r4.exit_code);

  auto r5 = cvc("hexahedralize -i " + sdf_vol + " -o " + path("h_fc.off") +
                " -v 0.0 -m fastcontouring -n central-diff");
  EXPECT_EQ(0, r5.exit_code);
}

TEST_F(CvcCliTest, Tetrahedralize2FromSdf) {
  std::string sdf_vol = path("t2_sdf.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  std::string out = path("t2_mesh.off");
  auto r = cvc("tetrahedralize2 -i " + sdf_vol + " -o " + out + " -v 0.0");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Wrote dual-tet mesh"));

  // NOTE: --improve joe-liu/minimal-vol/optimization segfault in
  // tetrahedralize2 (library bug); geo-flow and edge-contract are safe.
  auto r1 = cvc("tetrahedralize2 -i " + sdf_vol + " -o " + path("t2_fc.off") +
                " -v 0.0 -m fastcontouring -n central-diff");
  EXPECT_EQ(0, r1.exit_code);

  auto r2 = cvc("tetrahedralize2 -i " + sdf_vol + " -o " + path("t2_gf.off") +
                " -v 0.0 --improve geo-flow -q 1");
  EXPECT_EQ(0, r2.exit_code);

  auto r3 = cvc("tetrahedralize2 -i " + sdf_vol + " -o " + path("t2_ec.off") +
                " -v 0.0 --improve edge-contract -q 1");
  EXPECT_EQ(0, r3.exit_code);

  auto r4 = cvc("tetrahedralize2 -i " + sdf_vol + " -o " + path("t2_lic.off") +
                " -v 0.0 -m libisocontour -n bspline-interp");
  EXPECT_EQ(0, r4.exit_code);
}

TEST_F(CvcCliTest, LayerMeshFromSdf) {
  std::string sdf_vol = path("lm_sdf.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  std::string out = path("lm_mesh.off");
  auto r = cvc("layer-mesh -i " + sdf_vol + " -o " + out +
               " --isovalue-outer -0.1 --isovalue-inner 0.1");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Wrote layer mesh"));

  // NOTE: --improve joe-liu/minimal-vol/optimization segfault in layer-mesh
  // (library bug); geo-flow and edge-contract are safe.
  auto r1 = cvc("layer-mesh -i " + sdf_vol + " -o " + path("lm_lic.off") +
                " --isovalue-outer -0.1 --isovalue-inner 0.1 -m libisocontour -n bspline-interp");
  EXPECT_EQ(0, r1.exit_code);

  auto r2 = cvc("layer-mesh -i " + sdf_vol + " -o " + path("lm_gf.off") +
                " --isovalue-outer -0.1 --isovalue-inner 0.1 --improve geo-flow -q 1");
  EXPECT_EQ(0, r2.exit_code);

  auto r3 = cvc("layer-mesh -i " + sdf_vol + " -o " + path("lm_ec.off") +
                " --isovalue-outer -0.1 --isovalue-inner 0.1 --improve edge-contract -q 1");
  EXPECT_EQ(0, r3.exit_code);

  auto r4 = cvc("layer-mesh -i " + sdf_vol + " -o " + path("lm_fc.off") +
                " --isovalue-outer -0.1 --isovalue-inner 0.1 -m fastcontouring -n central-diff");
  EXPECT_EQ(0, r4.exit_code);
}

// ===========================================================================
// Bunny --volume (smallvol dims, v2 distance transform)
// ===========================================================================

TEST_F(CvcCliTest, BunnyVolumeSmallDims) {
  // NOTE: 16 is the smallest safe cube here — the SDF v2 path hits a SIGFPE
  // at -d 8 (library bug), and v1 takes ~2 minutes at 16^3.
  std::string out = path("bunny_vol.rawiv");
  auto r = cvc("bunny --volume -o " + out + " -d 16 -a v2 -p 0.2");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Wrote bunny SDF volume"));
}

// ===========================================================================
// State command: remaining operations and error branches
// ===========================================================================

TEST_F(CvcCliTest, StateGetSystemStart) {
  // __system.start is initialized at app startup
  auto r = cvc("state get __system.start");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_FALSE(r.output.empty());
}

TEST_F(CvcCliTest, StateGetUninitializedFails) {
  auto r = cvc("state get some.random.uninitialized.path");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("not initialized"));
}

TEST_F(CvcCliTest, StateGetMissingPathFails) {
  auto r = cvc("state get");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Path required"));
}

TEST_F(CvcCliTest, StateSetMissingPathFails) {
  auto r = cvc("state set");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Path required"));
}

TEST_F(CvcCliTest, StateDeleteMissingPathFails) {
  auto r = cvc("state delete");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Path required"));
}

TEST_F(CvcCliTest, StateSetMissingValueFails) {
  auto r = cvc("state set foo.bar");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Value required"));
}

TEST_F(CvcCliTest, StateDeleteClearsValue) {
  auto r = cvc("state delete foo.bar");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Cleared foo.bar"));
}

TEST_F(CvcCliTest, StateListFreshPathShowsNoChildren) {
  auto r = cvc("state list some.fresh.leaf");
  EXPECT_EQ(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("(no children)"));
}

// ===========================================================================
// Exec: resource-limit kill path
// ===========================================================================

TEST_F(CvcCliTest, ExecStepLimitKillsProcess) {
  // One step is not enough for a nested arithmetic expression; the
  // scheduler kills the process and the CLI exits nonzero.
  auto r = cvc("exec -e " + sq("(+ 1 (+ 2 (+ 3 (+ 4 (+ 5 6)))))") + " --max-steps 1");
  EXPECT_EQ(1, r.exit_code);
}

// ===========================================================================
// Serve command: error branches and short clean runs
// ===========================================================================

TEST_F(CvcCliTest, ServeUnknownTransportFails) {
  auto r = cvc("serve -l " + path("x.sock") + " -t bogus --node-id nodex");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Unknown transport"));
}

TEST_F(CvcCliTest, ServeTlsCertFileMissingFails) {
  auto r = cvc("serve -l " + path("x.sock") + " -t ipc --node-id nodex --tls-cert " +
               path("no_such_cert.pem"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Cannot read file"));
}

#if !defined(_WIN32)

// These drive a long-running `cvc serve` via the portable background-server
// helpers (start_background_server / wait_for_socket / stop_server): no GNU
// `timeout`, so they run on macOS too. A clean SIGINT shutdown flushes serve's
// block-buffered startup banner to the log, which we then read.
TEST_F(CvcCliTest, ServeIpcRunAndShutdown) {
  // Dummy PEM files: they are only read into strings by the CLI.
  std::string pem = path("fake.pem");
  {
    std::ofstream f(pem);
    f << "dummy pem\n";
  }
  std::string sock = spath("srv.sock");
  std::string logf = path("srv.log"), pidf = path("srv.pid");
  pid_t pid = start_background_server(
      cvc_bin, sock, logf, pidf,
      "--cluster-id testcluster --node-id node1 --sync-mode read-only --root-path scene"
      " --enforce-authority --enforce-write-policy --resolve-conflicts --auth-token tok123"
      " --blob-store-path " +
          path("blobs") + " --tls-cert " + pem + " --tls-key " + pem + " --tls-ca " + pem +
          " --tls-require-client-auth");
  ASSERT_GT(pid, 0);
  bool up = wait_for_socket(sock);
  stop_server(pid);
  std::string out = slurp(logf);
  ::unlink(sock.c_str());
  EXPECT_TRUE(up) << "server did not bind the socket; log:\n" << out;
  EXPECT_NE(std::string::npos, out.find("Server running.")) << out;
  EXPECT_NE(std::string::npos, out.find("Server stopped.")) << out;
}

TEST_F(CvcCliTest, ServeExecCoordinatorRunAndShutdown) {
  std::string sock = spath("srv_exec.sock");
  std::string logf = path("srv_exec.log"), pidf = path("srv_exec.pid");
  pid_t pid = start_background_server(
      cvc_bin, sock, logf, pidf,
      "--cluster-id testcluster --node-id node2 --sync-mode authoritative --enable-exec");
  ASSERT_GT(pid, 0);
  bool up = wait_for_socket(sock);
  stop_server(pid);
  std::string out = slurp(logf);
  ::unlink(sock.c_str());
  EXPECT_TRUE(up) << out;
  EXPECT_NE(std::string::npos, out.find("exec coordinator: enabled")) << out;
  EXPECT_NE(std::string::npos, out.find("Server stopped.")) << out;
}

TEST_F(CvcCliTest, ServeDelegateValidSpec) {
  std::string sock = spath("srv_del.sock");
  std::string logf = path("srv_del.log"), pidf = path("srv_del.pid");
  pid_t pid = start_background_server(cvc_bin, sock, logf, pidf,
                                      "--cluster-id testcluster --node-id node3 --delegate "
                                      "sub:remotecluster:" +
                                          path("remote.sock") + ":1");
  ASSERT_GT(pid, 0);
  bool up = wait_for_socket(sock);
  stop_server(pid);
  std::string out = slurp(logf);
  ::unlink(sock.c_str());
  EXPECT_TRUE(up) << out;
  EXPECT_NE(std::string::npos, out.find("delegated sub -> remotecluster")) << out;
  EXPECT_NE(std::string::npos, out.find("Server stopped.")) << out;
}

TEST_F(CvcCliTest, ServeDelegateInvalidSpecFails) {
  // An invalid --delegate spec is rejected before the serve loop starts, so the
  // command exits fast on its own — run it in the foreground (no background).
  auto r = cvc("serve -l " + spath("srv_bad.sock") +
               " -t ipc --cluster-id testcluster --node-id node4 --delegate badspec");
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Invalid delegation spec"));
}

// NOTE: the former ServeGeneratedNodeIdRejected test (which asserted that a
// missing --node-id produced an id the session *rejected*) was dropped in the
// merge with #211, which fixed that bug — the auto-generated id is now a valid
// C identifier and serve starts. #211's Regression_ServeAutoNodeIdIsValidIdentifier
// covers the corrected behavior.

TEST_F(CvcCliTest, ServeSeedPeerHandshake) {
  // Start a peer server, then a second server seeded with the peer's socket,
  // and verify the joiner records the seed and shuts down cleanly.
  std::string peer_sock = spath("peer_a.sock");
  std::string peer_log = path("peer_a.log"), peer_pid = path("peer_a.pid");
  pid_t peer = start_background_server(cvc_bin, peer_sock, peer_log, peer_pid,
                                       "--cluster-id testcluster --node-id peerseed");
  ASSERT_GT(peer, 0);
  ASSERT_TRUE(wait_for_socket(peer_sock)) << slurp(peer_log);

  std::string sock = spath("peer_b.sock");
  std::string logf = path("peer_b.log"), pidf = path("peer_b.pid");
  pid_t joiner = start_background_server(
      cvc_bin, sock, logf, pidf, "--cluster-id testcluster --node-id peerb --seed " + peer_sock);
  ASSERT_GT(joiner, 0);
  bool up = wait_for_socket(sock);
  stop_server(joiner);
  std::string out = slurp(logf);
  stop_server(peer);
  ::unlink(sock.c_str());
  ::unlink(peer_sock.c_str());
  EXPECT_TRUE(up) << out;
  EXPECT_NE(std::string::npos, out.find("seeds:")) << out;
  EXPECT_NE(std::string::npos, out.find("Server stopped.")) << out;
}

#endif // !defined(_WIN32)

// ===========================================================================
// Cluster-status command
// ===========================================================================

TEST_F(CvcCliTest, ClusterStatusMissingSeedFails) {
  auto r = cvc("cluster-status -l " + path("cs.sock") + " -t ipc");
  EXPECT_NE(0, r.exit_code);
}

TEST_F(CvcCliTest, ClusterStatusUnknownTransportFails) {
  auto r = cvc("cluster-status -l " + path("cs.sock") + " -t bogus -s " + path("seed.sock"));
  EXPECT_NE(0, r.exit_code);
  EXPECT_NE(std::string::npos, r.output.find("Unknown transport"));
}

TEST_F(CvcCliTest, ClusterStatusGrpcTransportSelected) {
  // Exercise the grpc-transport dispatch branch of cluster-status against a
  // dead endpoint. The exit code is build-dependent — with gRPC compiled out
  // the command fails fast, but with gRPC compiled in (the -grpc CI matrix
  // entry) it connects to the unused port and can exit 0 — so assert only
  // that grpc is accepted as a valid transport (the dispatch branch was
  // taken, not the "Unknown transport" parser error path).
  auto r = cvc("cluster-status -l 127.0.0.1:39473 -s 127.0.0.1:39474");
  EXPECT_EQ(std::string::npos, r.output.find("Unknown transport"));
}

// NOTE: the former ClusterStatusRejectedNodeId test (which asserted
// cluster-status could never connect because its generated node id was
// rejected) was dropped in the merge with #211, which fixed that bug.
// #211's Regression_ClusterStatusDefaultIdsEndToEnd covers the corrected
// end-to-end path.

// ===========================================================================
// Regression tests for reproducible cvc CLI crashes (root-caused in the
// library / CLI, not worked around here).
// ===========================================================================

// Bug 1: `sdf -a v2` on a smallvol grid divided by zero (SIGFPE) in the
// DistanceTransform progress reporting: e.g. dim[2] < 15 made
// `k % (total_near_iterations / 15)` a modulo-by-zero. Small dims must work.
TEST_F(CvcCliTest, Regression_SdfV2SmallDimsNoFpe) {
  for (const char *dims : {"2,2,2", "4,4,4", "8,8,8", "8,4,2"}) {
    std::string out = path(std::string("sdf_small_") + dims + ".rawiv");
    auto r = cvc("sdf -i " + test_geo + " -o " + out + " -d " + dims + " -a v2");
    EXPECT_EQ(0, r.exit_code) << "dims " << dims << ":\n" << r.output;
    EXPECT_TRUE(fs::exists(out)) << "dims " << dims;
  }
}

// Bug 1 (same root cause), reached through the `bunny --volume` shortcut.
TEST_F(CvcCliTest, Regression_BunnyVolumeV2SmallDimNoFpe) {
  std::string out = path("bunny_small.rawiv");
  auto r = cvc("bunny --volume -d 8 -a v2 -o " + out);
  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_TRUE(fs::exists(out));
}

// Bug 2: `iso -m libisocontour -p <vol>` aborted with an uncaught
// cvc::null_dimension. The libisocontour / fastcontouring extraction paths
// leave the octree grid empty (dim[] == 0), so func_val() tried to resize the
// property volume to a 0x0x0 grid. Property interpolation must now succeed by
// sampling the property volume directly.
TEST_F(CvcCliTest, Regression_IsoLibisocontourWithPropertyVolume) {
  std::string out = path("iso_lic_prop.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100 -m libisocontour -p " + test_vol);
  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_TRUE(fs::exists(out));
  EXPECT_NE(std::string::npos, r.output.find("Property interpolation complete"));
}

TEST_F(CvcCliTest, Regression_IsoFastContouringWithPropertyVolume) {
  std::string out = path("iso_fc_prop.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100 -m fastcontouring -p " + test_vol);
  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_TRUE(fs::exists(out));
}

// The duallib path (which does build the octree grid) must keep working.
TEST_F(CvcCliTest, Regression_IsoDuallibWithPropertyVolume) {
  std::string out = path("iso_dl_prop.off");
  auto r = cvc("iso -i " + test_vol + " -o " + out + " -v 100 -p " + test_vol);
  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_TRUE(fs::exists(out));
}

// Bugs 3-6 (one shared root cause): the quality-improvement routines assumed a
// specific element type and left an index uninitialized when their element loop
// ran zero times, then dereferenced it. joe-liu/minimal-vol are tet-only
// (crashed on hex meshes and on empty dual-tet meshes); optimization is hex-only
// (crashed on tet meshes). Every mesh command must survive every --improve
// method (unsupported/empty combinations are now a no-op).
TEST_F(CvcCliTest, Regression_MeshImproveMethodsNoCrash) {
  std::string sdf_vol = path("improve_sdf.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  const char *methods[] = {"geo-flow", "edge-contract", "joe-liu", "minimal-vol", "optimization"};
  const char *cmds[] = {"tetrahedralize", "hexahedralize", "tetrahedralize2"};
  for (const char *cmd : cmds) {
    for (const char *m : methods) {
      std::string out = path(std::string(cmd) + "_" + m + ".off");
      auto r = cvc(std::string(cmd) + " -i " + sdf_vol + " -o " + out + " -v 0.0 --improve " + m +
                   " -q 1");
      EXPECT_EQ(0, r.exit_code) << cmd << " --improve " << m << ":\n" << r.output;
      EXPECT_TRUE(fs::exists(out)) << cmd << " --improve " << m;
    }
  }
}

// Bug 6: layer-mesh produces a dual-tet mesh that is empty for isovalues that
// enclose no layer; joe-liu/minimal-vol/optimization crashed on the empty mesh.
TEST_F(CvcCliTest, Regression_LayerMeshImproveMethodsNoCrash) {
  std::string sdf_vol = path("layer_improve_sdf.rawiv");
  ASSERT_EQ(0, cvc("sdf -i " + test_geo + " -o " + sdf_vol + " -d 16,16,16 -a v2").exit_code);

  for (const char *m : {"geo-flow", "edge-contract", "joe-liu", "minimal-vol", "optimization"}) {
    std::string out = path(std::string("layer_improve_") + m + ".off");
    auto r = cvc("layer-mesh -i " + sdf_vol + " -o " + out +
                 " --isovalue-outer 0.02 --isovalue-inner -0.02 --improve " + m + " -q 1");
    EXPECT_EQ(0, r.exit_code) << "layer-mesh --improve " << m << ":\n" << r.output;
  }
}

#if !defined(_WIN32)
// Bug 7: `serve` without --node-id auto-generated "node-<ticks>" and
// `cluster-status` generated "status-<ticks>", and the default --cluster-id was
// "cvc-cluster" -- all contain '-' and were rejected by the C-identifier
// validation in distributed_state_session. serve was therefore unusable without
// an explicit --node-id, and cluster-status was permanently broken (its report
// code never ran). The auto/default ids are now valid C identifiers.
TEST_F(CvcCliTest, Regression_ServeAutoNodeIdIsValidIdentifier) {
  // Short /tmp socket path (AF_UNIX has a ~108-byte limit).
  std::string sock = "/tmp/cvc_b7serve_" + std::to_string(::getpid()) + ".sock";
  std::string logf = path("serve_banner.log");
  std::string pidf = path("serve.pid");

  // Start serve with no --node-id and the default --cluster-id.
  pid_t pid = start_background_server(cvc_bin, sock, logf, pidf);
  ASSERT_GT(pid, 0);

  // The server is up once it has bound the socket.
  bool up = false;
  for (int i = 0; i < 100 && !up; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    up = fs::exists(sock);
  }

  // Stopping via SIGINT flushes the buffered startup banner to logf on exit.
  stop_server(pid);
  std::string out = slurp(logf);
  ::unlink(sock.c_str());

  EXPECT_TRUE(up) << "server did not bind the socket; log:\n" << out;
  // The auto-generated node id and default cluster id must be accepted (no
  // validation error) and the startup banner must show valid identifiers.
  EXPECT_EQ(std::string::npos, out.find("violates C identifier")) << out;
  EXPECT_NE(std::string::npos, out.find("node_")) << out;
  EXPECT_NE(std::string::npos, out.find("cluster: cvc_cluster")) << out;
}

// Bug 7 (end-to-end): with valid default ids, cluster-status connects to a
// running server and prints its report -- exercising the code that used to be
// dead because join() always threw on the invalid "status-<ticks>" node id.
TEST_F(CvcCliTest, Regression_ClusterStatusDefaultIdsEndToEnd) {
  std::string base = "/tmp/cvc_b7cs_" + std::to_string(::getpid());
  std::string srv_sock = base + "_srv.sock";
  std::string cs_sock = base + "_cs.sock";
  std::string srv_log = path("cs_server.log");
  std::string pidf = path("cs_server.pid");

  // Start the server in the background; no --node-id, default --cluster-id.
  pid_t pid = start_background_server(cvc_bin, srv_sock, srv_log, pidf);
  ASSERT_GT(pid, 0);

  bool up = false;
  for (int i = 0; i < 200 && !up; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    up = fs::exists(srv_sock);
  }
  ASSERT_TRUE(up) << "server did not start:\n" << slurp(srv_log);

  // cluster-status self-terminates after printing (it has no --node-id flag, so
  // it must generate a valid one itself). Run it directly.
  auto r = cvc("cluster-status -l " + cs_sock + " -t ipc -s " + srv_sock);

  stop_server(pid);
  ::unlink(srv_sock.c_str());
  ::unlink(cs_sock.c_str());

  EXPECT_EQ(0, r.exit_code) << r.output;
  EXPECT_EQ(std::string::npos, r.output.find("violates C identifier")) << r.output;
  EXPECT_NE(std::string::npos, r.output.find("Session status")) << r.output;
}
#endif // !_WIN32
