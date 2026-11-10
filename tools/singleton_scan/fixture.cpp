// The scan's end-to-end fixture (test_scan.py): one of each kind of process-wide object the scan
// must find, and constant tables it must leave alone. No includes, so a cross compiler needs no
// sysroot. Every object is reached from a function with external linkage, so -O2 keeps it.

int make_value();

struct widget {
  widget() : n(make_value()) {}
  int n;
};

// Found: namespace-scope mutables, internal and external.
int g_counter = 1;
static int s_internal;
namespace {
int anon_count;
}
namespace fx {
widget g_widget;
}

// Found: a class static.
struct registry {
  static int count;
};
int registry::count = 0;

// Found: a function-local static and its guard.
int &local_static() {
  static int n = make_value();
  return n;
}

// Found, keyed <header>: an inline function's static and a template's static member.
inline int &header_static() {
  static int h = make_value();
  return h;
}
template <class T> struct tpl {
  static int v;
};
template <class T> int tpl<T>::v = 7;

// Found: thread-locals.
thread_local int tl_scratch;
thread_local widget tl_widget;

// Not found: constants.
constexpr int k_table[] = {1, 2, 3, 5, 8};
const char *const k_names[] = {"alpha", "beta"};

// Found: a table of pointers that may be rewritten.
const char *g_names[] = {"gamma", "delta"};

int use_all(int i) {
  return g_counter + ++s_internal + ++anon_count + fx::g_widget.n + ++registry::count +
         local_static() + header_static() + ++tpl<int>::v + ++tl_scratch + tl_widget.n +
         k_table[i % 5] + k_names[i % 2][0] + g_names[i % 2][0];
}

void set_name(int i, const char *s) { g_names[i % 2] = s; }
