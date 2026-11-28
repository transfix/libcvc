// Boost.Regex's static mutex, for NetBSD only (src/cvc/CMakeLists.txt adds this file there).
//
// NetBSD's C++ library is the base system's libstdc++ (gcc 10). It has std::mutex but no timed
// mutex (_GTHREAD_USE_MUTEX_TIMEDLOCK is 0), and Boost.Config reads that, under clang's __GNUC__ of
// 4, as "no <mutex>" (BOOST_NO_CXX11_HDR_MUTEX). So Boost.Regex takes its C++03 path there, whose
// scoped_static_mutex_lock (boost/regex/pending/static_mutex.hpp) is defined in Boost.Regex's
// compiled sources. cvcpkg's Boost is header-only, so every program linking libcvc failed on
// undefined references to it. These are the pthreads version's four members, as Boost's own
// static_mutex.cpp has them: lock in the constructor when asked, unlock in the destructor.
#define BOOST_REGEX_SOURCE // BOOST_REGEX_DECL exports them from libcvc
#include <boost/regex/config.hpp>
#include <boost/regex/pending/static_mutex.hpp>

#if !defined(BOOST_REGEX_CXX03)
// Boost.Regex is on its C++11 path here, which needs none of this.
#elif defined(BOOST_HAS_PTHREADS) && defined(PTHREAD_MUTEX_INITIALIZER)
namespace boost {

scoped_static_mutex_lock::scoped_static_mutex_lock(static_mutex &m, bool lk)
    : m_mutex(m), m_have_lock(false) {
  if (lk)
    lock();
}

scoped_static_mutex_lock::~scoped_static_mutex_lock() {
  if (m_have_lock)
    unlock();
}

void scoped_static_mutex_lock::lock() {
  if (!m_have_lock)
    m_have_lock = pthread_mutex_lock(&m_mutex.m_mutex) == 0;
}

void scoped_static_mutex_lock::unlock() {
  if (m_have_lock) {
    pthread_mutex_unlock(&m_mutex.m_mutex);
    m_have_lock = false;
  }
}

} // namespace boost
#else
#error "Boost.Regex's C++03 path without pthreads: this file defines only the pthreads mutex"
#endif
