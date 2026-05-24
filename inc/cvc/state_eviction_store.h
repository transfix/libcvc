#ifndef CVC_STATE_EVICTION_STORE_H
#define CVC_STATE_EVICTION_STORE_H

#include <boost/any.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace cvc {

/// Abstract backing store for evicted state node payloads.
///
/// When the state_memory_manager evicts a node, it stores the
/// node's value and data via this interface.  On repopulation the
/// stored payload is retrieved through the token returned by store().
class state_eviction_store {
public:
    virtual ~state_eviction_store() = default;

    /// Store evicted payload.  Returns an opaque token for later retrieval.
    virtual std::string store(const std::string& path,
                              const std::string& value,
                              const boost::any& data) = 0;

    /// Retrieve a previously evicted payload.
    /// Returns false if the token is not found.
    virtual bool retrieve(const std::string& token,
                          std::string& value_out,
                          boost::any& data_out) = 0;

    /// Discard a stored payload (e.g. when the node is deleted).
    virtual bool discard(const std::string& token) = 0;
};

// ---------------------------------------------------------------------------
// Built-in implementations
// ---------------------------------------------------------------------------

/// Stores evicted payloads in an in-process map.  Data remains in RAM
/// but is outside the memory manager's tracked budget.
class memory_eviction_store : public state_eviction_store {
public:
    std::string store(const std::string& path,
                      const std::string& value,
                      const boost::any& data) override;
    bool retrieve(const std::string& token,
                  std::string& value_out,
                  boost::any& data_out) override;
    bool discard(const std::string& token) override;

    std::size_t stored_count() const;

private:
    struct entry {
        std::string value;
        boost::any data;
    };
    mutable std::mutex _mu;
    std::unordered_map<std::string, entry> _entries;
    std::size_t _next_id = 0;
};

/// Discards evicted payloads entirely.  Reads after eviction
/// return empty string / empty any.
class null_eviction_store : public state_eviction_store {
public:
    std::string store(const std::string& path,
                      const std::string& value,
                      const boost::any& data) override;
    bool retrieve(const std::string& token,
                  std::string& value_out,
                  boost::any& data_out) override;
    bool discard(const std::string& token) override;
};

} // namespace cvc

#endif // CVC_STATE_EVICTION_STORE_H
