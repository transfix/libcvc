#ifndef CVC_STATE_MEMORY_MANAGER_H
#define CVC_STATE_MEMORY_MANAGER_H

#include <cvc/state_eviction_store.h>

#include <chrono>
#include <cstddef>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace cvc {

class state; // forward

/// Memory-pressure-aware manager for cvc::state trees.
///
/// Tracks the estimated memory footprint of state node payloads and
/// evicts least-recently-used entries when the total exceeds a
/// configurable budget.  Evicted payloads are pushed to a
/// pluggable state_eviction_store and can be repopulated on demand.
class state_memory_manager {
public:
    enum class eviction_policy { lru, lfu, size_weighted_lru, ttl, manual };

    /// Construct a memory manager for the given state tree root.
    /// @param root          Root of the state tree to manage.
    /// @param budget_bytes  Maximum resident payload bytes before eviction.
    /// @param policy        Eviction policy (default: LRU).
    /// @param store         Backing store for evicted payloads.
    ///                      If null, a memory_eviction_store is created.
    state_memory_manager(state& root,
                         std::size_t budget_bytes,
                         eviction_policy policy = eviction_policy::lru,
                         std::unique_ptr<state_eviction_store> store = nullptr);

    ~state_memory_manager();

    // -- Budget management ---------------------------------------------------

    void set_budget(std::size_t bytes);
    std::size_t budget() const;
    std::size_t resident_bytes() const;
    std::size_t evicted_count() const;

    // -- Manual eviction / repopulation --------------------------------------

    /// Evict a node's payload.  Returns true if the node was resident.
    bool evict(const std::string& path);

    /// Repopulate a previously evicted node.  Returns true on success.
    bool repopulate(const std::string& path);

    /// Returns true if the path has been evicted and not yet repopulated.
    bool is_evicted(const std::string& path) const;

    // -- Tracking ------------------------------------------------------------

    /// Register a node payload for tracking.
    /// Call this when a node's value or data changes.
    void track(const std::string& path, std::size_t payload_bytes);

    /// Remove a node from tracking (e.g. when the node is deleted).
    void untrack(const std::string& path);

    /// Record an access to a tracked node (updates LRU order).
    void touch(const std::string& path);

    // -- Eviction callbacks --------------------------------------------------

    using eviction_callback = std::function<void(const std::string& path)>;
    void on_eviction(eviction_callback cb);
    void on_repopulation(eviction_callback cb);

    // -- Policy tuning -------------------------------------------------------

    void set_ttl(std::chrono::seconds ttl);
    void set_high_watermark(double ratio);
    void set_low_watermark(double ratio);

    eviction_policy policy() const { return _policy; }

    // -- Maintenance ---------------------------------------------------------

    /// Run one eviction pass: evict nodes until resident_bytes <= low watermark.
    /// Called automatically by track() when resident bytes exceed the high
    /// watermark.  Can also be called manually.
    std::size_t run_eviction();

private:
    state& _root;
    std::size_t _budget;
    eviction_policy _policy;
    std::unique_ptr<state_eviction_store> _store;
    double _high_watermark = 0.95;
    double _low_watermark = 0.80;
    std::chrono::seconds _ttl{0};

    mutable std::mutex _mu;

    // Per-node tracking entry
    struct node_entry {
        std::string path;
        std::size_t bytes = 0;
        std::size_t access_count = 0;
        std::chrono::steady_clock::time_point last_access;
        std::chrono::steady_clock::time_point tracked_since;
    };

    // LRU order: front = most recently used, back = least recently used
    std::list<node_entry> _lru_list;
    std::unordered_map<std::string, std::list<node_entry>::iterator> _node_map;

    // Evicted nodes: path -> eviction store token
    std::unordered_map<std::string, std::string> _evicted;

    std::size_t _resident_bytes = 0;

    std::vector<eviction_callback> _eviction_cbs;
    std::vector<eviction_callback> _repopulation_cbs;

    // Evict the least valuable node according to policy.
    // Caller must hold _mu.  Returns bytes freed, or 0 if nothing to evict.
    std::size_t evict_one_locked();

    // Resolve a state node by dot-delimited path from root.
    state* resolve(const std::string& path);
};

} // namespace cvc

#endif // CVC_STATE_MEMORY_MANAGER_H
