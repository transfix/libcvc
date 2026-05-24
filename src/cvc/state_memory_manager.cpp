#include <cvc/state_memory_manager.h>
#include <cvc/state.h>

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace cvc {

state_memory_manager::state_memory_manager(
    state& root,
    std::size_t budget_bytes,
    eviction_policy policy,
    std::unique_ptr<state_eviction_store> store)
    : _root(root)
    , _budget(budget_bytes)
    , _policy(policy)
    , _store(store ? std::move(store)
                   : std::make_unique<memory_eviction_store>())
{
}

state_memory_manager::~state_memory_manager() = default;

// -- Budget management -------------------------------------------------------

void state_memory_manager::set_budget(std::size_t bytes)
{
    std::lock_guard<std::mutex> lk(_mu);
    _budget = bytes;
}

std::size_t state_memory_manager::budget() const
{
    std::lock_guard<std::mutex> lk(_mu);
    return _budget;
}

std::size_t state_memory_manager::resident_bytes() const
{
    std::lock_guard<std::mutex> lk(_mu);
    return _resident_bytes;
}

std::size_t state_memory_manager::evicted_count() const
{
    std::lock_guard<std::mutex> lk(_mu);
    return _evicted.size();
}

// -- Tracking ----------------------------------------------------------------

void state_memory_manager::track(const std::string& path,
                                  std::size_t payload_bytes)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto now = std::chrono::steady_clock::now();

    auto it = _node_map.find(path);
    if (it != _node_map.end()) {
        // Update existing entry
        _resident_bytes -= it->second->bytes;
        it->second->bytes = payload_bytes;
        it->second->last_access = now;
        it->second->access_count++;
        _resident_bytes += payload_bytes;
        // Move to front (most recently used)
        _lru_list.splice(_lru_list.begin(), _lru_list, it->second);
    } else {
        // New entry — push to front
        _lru_list.push_front({path, payload_bytes, 1, now, now});
        _node_map[path] = _lru_list.begin();
        _resident_bytes += payload_bytes;
    }

    // If we had an eviction record for this path, clear it
    _evicted.erase(path);

    // Check if eviction is needed
    auto high = static_cast<std::size_t>(_high_watermark * _budget);
    if (_resident_bytes > high) {
        auto low = static_cast<std::size_t>(_low_watermark * _budget);
        while (_resident_bytes > low && !_lru_list.empty()) {
            if (evict_one_locked() == 0)
                break;
        }
    }
}

void state_memory_manager::untrack(const std::string& path)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _node_map.find(path);
    if (it != _node_map.end()) {
        _resident_bytes -= it->second->bytes;
        _lru_list.erase(it->second);
        _node_map.erase(it);
    }
    // Also discard any eviction store entry
    auto ev = _evicted.find(path);
    if (ev != _evicted.end()) {
        _store->discard(ev->second);
        _evicted.erase(ev);
    }
}

void state_memory_manager::touch(const std::string& path)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _node_map.find(path);
    if (it == _node_map.end())
        return;
    it->second->last_access = std::chrono::steady_clock::now();
    it->second->access_count++;
    _lru_list.splice(_lru_list.begin(), _lru_list, it->second);
}

// -- Manual eviction / repopulation ------------------------------------------

bool state_memory_manager::evict(const std::string& path)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _node_map.find(path);
    if (it == _node_map.end())
        return false;

    // Resolve the state node to capture its payload
    state* node = resolve(path);
    if (!node)
        return false;

    std::string val = node->value();
    boost::any dat;
    if (node->isData<boost::any>())
        dat = node->data();

    auto token = _store->store(path, val, dat);
    _evicted[path] = token;

    // Clear the node's payload (keep children/metadata)
    node->value("");
    node->data(boost::any());

    _resident_bytes -= it->second->bytes;
    _lru_list.erase(it->second);
    _node_map.erase(it);

    // Fire callbacks (outside lock would be better, but kept simple)
    for (auto& cb : _eviction_cbs)
        cb(path);

    return true;
}

bool state_memory_manager::repopulate(const std::string& path)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto ev = _evicted.find(path);
    if (ev == _evicted.end())
        return false;

    state* node = resolve(path);
    if (!node)
        return false;

    std::string val;
    boost::any dat;
    if (!_store->retrieve(ev->second, val, dat))
        return false;

    _evicted.erase(ev);

    // Restore payload
    if (!val.empty())
        node->value(val);
    if (!dat.empty())
        node->data(dat);

    // Re-track
    std::size_t bytes = val.size() + sizeof(boost::any);
    auto now = std::chrono::steady_clock::now();
    _lru_list.push_front({path, bytes, 1, now, now});
    _node_map[path] = _lru_list.begin();
    _resident_bytes += bytes;

    for (auto& cb : _repopulation_cbs)
        cb(path);

    return true;
}

bool state_memory_manager::is_evicted(const std::string& path) const
{
    std::lock_guard<std::mutex> lk(_mu);
    return _evicted.count(path) > 0;
}

// -- Callbacks ---------------------------------------------------------------

void state_memory_manager::on_eviction(eviction_callback cb)
{
    std::lock_guard<std::mutex> lk(_mu);
    _eviction_cbs.push_back(std::move(cb));
}

void state_memory_manager::on_repopulation(eviction_callback cb)
{
    std::lock_guard<std::mutex> lk(_mu);
    _repopulation_cbs.push_back(std::move(cb));
}

// -- Policy tuning -----------------------------------------------------------

void state_memory_manager::set_ttl(std::chrono::seconds ttl)
{
    std::lock_guard<std::mutex> lk(_mu);
    _ttl = ttl;
}

void state_memory_manager::set_high_watermark(double ratio)
{
    std::lock_guard<std::mutex> lk(_mu);
    _high_watermark = ratio;
}

void state_memory_manager::set_low_watermark(double ratio)
{
    std::lock_guard<std::mutex> lk(_mu);
    _low_watermark = ratio;
}

// -- Maintenance -------------------------------------------------------------

std::size_t state_memory_manager::run_eviction()
{
    std::lock_guard<std::mutex> lk(_mu);
    auto low = static_cast<std::size_t>(_low_watermark * _budget);
    std::size_t freed = 0;
    while (_resident_bytes > low && !_lru_list.empty()) {
        auto f = evict_one_locked();
        if (f == 0)
            break;
        freed += f;
    }
    return freed;
}

// -- Private -----------------------------------------------------------------

std::size_t state_memory_manager::evict_one_locked()
{
    if (_lru_list.empty())
        return 0;

    // Select victim based on policy
    std::list<node_entry>::iterator victim;

    switch (_policy) {
    case eviction_policy::lru:
        // Back of list = least recently used
        victim = std::prev(_lru_list.end());
        break;

    case eviction_policy::lfu:
        // Find least frequently used
        victim = std::min_element(
            _lru_list.begin(), _lru_list.end(),
            [](const node_entry& a, const node_entry& b) {
                return a.access_count < b.access_count;
            });
        break;

    case eviction_policy::size_weighted_lru: {
        // LRU but prefer large payloads (score = bytes / recency_rank)
        // Simple approach: find least recently used among the top 25% by size
        std::vector<std::list<node_entry>::iterator> candidates;
        for (auto it = _lru_list.begin(); it != _lru_list.end(); ++it)
            candidates.push_back(it);
        std::sort(candidates.begin(), candidates.end(),
                  [](auto a, auto b) { return a->bytes > b->bytes; });
        auto top_quarter = std::max<std::size_t>(candidates.size() / 4, 1);
        candidates.resize(top_quarter);
        // Among the large ones, pick the least recently used
        victim = *std::min_element(
            candidates.begin(), candidates.end(),
            [](auto a, auto b) { return a->last_access < b->last_access; });
        break;
    }

    case eviction_policy::ttl: {
        auto now = std::chrono::steady_clock::now();
        victim = _lru_list.end();
        for (auto it = _lru_list.begin(); it != _lru_list.end(); ++it) {
            if (now - it->tracked_since >= _ttl) {
                if (victim == _lru_list.end() ||
                    it->tracked_since < victim->tracked_since)
                    victim = it;
            }
        }
        if (victim == _lru_list.end())
            return 0; // nothing expired
        break;
    }

    case eviction_policy::manual:
        return 0; // no automatic eviction
    }

    // Perform the eviction
    auto& entry = *victim;
    state* node = resolve(entry.path);
    if (!node) {
        // Node no longer exists — just remove tracking
        auto bytes = entry.bytes;
        _resident_bytes -= bytes;
        _node_map.erase(entry.path);
        _lru_list.erase(victim);
        return bytes;
    }

    std::string val = node->value();
    boost::any dat;
    if (node->isData<boost::any>())
        dat = node->data();

    auto token = _store->store(entry.path, val, dat);
    _evicted[entry.path] = token;

    node->value("");
    node->data(boost::any());

    auto bytes = entry.bytes;
    _resident_bytes -= bytes;

    for (auto& cb : _eviction_cbs)
        cb(entry.path);

    _node_map.erase(entry.path);
    _lru_list.erase(victim);

    return bytes;
}

state* state_memory_manager::resolve(const std::string& path)
{
    if (path.empty())
        return &_root;

    state* current = &_root;
    std::istringstream ss(path);
    std::string segment;
    while (std::getline(ss, segment, '.')) {
        if (segment.empty())
            continue;
        current = &(*current)(segment);
    }
    return current;
}

} // namespace cvc
