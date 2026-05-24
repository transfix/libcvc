#include <cvc/state_eviction_store.h>

#include <sstream>

namespace cvc {

// ---------------------------------------------------------------------------
// memory_eviction_store
// ---------------------------------------------------------------------------

std::string memory_eviction_store::store(const std::string& /*path*/,
                                          const std::string& value,
                                          const boost::any& data)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto id = std::to_string(_next_id++);
    _entries[id] = {value, data};
    return id;
}

bool memory_eviction_store::retrieve(const std::string& token,
                                      std::string& value_out,
                                      boost::any& data_out)
{
    std::lock_guard<std::mutex> lk(_mu);
    auto it = _entries.find(token);
    if (it == _entries.end())
        return false;
    value_out = it->second.value;
    data_out = it->second.data;
    _entries.erase(it);
    return true;
}

bool memory_eviction_store::discard(const std::string& token)
{
    std::lock_guard<std::mutex> lk(_mu);
    return _entries.erase(token) > 0;
}

std::size_t memory_eviction_store::stored_count() const
{
    std::lock_guard<std::mutex> lk(_mu);
    return _entries.size();
}

// ---------------------------------------------------------------------------
// null_eviction_store
// ---------------------------------------------------------------------------

std::string null_eviction_store::store(const std::string& /*path*/,
                                        const std::string& /*value*/,
                                        const boost::any& /*data*/)
{
    return "__null__";
}

bool null_eviction_store::retrieve(const std::string& /*token*/,
                                    std::string& value_out,
                                    boost::any& data_out)
{
    value_out.clear();
    data_out = boost::any();
    return true;
}

bool null_eviction_store::discard(const std::string& /*token*/)
{
    return true;
}

} // namespace cvc
