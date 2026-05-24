#include <cvc/state_exec/types.h>

#include <sstream>

namespace cvc::state_exec {

// -- value_tag ---------------------------------------------------------------

std::string value_tag::type_name() const
{
    return std::visit([](auto&& arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::monostate>)
            return "nil";
        else if constexpr (std::is_same_v<T, bool>)
            return "bool";
        else if constexpr (std::is_same_v<T, int64_t>)
            return "int";
        else if constexpr (std::is_same_v<T, double>)
            return "float";
        else if constexpr (std::is_same_v<T, std::string>)
            return "string";
        else if constexpr (std::is_same_v<T, symbol>)
            return "symbol";
        else if constexpr (std::is_same_v<T, list_ptr>)
            return "list";
        else if constexpr (std::is_same_v<T, closure_ptr>)
            return "closure";
        else if constexpr (std::is_same_v<T, dict_ptr>)
            return "dict";
        else if constexpr (std::is_same_v<T, native_fn>)
            return "native_fn";
        else if constexpr (std::is_same_v<T, data_object_ptr>)
            return "data_object";
        else
            return "unknown";
    }, v);
}

// -- environment -------------------------------------------------------------

value_t* environment::lookup(const std::string& name)
{
    auto it = bindings.find(name);
    if (it != bindings.end())
        return &it->second;
    if (outer)
        return outer->lookup(name);
    return nullptr;
}

const value_t* environment::lookup(const std::string& name) const
{
    auto it = bindings.find(name);
    if (it != bindings.end())
        return &it->second;
    if (outer)
        return outer->lookup(name);
    return nullptr;
}

void environment::set(const std::string& name, value_t val)
{
    bindings[name] = std::move(val);
}

void environment::set_existing(const std::string& name, value_t val)
{
    auto it = bindings.find(name);
    if (it != bindings.end()) {
        it->second = std::move(val);
        return;
    }
    if (outer) {
        outer->set_existing(name, std::move(val));
        return;
    }
    // Not found anywhere — set in this scope
    bindings[name] = std::move(val);
}

environment_ptr environment::extend(environment_ptr parent)
{
    auto child = std::make_shared<environment>();
    child->outer = std::move(parent);
    return child;
}

// -- to_string ---------------------------------------------------------------

std::string to_string(const value_t& val)
{
    return std::visit([](auto&& arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::monostate>)
            return "nil";
        else if constexpr (std::is_same_v<T, bool>)
            return arg ? "#t" : "#f";
        else if constexpr (std::is_same_v<T, int64_t>)
            return std::to_string(arg);
        else if constexpr (std::is_same_v<T, double>) {
            std::ostringstream oss;
            oss << arg;
            return oss.str();
        }
        else if constexpr (std::is_same_v<T, std::string>)
            return "\"" + arg + "\"";
        else if constexpr (std::is_same_v<T, symbol>)
            return arg.name;
        else if constexpr (std::is_same_v<T, list_ptr>) {
            if (!arg || arg->empty())
                return "()";
            std::string s = "(";
            for (std::size_t i = 0; i < arg->size(); ++i) {
                if (i > 0) s += " ";
                s += to_string((*arg)[i]);
            }
            s += ")";
            return s;
        }
        else if constexpr (std::is_same_v<T, closure_ptr>)
            return "<closure>";
        else if constexpr (std::is_same_v<T, dict_ptr>) {
            if (!arg || arg->empty())
                return "{}";
            std::string s = "{";
            for (std::size_t i = 0; i < arg->size(); ++i) {
                if (i > 0) s += ", ";
                s += "\"" + (*arg)[i].first + "\": "
                     + to_string((*arg)[i].second);
            }
            s += "}";
            return s;
        }
        else if constexpr (std::is_same_v<T, native_fn>)
            return "<native_fn>";
        else if constexpr (std::is_same_v<T, data_object_ptr>)
            return arg ? "<data_object:" + arg->type_name + ">"
                       : "<data_object:null>";
        else
            return "<unknown>";
    }, val.v);
}

// -- values_equal ------------------------------------------------------------

bool values_equal(const value_t& a, const value_t& b)
{
    if (a.v.index() != b.v.index())
        return false;

    return std::visit([&b](auto&& arg_a) -> bool {
        using T = std::decay_t<decltype(arg_a)>;
        auto& arg_b = std::get<T>(b.v);

        if constexpr (std::is_same_v<T, std::monostate>)
            return true;
        else if constexpr (std::is_same_v<T, bool>)
            return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, int64_t>)
            return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, double>)
            return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, std::string>)
            return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, symbol>)
            return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, list_ptr>) {
            if (arg_a == arg_b) return true;
            if (!arg_a || !arg_b) return false;
            if (arg_a->size() != arg_b->size()) return false;
            for (std::size_t i = 0; i < arg_a->size(); ++i)
                if (!values_equal((*arg_a)[i], (*arg_b)[i]))
                    return false;
            return true;
        }
        else if constexpr (std::is_same_v<T, closure_ptr>)
            return arg_a == arg_b; // identity comparison
        else if constexpr (std::is_same_v<T, dict_ptr>) {
            if (arg_a == arg_b) return true;
            if (!arg_a || !arg_b) return false;
            if (arg_a->size() != arg_b->size()) return false;
            for (std::size_t i = 0; i < arg_a->size(); ++i) {
                if ((*arg_a)[i].first != (*arg_b)[i].first)
                    return false;
                if (!values_equal((*arg_a)[i].second, (*arg_b)[i].second))
                    return false;
            }
            return true;
        }
        else if constexpr (std::is_same_v<T, native_fn>)
            return false; // functions are never equal
        else if constexpr (std::is_same_v<T, data_object_ptr>)
            return arg_a == arg_b; // identity comparison
        else
            return false;
    }, a.v);
}

} // namespace cvc::state_exec
