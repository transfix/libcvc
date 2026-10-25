/**
 * @file exceptions.h
 * @brief DSL exceptions: the built-in exception classes, exception values, and how a C++
 *        exception thrown during evaluation becomes one (docs/STATE_EXEC_EXCEPTIONS.md).
 *
 * An exception is an ordinary class instance: a dict whose `__class__` is its class dict, with
 * `message`, `data`, `cause`, `origin` and `traceback` fields (and origin-specific ones such as
 * `native_type`). The built-in classes (Exception, RuntimeError, TypeError, ...) are class dicts
 * built once per process and shared by every environment, so an exception a native raises is
 * caught by the same class a program names. A user class derives from them with defclass.
 */
#ifndef CVC_STATE_EXEC_EXCEPTIONS_H
#define CVC_STATE_EXEC_EXCEPTIONS_H

#include <cstddef>
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace cvc::state_exec {

/// An error a libcvc builtin (or the evaluator itself) throws, naming the built-in exception
/// class it raises in the DSL ("TypeError", "IndexError", ...). The evaluator turns it into an
/// exception of that class; what() is the message.
///
/// `origin` and `fields` are the exception's `origin` and any fields of its own. A host derives
/// its own error type from this one to raise with them: pycvc's error for a Python function's
/// exception is a PythonError with origin "python" and `python_type`, and keeps the Python
/// exception to re-raise.
class dsl_error : public std::runtime_error {
public:
  dsl_error(std::string class_name, const std::string &message, std::string origin = "builtin",
            std::vector<std::pair<std::string, value_t>> fields = {})
      : std::runtime_error(message), class_name_(std::move(class_name)), origin_(std::move(origin)),
        fields_(std::move(fields)) {}
  const std::string &class_name() const noexcept { return class_name_; }
  const std::string &origin() const noexcept { return origin_; }
  const std::vector<std::pair<std::string, value_t>> &fields() const noexcept { return fields_; }

private:
  std::string class_name_;
  std::string origin_;
  std::vector<std::pair<std::string, value_t>> fields_;
};

/// A DSL exception value crossing C++. The `error` builtin and the recursive evaluator throw it
/// to raise; an evaluator's step()/run()/evaluate() throw it when nothing caught an exception, and
/// a scheduler's raise_exit_exception() throws a failed process's. what() is "Class: message".
class dsl_exception : public std::runtime_error {
public:
  /// `original`: the C++ exception the value was made from, when a native threw one. A host may
  /// rethrow it: pycvc re-raises a Python function's exception this way.
  explicit dsl_exception(value_t exc, std::exception_ptr original = nullptr);
  const value_t &value() const noexcept { return exc_; }
  std::exception_ptr original() const noexcept { return original_; }
  /// The exception's class name, and its class chain, most derived first.
  std::string class_name() const;
  std::vector<std::string> class_chain() const;

private:
  value_t exc_;
  std::exception_ptr original_;
};

// ---------------------------------------------------------------------------
// The built-in classes
// ---------------------------------------------------------------------------

/// The built-in exception class named `name` (Exception, RuntimeError, TypeError, ValueError,
/// NameError, KeyError, IndexError, ArithmeticError, MemoryError, ParseError, TimeoutError,
/// NativeError, PythonError), or null.
dict_ptr exception_class(std::string_view name);

/// The built-in exception class names, parents before children.
const std::vector<std::string> &exception_class_names();

/// Bind each built-in class in `env` as defclass binds a class: `Name` (its constructor) and
/// `Name.__cls_dict__`.
void register_exception_classes(const environment_ptr &env);

// ---------------------------------------------------------------------------
// Values
// ---------------------------------------------------------------------------

/// A new exception of class `cls`: `message`, `data`, `cause` nil, `origin`, `traceback` nil
/// (the evaluator fills it in at the raise).
value_t make_exception(const dict_ptr &cls, const std::string &message, value_t data = {},
                       const std::string &origin = "dsl");

/// The class dict of the instance `v`, or null when `v` isn't an instance.
dict_ptr class_of_instance(const value_t &v);

/// The class dict a class constructor (`Name`'s value) makes instances of, or null.
dict_ptr class_of_constructor(const value_t &ctor);

/// Whether `v` is an instance of `cls` or of a class derived from it.
bool is_instance(const value_t &v, const dict_ptr &cls);

/// Whether `v` is an exception: an instance of Exception or a class derived from it.
bool is_exception(const value_t &v);

/// A class's name (its `__name__`), or "" for a class dict without one.
std::string class_name(const dict_ptr &cls);

/// The class names from `cls` up its `__super__` chain, most derived first.
std::vector<std::string> class_chain(const dict_ptr &cls);

/// The field `key` of a dict value, or null.
const value_t *field(const value_t &dict, std::string_view key);

/// Set the field `key` of a dict value, adding it when missing.
void set_field(const value_t &dict, const std::string &key, value_t v);

/// The exception's message ("" when it has none).
std::string exception_message(const value_t &exc);

/// "Class: message".
std::string exception_summary(const value_t &exc);

/// The exception a C++ exception thrown during evaluation becomes:
/// - a dsl_exception is its value;
/// - a dsl_error is its class, with origin "builtin";
/// - a parse_error is ParseError, with `line` and `col`;
/// - std::invalid_argument, std::out_of_range and std::bad_alloc are ValueError, IndexError and
///   MemoryError;
/// - a plain std::runtime_error is RuntimeError;
/// - anything else is NativeError, with `native_type` naming the C++ type.
/// All but the first two have origin "native".
value_t exception_from(const std::exception &e);

/// Whether an exception that escaped a step is the evaluator's own interruption (a timeout or
/// interrupt), which a program can't catch.
bool is_uncatchable(const std::exception &e);

/// Whether `a` and `b` are the same exception (the same instance).
bool same_exception(const value_t &a, const value_t &b);

/// Whether `exc` is `handled` or one of its causes: as handled's cause it would make a cycle.
bool in_cause_chain(const value_t &handled, const value_t &exc);

// ---------------------------------------------------------------------------
// For the evaluators: the try form and tracebacks
// ---------------------------------------------------------------------------

/// The parts of a (try body... (catch (spec) handler...)... (finally cleanup...)) form.
struct try_form {
  std::vector<value_t> body;    ///< the body's forms
  std::vector<value_t> catches; ///< the catch clauses, in order
  value_t cleanup;              ///< (begin cleanup...) when there is a finally, else nil
};

/// Split a try form's arguments. Throws TypeError for a malformed one: a form after the body
/// that isn't a clause, a catch whose spec isn't (Type var), (Type), (var) or (), a clause after
/// the finally, or neither a catch nor a finally.
try_form parse_try(std::span<const value_t> args);

/// The class `name` names in `env`: its class dict, or null.
dict_ptr class_named(const environment_ptr &env, const std::string &name);

/// Whether a catch clause catches `exc`, and the variable it binds it to ("" for none). Its
/// class is looked up in `env`, where the try is: (catch (Type var) ...) and (catch (Type) ...)
/// catch instances of Type, and a Type that names no class throws NameError; (catch (var) ...)
/// and (catch () ...) catch any exception. A lone symbol is a Type when it names a class.
bool catch_matches(const environment_ptr &env, const value_t &clause, const value_t &exc,
                   std::string &var);

/// A catch clause's handler forms.
std::span<const value_t> catch_handler(const value_t &clause);

/// How many of the innermost frames a traceback keeps.
inline constexpr std::size_t kTracebackFrames = 64;

/// A traceback line: the expression a frame evaluates, printed, at most 120 characters.
std::string traceback_line(const value_t &expr);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_EXCEPTIONS_H
