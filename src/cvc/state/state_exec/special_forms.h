// special_forms.h — which special form a call head names, found without a chain of string
// compares (private to the evaluators).
#ifndef CVC_STATE_EXEC_SPECIAL_FORMS_H
#define CVC_STATE_EXEC_SPECIAL_FORMS_H

#include <cstdint>
#include <string_view>

namespace cvc::state_exec::detail {

/// The special forms the stackless evaluator knows (stackless_evaluator::all_special_forms());
/// the recursive evaluator knows all of them but `yield` and `break`.
enum class special_form : std::uint8_t {
  none, // not a special form: a user macro, or a function call
  if_,
  begin,
  while_,
  for_,
  set,
  quote,
  lambda,
  return_,
  yield,
  break_,
  let,
  defun,
  defmacro,
  eval,
  root,
  super,
  defclass,
  try_,
  raise,
};

/// The special form `name` names, or special_form::none. A switch on the length and a letter or
/// two picks the one candidate, and one compare confirms it, so an ordinary call head costs at
/// most one string compare (most cost none) where a chain of compares cost up to 19.
inline special_form special_form_of(std::string_view name) {
  using sf = special_form;
  sf candidate = sf::none;
  std::string_view spelled;
  switch (name.size()) {
  case 2:
    candidate = sf::if_, spelled = "if";
    break;
  case 3:
    switch (name[0]) {
    case 'f':
      candidate = sf::for_, spelled = "for";
      break;
    case 's':
      candidate = sf::set, spelled = "set";
      break;
    case 'l':
      candidate = sf::let, spelled = "let";
      break;
    case 't':
      candidate = sf::try_, spelled = "try";
      break;
    default:
      return sf::none;
    }
    break;
  case 4:
    switch (name[0]) {
    case 'e':
      candidate = sf::eval, spelled = "eval";
      break;
    case 'r':
      candidate = sf::root, spelled = "root";
      break;
    default:
      return sf::none;
    }
    break;
  case 5:
    switch (name[0]) {
    case 'b':
      if (name[1] == 'e')
        candidate = sf::begin, spelled = "begin";
      else
        candidate = sf::break_, spelled = "break";
      break;
    case 'w':
      candidate = sf::while_, spelled = "while";
      break;
    case 'q':
      candidate = sf::quote, spelled = "quote";
      break;
    case 'y':
      candidate = sf::yield, spelled = "yield";
      break;
    case 'd':
      candidate = sf::defun, spelled = "defun";
      break;
    case 's':
      candidate = sf::super, spelled = "super";
      break;
    case 'r':
      candidate = sf::raise, spelled = "raise";
      break;
    default:
      return sf::none;
    }
    break;
  case 6:
    switch (name[0]) {
    case 'l':
      candidate = sf::lambda, spelled = "lambda";
      break;
    case 'r':
      candidate = sf::return_, spelled = "return";
      break;
    default:
      return sf::none;
    }
    break;
  case 8:
    if (name[3] == 'm')
      candidate = sf::defmacro, spelled = "defmacro";
    else
      candidate = sf::defclass, spelled = "defclass";
    break;
  default:
    return sf::none;
  }
  return name == spelled ? candidate : sf::none;
}

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_SPECIAL_FORMS_H
