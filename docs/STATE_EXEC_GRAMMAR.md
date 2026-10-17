# state_exec Grammar

This is the formal grammar of the state_exec s-expression language: the text that
`cvc::state_exec::parse()` and `parse_all()` read (`inc/cvc/state/state_exec/parser.h`). Ariadne
`.ari` documents embed it in `init`, `on`, `bind` and `*_when` fields.

**Status: target grammar.** The parser in `src/cvc/state/state_exec/parser.cpp` is still the
hand-written one, now with the nesting limit and its [settings](#parser-settings) (P1b). It
moves onto this grammar in phases (P2 to P4, see
[Changes from the pre-grammar parser](#changes-from-the-pre-grammar-parser)). Until then it differs
from this document exactly where that table says. `state_exec_grammar_test` checks both sides:
cases for landed phases must hold, and cases for pending phases must still differ.

From P2 on, the parser is built from this declaration. Each production below becomes a
[PEGTL](https://github.com/taocpp/PEGTL) rule with the same name in
`src/cvc/state/state_exec/grammar.h`, and `state_exec_grammar_test` checks that the names match.

## Notation

The grammar uses the [W3C EBNF notation](https://www.w3.org/TR/xml/#sec-notation): `::=` defines
a production, `|` separates alternatives, `?` `*` `+` are optional, zero-or-more and one-or-more,
`A - B` matches what `A` matches unless `B` also matches it, `'…'` and `"…"` are literal text,
`[…]` is a character class, and `#xN` is the Unicode code point N.

The input is the sequence of Unicode scalar values decoded from UTF-8. Bytes that are not
well-formed UTF-8 match nothing, so they are a parse error.

Choices are ordered and matching is greedy, as in a PEG. The prose sections below state the
few places where that matters.

## Grammar

```ebnf
/* Whole inputs. parse_all() reads a program; parse() reads a single_datum. */
program        ::= bom? atmosphere* (datum atmosphere*)*
single_datum   ::= bom? atmosphere* (datum atmosphere*)?

/* Data */
datum          ::= list | quoted | text_literal | bytes_literal | atom
list           ::= '(' atmosphere* (datum atmosphere*)* ')'
quoted         ::= "'" atmosphere* datum

/* Source characters and atmosphere */
bom            ::= #xFEFF
scalar_value   ::= [#x0-#xD7FF] | [#xE000-#x10FFFF]
source_char    ::= scalar_value - (control | bidi_control | #xFEFF)
control        ::= [#x0-#x8] | [#xE-#x1F] | [#x7F-#x84] | [#x86-#x9F]
bidi_control   ::= #x61C | [#x202A-#x202E] | [#x2066-#x2069]
atmosphere     ::= whitespace | line_comment
whitespace     ::= [#x9-#xD] | #x20 | #x85 | #x200E | #x200F | #x2028 | #x2029
other_space    ::= #xA0 | #x1680 | [#x2000-#x200A] | #x202F | #x205F | #x3000
line_comment   ::= ';' (source_char - line_end)*
line_end       ::= #xA | #xD

/* Atoms: the longest run of constituents, then classified (see "Atoms") */
atom           ::= boolean | nil | number | symbol
token          ::= constituent+
constituent    ::= source_char - (whitespace | other_space | delimiter | reserved)
delimiter      ::= #x22 | #x27 | #x28 | #x29 | #x3B                       /* " ' ( ) ; */
reserved       ::= #x2C | #x5B | #x5C | #x5D | #x60 | #x7B | #x7C | #x7D  /* , [ \ ] ` { | } */
boolean        ::= '#t' | 'true'
nil            ::= '#f' | 'false' | 'nil'
number         ::= integer | decimal_float | float_special
integer        ::= sign? (dec_digits | radix_integer)
radix_integer  ::= ('0x' | '0X' | '#x' | '#X') hex_digits
                 | ('0o' | '0O' | '#o' | '#O') oct_digits
                 | ('0b' | '0B' | '#b' | '#B') bin_digits
sign           ::= '+' | '-'
dec_digits     ::= [0-9] ('_'? [0-9])*
hex_digits     ::= hex ('_'? hex)*
oct_digits     ::= [0-7] ('_'? [0-7])*
bin_digits     ::= [0-1] ('_'? [0-1])*
hex            ::= [0-9A-Fa-f]
decimal_float  ::= sign? ( dec_digits '.' dec_digits? exponent?
                         | '.' dec_digits exponent?
                         | dec_digits exponent )
exponent       ::= [eE] sign? dec_digits
float_special  ::= '+inf.0' | '-inf.0' | '+nan.0' | '-nan.0'
radix_prefixed ::= sign? [0#] [xXoObB] constituent*
symbol         ::= token - (boolean | nil | number | radix_prefixed)

/* Text strings */
text_literal   ::= text_string | raw_text
text_string    ::= '"' (string_char | text_escape)* '"'
string_char    ::= source_char - (#x22 | #x5C)
text_escape    ::= '\' (simple_escape | octal_escape | hex_escape | unicode_escape)
simple_escape  ::= [abfnrtv] | #x22 | #x5C
octal_escape   ::= [0-7] ([0-7] [0-7]?)?
hex_escape     ::= 'x' hex hex
unicode_escape ::= 'u' hex hex hex hex
                 | 'u{' hex (hex (hex (hex (hex hex?)?)?)?)? '}'
raw_text       ::= 'r' raw_body
raw_body       ::= '"' raw_char* '"'
                 | hash_fence '"' raw_char* '"' hash_fence
hash_fence     ::= '#'+
raw_char       ::= source_char

/* Byte strings */
bytes_literal  ::= byte_string | raw_bytes
byte_string    ::= 'b"' (string_char | byte_escape)* '"'
byte_escape    ::= '\' (simple_escape | octal_escape | hex_escape)
raw_bytes      ::= 'br' raw_body
```

## Source text

- **Encoding.** Source is UTF-8. A malformed byte sequence, an overlong form or an encoded
  surrogate is an error. One leading U+FEFF (a byte-order mark) is skipped; U+FEFF anywhere else
  is an error.
- **Forbidden characters.** Outside of escapes, these may not appear anywhere, including strings
  and comments:
  - C0 controls other than TAB, LF, VT, FF and CR;
  - DEL;
  - C1 controls other than NEL;
  - the bidirectional embedding, override and isolate controls U+202A–U+202E and
    U+2066–U+2069, plus U+061C. This is the "Trojan Source" defence (CVE-2021-42574): these
    characters can make source text display in a different order from the order it parses in.
    Write them as `\u{202E}` and so on when a string needs one.
- **Whitespace** is Unicode Pattern_White_Space: TAB, LF, VT, FF, CR, space, NEL, U+200E,
  U+200F, U+2028 and U+2029. The other Unicode spaces (`other_space`: NBSP, U+3000 and so on) are
  allowed inside strings and comments, but between tokens they are an error rather than a silent
  separator, because they look like a space and are not one.
- **Comments** run from `;` to the end of the line. LF and CR both end a line.
- **Positions.** `parse_error::line()` and `col()` are 1-based, and point at the character where
  the error was found, such as the non-hex character in `"\x4"`. A new line starts after LF,
  after CR, and once after CRLF. Columns count code points, not bytes.
- **Nesting.** Lists and quotes may nest at most `max_depth` levels, 256 by default.
  - `(a (b (c)))` is three levels, and `'x` counts as one, since it reads as `(quote x)`.
  - The parser is stackless. Open lists and quotes live on a heap stack, not on the native
    stack, so no input depth can overflow it.
  - The limit is for what happens to the data afterwards. Some code still walks values on the
    native stack: destroying a nested list, the snapshot codec, `to_string`. Deeper input is a
    `parse_error`, so a script from an untrusted source (a message, an HTTP body, the
    multiprocess wire) can't produce data those would crash on.
  - The default fits the 64 KiB default stack of a wasm build. Real scripts nest far less: the
    deepest in the corpus is 13 levels, and the median is 2. Once that downstream code is
    iterative too, the default can rise.
  - The limit is a parser setting. Like state_exec's other settings, it lives on the state tree;
    see [Parser settings](#parser-settings).

## Parser settings

The parser's settings live on the state tree, next to the scheduler's (`state_exec.defaults.*`
and `state_exec.schedulers.<id>.*`, see `scheduler::load_settings()`). Each setting resolves in
this order:

1. `state_exec.schedulers.<id>.parser.<key>`, for a scheduler that has an id;
2. `state_exec.defaults.parser.<key>`, for everything on that tree;
3. the built-in default.

A value that is missing, empty, not a non-negative integer or out of range is skipped, and the
next one in the order applies. `max_depth` must also be positive.

| Key | Meaning | Default |
|---|---|---|
| `max_depth` | how many lists and quotes may nest | 256 |
| `step_tokens` | 0: a scheduler parses a submitted script all at once, when it is submitted. N: it parses the script inside the new process, N tokens per step (see [Reading inside the scheduler](#reading-inside-the-scheduler)) | 0; **1024** for the app's exec scheduler and pycvc's `Exec` |

Who reads them:

- **`scheduler`** resolves them in `load_settings()`, like `max_pending_messages`. It publishes
  the effective values under `state_exec.schedulers.<id>.parser.*` and uses them for every
  script submitted to it afterwards.
- **`async_scheduler`** has no settings loader. It reads `state_exec.defaults.parser.*` from its
  watch root each time a script is submitted. Keys that aren't set fall back to the scheduler's
  own defaults (`set_parser_defaults()`), which are the built-in ones unless the host changes
  them.
  - **libcvc's own hosts all use `async_scheduler::host_parser_defaults()`** (`step_tokens` =
    1024), so a script behaves the same in each:
    - the app's exec scheduler (`app::exec_scheduler()`, which runs Ariadne's actions and
      residents);
    - pycvc's `Exec`, on its private scheduler. That scheduler watches the same app tree, so the
      same keys override both.
  - **Why 1024:** every script in the corpus fits in one slice (the largest is 159 tokens, and
    `.ari` scripts are at most 24). So a normal action pays one extra step out of its 5000. A
    slice costs about 80 µs at the parser's measured ~80 ns per token, so a pathologically large
    script is spread over steps and frames instead of stalling a drain.
  - pycvc's `Exec.run()` raises a `RuntimeError` carrying the parse error either way: from the
    killed process's exit error, or straight from `execute()` when the tree sets
    `step_tokens` = 0.
- **Ariadne** reads `state_exec.defaults.parser.*` from the app's tree when it compiles a
  predicate.
- **Code without a state tree** (a standalone evaluator, the `.ari` channel lint) uses the
  built-in defaults, or passes `parse_options` to `parse()` and `parse_all()` explicitly.
  `parse_options_from_state()` does the resolution above for any host that has a tree.

### Reading inside the scheduler

The parser is stackless and resumable. Its whole state is an input position plus a heap stack
of open lists and quotes, and `parser::read(out, budget)` stops after `budget` tokens. So a
scheduler can parse a script the same way it evaluates one: a slice per step. With
`parser.step_tokens` = N:

- **Submitting.** `execute(script)` returns at once. The new process starts in a reading phase.
  Each of its scheduler steps reads N more tokens (parens, quotes, atoms, strings). The step
  after the last token sets up evaluation, exactly as parsing at submit would have, and
  evaluation steps follow.
- **Budgets.** Reading steps are process steps. They count against `max_steps` and `max_time`,
  and they interleave with other processes. A huge script costs its own process's budget instead
  of stalling the scheduler before the process exists.
- **Errors.** A syntax error surfaces where a runtime error would. The async scheduler kills the
  process, with the parse error as its exit error. The sync scheduler throws it from `step()` or
  `run()`. With step_tokens = 0, `execute()` throws it, as before. Ariadne reports a killed
  action or resident with its exit error ("ari: on: action program did not complete: …",
  "ari: resident [...] stopped: …").
- **Signals and watch events** wait until the script is read, because their handlers run on the
  evaluation stack.
- **Snapshots.** A process snapshotted while reading stores its source and parser settings. After
  a restore it reads the script again from the start.

## Data

| Production | Value (`value_t` alternative) |
|---|---|
| `list` | `list_ptr` of the elements; `()` is an empty list |
| `quoted` | the two-element list `(quote datum)` |
| `text_string`, `raw_text` | `std::string` holding UTF-8 text |
| `byte_string`, `raw_bytes` | `bytes_value` |
| `boolean` | `true` |
| `nil` | nil (`std::monostate`). `#f` and `false` read as nil, not as `bool false` |
| `integer` | `int64_t` |
| `decimal_float`, `float_special` | `double` |
| `symbol` | `symbol` |

`parse_all()` returns one value per top-level datum. `parse()` returns the one datum. It returns
nil for input that holds only atmosphere, and anything after the datum is an error.

## Atoms

An atom is the longest run of constituents (`token`). The whole token is then classified, in this
order:

1. `#t` and `true` are `boolean`. `#f`, `false` and `nil` are `nil`.
2. A token that matches `number` exactly is a number.
3. A token that matches `radix_prefixed` but is not a valid `integer` is an error, for example
   `0x1G`, `0b102`, `#xyz` or `0x`. A typo in a hex literal is reported, not read as a symbol.
4. Every other token is a `symbol`. That includes `+`, `-`, `...`, `->`, `1+`, `#foo`, `inf` and
   `nan`.

**Symbols** may use any character except whitespace, `other_space`, the delimiters `" ' ( ) ;`,
the reserved characters, controls and bidi controls. So `λ`, `→`, `café`, `日本語` and emoji
(including ZWJ sequences and variation selectors) are all symbol characters. Symbols are not
normalized, so a composed and a decomposed `é` are different symbols. That matches the raw-bytes
policy for strings and keys in UNICODE_SUPPORT_ROADMAP.md (SE-4). The grammar does not check
confusable or default-ignorable characters (U+200B and the like) in symbols.

**Reserved characters** `` , [ \ ] ` { | } `` are errors outside strings and comments. They are
kept free for quasiquote (`` ` ``, `,`), dict and vector literals (`{}` and `[]`; a dict literal is
sketched in roadmap/STATE_BINARY_STREAMING.md) and quoted symbols (`|…|`).

**Numbers.**

| Form | Examples | Value |
|---|---|---|
| decimal integer | `42`, `-7`, `+5`, `007`, `1_000_000` | `int64_t` |
| hex, octal, binary integer | `0x1F`, `#x1f`, `-0x10`, `0o17`, `#o17`, `0b1011`, `#b1011`, `0xFFFF_FFFF` | `int64_t` |
| float | `1.5`, `1.`, `.5`, `1e3`, `-2.5E-3`, `6.022_140e23` | `double` |
| special float | `+inf.0`, `-inf.0`, `+nan.0`, `-nan.0` | `double` |

- `_` may separate digits, one at a time, never at the start or end of a digit run.
- The sign goes before any radix prefix: `-0x10`, not `0x-10`.
- An integer must fit in `int64_t`, from -9223372036854775808 to 9223372036854775807. Overflow is
  an error, not a silent conversion to `double`.
- Floats are read locale-independently and correctly rounded. A float too large for a `double`
  (`1e999`) is an error. One too small rounds to zero or to a subnormal.
- There are no hex floats, rationals or complex numbers.

## Strings

A **text string** (`"…"`) is UTF-8 text. Any source character may appear in it except `"` and
`\`, including raw line breaks. A string can span lines. Escapes:

| Escape | Meaning in a text string | Meaning in a byte string |
|---|---|---|
| `\n` `\t` `\r` | LF, TAB, CR | the same bytes |
| `\a` `\b` `\f` `\v` | BEL, BS, FF, VT | the same bytes |
| `\"` `\\` | `"` and `\` | the same bytes |
| `\N`, `\NN`, `\NNN` (octal, longest match) | code point 0–0o777; `\0` is NUL | byte 0–0o377; larger is an error |
| `\xNN` (exactly two hex digits) | code point U+0000–U+00FF | byte 0x00–0xFF |
| `\uXXXX` (exactly four) or `\u{X…}` (one to six) | that code point | not allowed |

- Any other character after `\` is an error.
- A `\u` escape that names a surrogate (U+D800–U+DFFF) or a value above U+10FFFF is an error.
- Escaped code points are stored UTF-8 encoded. So `"\xe9"`, `"é"` and `"\351"` are all the
  one-character string `é`.

A **byte string** (`b"…"`) is a `bytes_value`. A non-ASCII source character contributes its
UTF-8 encoding, so `b"é"` is the two bytes C3 A9.

A **raw string** (`r"…"`) or **raw byte string** (`br"…"`) processes no escapes: every character
between the quotes is taken as written, backslashes included. To put `"` in a raw string, fence
it with one or more `#`: `r#"say "hi""#`. The string ends at the first `"` followed by as many `#`
as opened it, so `r##"a"#b"##` is `a"#b`.

**Prefixes.** `b`, `r` and `br` start a literal only at the start of a datum and only when
touching the quote, or the `#` fence of a raw string. `ab"x"` is the symbol `ab` followed by the
string `x`, and `r#foo` is a symbol. Once a datum starts with `b"`, `r"`, `br"` or a prefix,
hashes and `"`, it is a string literal, and leaving it unterminated is an error.

## Changes from the pre-grammar parser

Phase P1b adds the nesting limit and the parser settings to the hand-written parser. Phase P2
replaces the hand-written lexer with the grammar, including the number syntax. P3 adds
the Unicode source rules and strict escapes. P4 adds octal escapes, the extra C escapes and raw
strings. Every row is a case in `state_exec_grammar_test`.

| Input | Pre-grammar parser | Grammar | Phase |
|---|---|---|---|
| `a)` (a stray `)`) | the symbols `a` and `)` | error | P2 |
| `[a]`, `a,b`, `` `a ``, `{}`, `a\|b`, `a\b` | odd one-character or glued symbols | error (reserved) | P2 |
| `parse("1 2")` | `1`; the rest is ignored | error | P2 |
| `'` at end of input | `(quote nil)` | error | P2 |
| 257 nested lists | parsed (no limit) | error at the default `max_depth` | P1b |
| `; c` CR `1` | comment runs to LF, so no datum | `1` | P2 |
| `0x1F` | `31.0`, a double (via `strtod`) | `31` | P2 |
| `#x1F`, `0o17`, `0b101`, `1_000` | symbols | integers | P2 |
| `0xZZ`, `#bar`, `0x` | symbols | error | P2 |
| `+5` | `5.0`, a double | `5` | P2 |
| `9223372036854775808` | `9.2233720368547758e18`, a double | error | P2 |
| `inf`, `nan`, `-INF` | doubles | symbols | P2 |
| `+inf.0`, `-inf.0`, `+nan.0` | symbols | doubles | P2 |
| `1e999` | `+inf` | error | P2 |
| `1.5` with a `,`-decimal C locale set by the host | symbol (from reading `strtod`; not tested) | `1.5` | P2 |
| bytes that are not UTF-8 | kept in the symbol or string | error | P3 |
| U+FEFF at the start | a symbol before the first datum | skipped | P3 |
| NBSP between `a` and `b` | one symbol, `a b` with an NBSP | error | P3 |
| NEL or U+2028 between `a` and `b` | one symbol | two symbols | P3 |
| a raw bidi control or C0 control, in a string or comment | kept | error | P3 |
| `"\q"` | `\q`, backslash kept | error | P3 |
| `"\uD800"`, `"\u{110000}"` | U+FFFD | error | P3 |
| error columns | bytes | code points | P3 |
| `"\101"`, `"\012"` | `\101`; NUL then `12` | `A`; LF | P4 |
| `b"\377"`, `b"\400"` | `\377` kept; `\400` kept | the byte FF; error | P4 |
| `"\a\b\f\v"` | backslashes kept | BEL BS FF VT | P4 |
| `r"a\nb"`, `br"…"` | the symbol `r` or `br`, then a string | raw string, raw bytes | P4 |
| `r#"say "hi""#` | a symbol `r#`, strings and symbols | raw string `say "hi"` | P4 |

Printing is not part of the grammar. `to_string` round-trip (escaping strings, shortest floats,
`+inf.0`) is phase P5.

## Not in the grammar

These were considered and left out. Each would need its own decision:

- block comments (`#| … |#`) and datum comments (`#;`);
- quasiquote, unquote and unquote-splicing (the reserved `` ` `` and `,`);
- quoted symbols (`|…|`), dict literals (`{…}`) and vector literals (`[…]`);
- hex byte literals such as `#x"de ad be ef"`;
- `\N{NAME}` escapes, which need the Unicode name tables;
- line continuations inside strings;
- hex floats;
- NFC normalization of symbols or strings.

## Tests and corpus

- `src/cvc/tests/state_exec_grammar_test.cpp` checks the grammar itself:
  - every EBNF production above is defined once, and every one except `program` and
    `single_datum` is used;
  - every production has at least one accept or reject case;
  - each case holds once its phase has landed, and still differs while its phase is pending.
    `kLanded` in the test lists the phases that have landed.
- `src/cvc/tests/state_exec_corpus_test.cpp` parses the corpus: every DSL script the repo carries,
  drawn from the `.ari` files, the docs' `lisp` blocks, the C++ test sources and the pycvc tests.
  It compares each parse with `state_exec_corpus/expected.txt`, so a parser change cannot alter
  how an existing script reads without that file changing in the same PR.

To regenerate the corpus after adding scripts, run the extractor (it needs PyYAML), then rewrite
the expected parses:

```sh
python src/cvc/tests/gen_state_exec_corpus.py
CVC_UPDATE_GOLDENS=1 build/bin/state_exec_corpus_test
```

The corpus held 597 scripts when it was made, and all of them parse under the pre-grammar
parser. Of the grammar's intentional changes, only the reserved `{` `}` touch any of them: a dict
literal sketched in roadmap/STATE_BINARY_STREAMING.md, which no parser has implemented. The 177
s-expression scripts in txwtf's cfos tests, checked for the txwtf.cfos-to-state_exec port, touch
none.

The expected parses use the test's own dump format (`state_exec_grammar_dump.h`), not
`to_string`. In that format:
- integers print in decimal;
- whole doubles print as `N.0`, other doubles in the shortest form that reads back exactly, and
  the specials as `+inf.0`, `-inf.0` and `+nan.0`;
- strings print escaped, bytes as `b"…"`, and nil as `nil`;
- a symbol is wrapped in `|…|` when its bare text would read as something else;
- a parse error prints as `!error line:col`.
