# Style Guide

This style guide is intended to maintain a consistent, readable style,
and ensure the use of best practices in our code. It is not set in
stone, and any part of it can be changed if enough members wish to do
so *and are willing to help update the codebase to match the proposed
style* - there isn't much point in a style guide if the code doesn't
follow it!

You can automatically apply the formatting rules using the
`clang-format` tool; see [Automatic Formatting with
`clang-format`](#automatic-formatting-with-clang-format) for more
information.

## Style Rules

### Formatting
- Use tabs to indent, and use additional spaces to align multi-line code
  (i.e. when lines are too long and have to be wrapped).
- Lines should be kept under 95 characters long, including indentation.
- Opening curly braces should be placed on the same line as the block header. 
  For example:
  ```c
  int MyFunction() {
      if (true) {
          while (false) {
          }
      }
      return 0;
  }
  ```
- Function names should be in `camelCase`, where the first letter of the first
  word is lowercase and the first letter of each subsequent word is uppercase.
- Struct, Enum, Class, Enum Class, Union, etc. names should be in
  `PascalCase`, where the first letter of every word is uppercase.
- Variable names should be in `snake_case`, where every letter is lowercase
  and there is an underscore between words.
- Indent `case` labels inside of a `switch` statement.
- When defining variables or parameters that have pointer or reference types,
  the `*` or `&` symbols should be aligned to the left, next to the type name,
  as they describe the *type* of the variable rather than the variable itself.

  For example:
  ```c
  int* value; // good
  ```
  as opposed to
  ```c
  int *value; // bad
  ```
- Include directives should be ordered as follows:
  1. Headers that are part of our codebase.
  2. C/C++ standard library headers.
  3. Other system headers, such as Linux kernel headers.
  An extra line should be placed between each group.

  For example:
  - Good:
    ```c
    #include "Globals.h"

    #include <cstdint>

    #include <signal.h>
    ```
  - Bad:
    ```c
    #include <iostream>
    #include <signal.h>

    #include "Globals.h"
    ```

### Best Practices
- Constants should be defined with the `constexpr` keyword wherever
  possible (for primitive types and some standard library
  classes). This allows the compiler to perform optimizations on them.
- Wherever possible, avoid `#define` macros for constants in favor of
  the `constexpr` keyword. For more complex macros (e.g. functions),
  please consider `constexpr` functions instead - they are also
  evaluated at compile time but are more type-safe. See [this
  page](https://docs.microsoft.com/en-us/cpp/cpp/constexpr-cpp?view=msvc-160#constexpr_functions)
  for more information.
- Wherever possible, avoid the use of raw pointers in favor of smart
  pointers (i.e `shared_ptr`) or references. If you need to have an
  output parameter for a function, a reference will work more often
  than not.
- The line `#pragma once` should *always* be at the beginning of a
  header file - this prevents it from being included (redefining all
  the symbols inside) more than once.
- Wherever possible for C standard library headers (e.g. `<stdint.h>`,
  `<stdio.h>`), prefer using C++-style compatibility headers
  (e.g. `<cstdint>`, `<cstdio>`). See [C Compatibility
  Headers](https://en.cppreference.com/w/cpp/header#C_compatibility_headers)
  on cppreference.com for more information.
- In C++ code, whenever dynamic memory allocation is required, the `new` and
  `delete`/`delete[]` keywords should be used as opposed to `malloc()`
  and `free()`; the former are more type-safe, easier to use with more
  complex types, and have various other advantages - see [this post on
  StackOverflow](https://stackoverflow.com/a/240308) for a pretty good
  description of the differences.

  There are a few exceptions:
  - In C++ files, if you are using C standard library functions like `strdup()`
    that return pointers to memory allocated with `malloc()`, you *must* use
    `free()` with these.
  - In C code, `new` and `delete`/`delete[]` are not keywords, so you must use
    `malloc()` and `free()`. The rule only applies to C++ code.

## Automatic formatting with `clang-format`

The formatting rules can be applied automatically using LLVM's
[`clang-format`
tool](https://clang.llvm.org/docs/ClangFormat.html). On Ubuntu, it can
be installed with `sudo apt install clang-format`; on Mac OS, it can
be installed from Homebrew with `brew install clang-format`.

Please note that `clang-format` can only automatically apply
formatting changes, and cannot implement the "best practices" rules
above.

To format a file with `clang-format`, run
```bash
clang-format -i <file> ...
```
with as many file names as you want. For example:

```bash
clang-format -i Autonomous.cpp
clang-format -i Rover.cpp Utils.cpp Utils.h
```
You can even run something like
```bash
clang-format -i *.cpp *.h
```
to format all C++ source files in a directory.

There are also plugins for several editors that integrate support for `clang-format`:
- [Emacs integration](https://clang.llvm.org/docs/ClangFormat.html#emacs-integration)
- [Vim integration](https://clang.llvm.org/docs/ClangFormat.html#vim-integration)
- [Visual Studio Code
  integration](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format)

The style rules for `clang-format` can be found in `.clang-format`
file in the top-level directory of the repository. When proposing
changes to the style guide, please include the corresponding change to
the `.clang-format` file as well (if possible).


