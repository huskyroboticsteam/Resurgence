## Style Guide
- Curly braces are always on the next line
```
int MyFunction()
{
    if(true)
    {
        while(false)
        {
        }
    }
    return 0;
}
```
- Function names are in `camelCase`
- Struct, Enum, Class, Enum Class, Union, etc. names are in `PascalCase`
- Variable names are in `snake_case`
- Always use C++'s includes: `<cstdint>, <cstdio>`, NOT `<stdint.h>, <stdio.h>`
- Order of includes is: Our includes, C++ Standard includes, kernel includes.
  Include an extra line between each group.
Good:
```
#include "Globals.h"

#include <cstdint>

#include <signal.h>
```
Bad:
```
#include <iostream>
#include <signal.h>

#include "Globals.h"
```
- ALWAYS guard a header file with `#pragma once`
- Avoid #define, prefer constexpr
- Avoid raw pointers, prefer smart pointers (i.e shared_ptr) or references  

### Indentation
Use tabs to indent, and use additional spaces to align multi-line code (i.e. when lines are too long and
have to be wrapped).
