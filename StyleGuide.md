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
- Never make threads. If you need to make a thread, talk to Sasha and he'll make it for you or tell you
  that you don't need one. 
