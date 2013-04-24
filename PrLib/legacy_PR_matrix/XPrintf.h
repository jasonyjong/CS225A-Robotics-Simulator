#ifndef XPRINTF_H
#define XPRINTF_H

// Override global printf with an application-specific function.

#include <cstdarg>
#include <cstdio>

#ifdef USING_XPRINTF
int XPrintf( const char* fmt, ... );
#define printf XPrintf
#endif //USING_XPRINTF

#endif // XPRINTF_H
