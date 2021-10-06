#pragma once

/**
 * @file
 * @brief   Fixes for problematic macros defined by the Arduino Core.
 * 
 * The Arduino Core defines some macros for abs, min, max, etc. that cause
 * compilation problems when including the C++ standard library headers that
 * define these names as functions (as they should be defined).
 */

#ifdef ARDUINO
#include <Arduino.h>

#ifdef abs
#undef abs
#endif
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#ifdef round
#undef round
#endif

#ifdef F
#undef F
#endif

#ifdef B1
#undef B1
#endif

#ifdef B2
#undef B2
#endif

#ifdef B3
#undef B3
#endif

#endif