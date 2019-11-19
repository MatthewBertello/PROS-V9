#pragma once

#ifndef mathFunctions_cpp
#define mathFunctions_cpp

#include "main.h"

int sgn( float a)
{
  if(a > 0)
  {
    return 1;
  }
  else if(a < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

const long double PI = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998;

#endif
