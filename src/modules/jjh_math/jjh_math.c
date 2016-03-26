
static float _flt_inv_fact[] =
{
  1.0 / 1.0,                    // 1 / 1!
  1.0 / 6.0,                    // 1 / 3!
  1.0 / 120.0,                  // 1 / 5!
  1.0 / 5040.0,                 // 1 / 7!
  1.0 / 362880.0,               // 1 / 9!
  1.0 / 39916800.0,             // 1 / 11!
};

float cosf(float x)
{
  return sinf(x + M_PI_2);
}

float sinf(float x)
{
  float x_squared;
  float sin_x;
  size_t i;

  /* Move x to [-pi, pi) */

  x = fmodf(x, 2 * M_PI);
  if (x >= M_PI)
    {
      x -= 2 * M_PI;
    }

  if (x < -M_PI)
    {
      x += 2 * M_PI;
    }

  /* Move x to [-pi/2, pi/2) */

  if (x >= M_PI_2)
    {
      x = M_PI - x;
    }

  if (x < -M_PI_2)
    {
      x = -M_PI - x;
    }

  x_squared = x * x;
  sin_x = 0.0;

  /* Perform Taylor series approximation for sin(x) with six terms */

  for (i = 0; i < 6; i++)
    {
      if (i % 2 == 0)
        {
          sin_x += x * _flt_inv_fact[i];
        }
      else
        {
          sin_x -= x * _flt_inv_fact[i];
        }

      x *= x_squared;
    }

  return sin_x;
}

float fmodf(float x, float div)
{
  float n0;

  x /= div;
  x = modff(x, &n0);
  x *= div;

  return x;
}

float modff(float x, float *iptr)
{
  if (fabsf(x) >= 8388608.0)
    {
      *iptr = x;
      return 0.0;
    }
  else if (fabs(x) < 1.0)
    {
      *iptr = 0.0;
      return x;
    }
  else
    {
      *iptr = (float)(int)x;
      return (x - *iptr);
    }
}

float fabsf(float x)
{
  return ((x < 0) ? -x : x);
}

double fabs(double x)
{
  return ((x < 0) ? -x : x);
}

