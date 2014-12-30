#ifdef SUPPORT_LONGLONG

void Print::println(int64_t number, uint8_t base)
{
  print(number, base);
  println();
}

void Print::print(int64_t number, uint8_t base)
{
  if (number < 0)
  {
    write('-');
    number = -number;
  }
  print((uint64_t)number, base);
}

void Print::println(uint64_t number, uint8_t base)
{
  print(number, base);
  println();
}

void Print::print(uint64_t number, uint8_t base)
{
  unsigned char buf[64];
  uint8_t i = 0;

  if (number == 0) 
  {
    print((char)'0');
    return;
  }
 
  if (base < 2) base = 2;
  else if (base > 16) base = 16;

  while (number > 0) 
  {
    uint64_t q = number/base;
    buf[i++] = number - q*base;
    number = q;
  }
  for (; i > 0; i--) 
    write((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}
#endif