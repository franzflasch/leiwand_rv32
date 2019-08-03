#include <stdio.h>

int x = 0;

int test_func(void)
{
    x++;
    return 42;
}

int main()
{
   test_func();

    x--;

   return 0;
}
