#include <stdio.h>

int main()
{
    int *tmp_ptr = (int *) 0x1000000;
    *tmp_ptr = 42;
    return 0;
}
