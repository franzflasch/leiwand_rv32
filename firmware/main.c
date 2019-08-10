#include <stdio.h>
#include <stdint.h>

typedef void (*funcp_t) (void);


extern unsigned _data_loadaddr, _sdata, _edata, _sbss, _ebss, _stack;


void busy_delay(int delay)
{
    uint32_t i = 0;

    for(i=0;i<delay;i++)
    {
        asm volatile ("nop\n\t") ;
    }
}

void __attribute__((noreturn, naked)) main(void)
{
    static uint32_t gpio_val = 2;
    static uint32_t *gpio_ptr = (uint32_t *) 0x30000000;

    while(1)
    {
        (*gpio_ptr) = gpio_val++;
        busy_delay(10000);
    }
}

void __attribute__((noreturn, naked)) c_env_init (void)
{
	volatile unsigned *src, *dest;
	funcp_t *fp;

	for (src = &_data_loadaddr, dest = &_sdata; dest < &_edata;src++, dest++) {
		*dest = *src;
	}

	for (dest = &_sbss; dest < &_ebss;dest++) {
		*dest = 0;
	}

    main();   
}
