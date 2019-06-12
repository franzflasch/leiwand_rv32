
`define BITS_TO_FIT(N) ($clog2(N+1))

/* Gives the number of bits needed to represent the number given in the argument */
`define HIGH_BIT_TO_FIT(N) (`BITS_TO_FIT(N)-1)
