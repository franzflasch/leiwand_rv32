#define REGBYTES 4

.global vecs_init

.section .vectors
vecs_init:
    j asm_init
    j irq_vec

.section .init_sec
asm_init:
    la sp, _stack
    call c_env_init

irq_vec:
    addi	sp, sp, -REGBYTES * 32

	/* Save Context */
	sw	x1, 0x0(sp)
	sw	x2, 1 * REGBYTES(sp)
	sw	x3, 2 * REGBYTES(sp)
	sw	x4, 3 * REGBYTES(sp)
	sw	x5, 4 * REGBYTES(sp)
	sw	x6, 5 * REGBYTES(sp)
	sw	x7, 6 * REGBYTES(sp)
	sw	x8, 7 * REGBYTES(sp)
	sw	x9, 8 * REGBYTES(sp)
	sw	x10, 9 * REGBYTES(sp)
	sw	x11, 10 * REGBYTES(sp)
	sw	x12, 11 * REGBYTES(sp)
	sw	x13, 12 * REGBYTES(sp)
	sw	x14, 13 * REGBYTES(sp)
	sw	x15, 14 * REGBYTES(sp)
	sw	x16, 15 * REGBYTES(sp)
	sw	x17, 16 * REGBYTES(sp)
	sw	x18, 17 * REGBYTES(sp)
	sw	x19, 18 * REGBYTES(sp)
	sw	x20, 19 * REGBYTES(sp)
	sw	x21, 20 * REGBYTES(sp)
	sw	x22, 21 * REGBYTES(sp)
	sw	x23, 22 * REGBYTES(sp)
	sw	x24, 23 * REGBYTES(sp)
	sw	x25, 24 * REGBYTES(sp)
	sw	x26, 25 * REGBYTES(sp)
	sw	x27, 26 * REGBYTES(sp)
	sw	x28, 27 * REGBYTES(sp)
	sw	x29, 28 * REGBYTES(sp)
	sw	x30, 29 * REGBYTES(sp)
	sw	x31, 30 * REGBYTES(sp)

    call irq_handler

    /* restore context */
	lw	x1, 0x0(sp)
    lw	x2, 1 * REGBYTES(sp)
    lw	x3, 2 * REGBYTES(sp)
	lw	x4, 3 * REGBYTES(sp)
	lw	x5, 4 * REGBYTES(sp)
	lw	x6, 5 * REGBYTES(sp)
	lw	x7, 6 * REGBYTES(sp)
	lw	x8, 7 * REGBYTES(sp)
	lw	x9, 8 * REGBYTES(sp)
	lw	x10, 9 * REGBYTES(sp)
	lw	x11, 10 * REGBYTES(sp)
	lw	x12, 11 * REGBYTES(sp)
	lw	x13, 12 * REGBYTES(sp)
	lw	x14, 13 * REGBYTES(sp)
	lw	x15, 14 * REGBYTES(sp)
	lw	x16, 15 * REGBYTES(sp)
	lw	x17, 16 * REGBYTES(sp)
	lw	x18, 17 * REGBYTES(sp)
	lw	x19, 18 * REGBYTES(sp)
	lw	x20, 19 * REGBYTES(sp)
	lw	x21, 20 * REGBYTES(sp)
	lw	x22, 21 * REGBYTES(sp)
	lw	x23, 22 * REGBYTES(sp)
	lw	x24, 23 * REGBYTES(sp)
	lw	x25, 24 * REGBYTES(sp)
	lw	x26, 25 * REGBYTES(sp)
	lw	x27, 26 * REGBYTES(sp)
	lw	x28, 27 * REGBYTES(sp)
	lw	x29, 28 * REGBYTES(sp)
	lw	x30, 29 * REGBYTES(sp)
	lw	x31, 30 * REGBYTES(sp)

    addi	sp, sp, REGBYTES * 32

    /* This will trigger the CPU to return to the normal program PC 
       this is just a hack and is not a standard risc-v solution 
       Altough the csrrwi instruction could possibly be used for
       this.
       */
    csrrwi x0, 0x0, 0x0
