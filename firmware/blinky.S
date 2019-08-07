start:
	li t0, 0x30000000
	lui t4,      0x2
	li t5,       0

reset:
	li t3,       0

repeat:
    addi t3, t3, 1
    beq t3, t4, toggle_led
	#lw t1,0(t0)
	j repeat
    
toggle_led:
	addi t5, t5, 1
	sw t5,0(t0)
    j reset