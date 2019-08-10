.section .init_sec

.global asm_init

asm_init:
    la sp, _stack
    call c_env_init
