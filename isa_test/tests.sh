tests_ui=(
lui
auipc
jal
jalr
beq
bne
blt
bge
bltu
bgeu
lb
lh
lw
lbu
lhu
sb
sh
sw
addi
slti
sltiu
xori
ori
andi
slli
srli
srai
add
sub
sll
slt
sltu
xor
srl
sra
or
and
# #fence
# #fence_i
# #ecall
# #ebreak
# #csrrw
# #csrrs
# #csrrc
# #csrrwi
# #csrrsi
# #csrrci
)

if [ "${ARCH}" = "64" ]; then
    tests_ui+=(
        addiw
        slliw
        srliw
        sraiw
        addw
        subw
        sllw
        srlw
        sraw
        ld
        lwu
        sd
    )
fi;


tests_ua=(
    amomax_w
)

