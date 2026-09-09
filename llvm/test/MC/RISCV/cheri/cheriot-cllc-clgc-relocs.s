# RUN: llvm-mc %s -triple=riscv32 -mcpu=cheriot -mattr=+xcheri,+xcheriot -riscv-no-aliases \
# RUN:     -show-encoding 2>&1 \
# RUN:     | FileCheck %s --implicit-check-not=fixup_riscv_pcrel_hi20 \
# RUN:                    --implicit-check-not=fixup_riscv_got_hi20 \
# RUN:                    --implicit-check-not=fixup_riscv_pcrel_lo12_i

# Regression test: cllc and clgc must emit the CHERIoT-specific
# 'fixup_riscv_cheriot_compartment_code_hi' relocation in CHERIoT ABI mode,
# not the standard PCREL_HI20 or GOT_HI20 (which the linker encodes with a
# 12-bit shift). AUIPCC and AUICGP both use an 11-bit shift (imm x 2048),
# so the linker must divide the offset by 2048 before writing imm20.
#
# Before the fix, 'cllc' used R_RISCV_PCREL_HI20 and 'clgc' used
# R_RISCV_GOT_HI20, causing the effective address to be off by a factor of 2
# (the linker wrote offset>>12 but hardware interprets imm20 as offset/2048).

sym:
        cllc ca0, sym
# The AUIPCC half must carry the 11-bit CHERIoT relocation, not pcrel_hi20.
# CHECK: fixup A - offset: 0, value: %cheriot_compartment_code_hi(sym), kind: fixup_riscv_cheriot_compartment_code_hi
# The paired CIncOffsetImm must use cheriot_compartment_lo_i (not pcrel_lo)
# so lld's rewriteCheriotLowRelocs can find and rewrite the pair.
# CHECK: fixup A - offset: 0, value: %cheriot_compartment_lo_i(

        clgc ca0, sym
# Same requirement for clgc: AUIPCC must use the CHERIoT 11-bit relocation.
# CHECK: fixup A - offset: 0, value: %cheriot_compartment_code_hi(sym), kind: fixup_riscv_cheriot_compartment_code_hi
# The paired CLC must also use cheriot_compartment_lo_i.
# CHECK: fixup A - offset: 0, value: %cheriot_compartment_lo_i(
