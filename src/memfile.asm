# riscvtest.asm
# Sarah.Harris@unlv.edu
# David_Harris@hmc.edu
# 5 April 2020
# Adapted from mipstest.asm
#
# Test the RISC-V processor.  
#  add, sub, and, or, slt, addi, lw, sw, beq, jal
# If successful, it should write the value 71 to address 84

#       RISC-V Assembly         Description             Address           Machine Code
main:   addi x2, x0, 5          # initialize x2 = 5     0                 00500113   
        addi x3, x0, 12         # initialize x3 = 12    4                 00c00193
        addi x7, x3, -9         # initialize x7 = 3     8                 ff718393
        or   x4, x7, x2         # x4 = (3 OR 5) = 7     C                 0023e233
        and  x5, x3, x4         # x5 = (12 AND 7) = 4   10                0041f2b3
        add  x5, x5, x4         # x5 = 4 + 7 = 11       14                004282b3
        beq  x5, x7, end        # shouldn't be taken    18                02728663
        slt  x4, x3, x4         # x4 = (12 < 7) = 0     1C                0041a233
        beq  x4, x0, around     # should be taken       20                00020463
        addi x5, x0, 0          # shouldn't happen      24                00000293
around: slt  x4, x7, x2         # x4 = (3 < 5) = 1      28                0023a233
        add  x7, x4, x5         # x7 = 1 + 11 = 12      2C                005203b3
        sub  x7, x7, x2         # x7 = 12 - 5 = 7       30                402383b3
        sw   x7, 68(x3)         # [80] = 7              34                0471a223
        lw   x2, 80(x0)         # x2 = [80] = 7         38                05002103
        jal  x3, end            # jump to end, x3 = 64  3C                008001ef
        addi x2, x0, 1          # shouldn't happen      40                00100113
end:    add  x2, x2, x3         # x2 = 7 + 64 = 71      44                00310133
        sw   x2, 84(x0)         # write mem[84] = 71    48                04202a23
        
        