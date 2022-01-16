.data
n: .word 10
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
#    addi t0, x0, 0
 #   add t0, t0, a0
addi sp, sp, -8
sw x1, 4(sp)
    jal x1, recursive
    #addi a0, x0, 10
    add x10, x5, x0
    lw x1, 4(sp)
    add x10, x5, x0
    jalr x0, 0(x1)
    #ecall
recursive:  
    # Save return address and n on stack
    addi sp, sp, -8
    sw x1, 4(sp)
    sw a0, 0(sp)
    addi t2, x0, 1# Set tem2 to 1
    bgt a0, t2, L1# If n > tem2, go to L1
    addi t0, x0, 4# Else, set T(1) to 4 
    addi sp, sp, 8# Pop stack
    jalr x0, 0(x1)# Return

L1:
    srli a0, a0, 1# n = n/2
    jal x1, recursive# Call recursive T(n/2)
    lw a0, 0(sp)# restore caller's n
    lw x1, 4(sp)# restore caller's return address
    addi sp, sp, 8# Pop stack
    slli t0, t0, 1# T(n/2) = T(n/2) * 2
    slli x20, a0, 3# tem4 = n * 8
    addi x20, x20, 5# tem4 = tem4 + 5
    add t0, t0, x20# T(n) = 2T(n/2) + 8n + 5
    jalr x0, 0(x1)# Retrun

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall
    
