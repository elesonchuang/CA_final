.data
    n: .word 10
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    # Save return address and n on stack
    addi sp, sp, -8
    sw x1, 4(sp)
    sw a0, 0(sp)
    
    addi a2, x0, 2
    addi a7, x0, 8

    addi t2, x0, 2# Set tem2 to 2
    bge a0, t2, L1# If n >= tem2, go to L1
    addi t0, x0, 4# Else, set T(1) to 4 
    addi sp, sp, 8# Pop stack
    addi x10,t0,0
    jalr x0, 0(x1)# Return

L1:
    div a0, a0, a2# n = n/2
    jal x1, FUNCTION# Call recursive T(n/2)
    lw a0, 0(sp)# restore caller's n
    lw x1, 4(sp)# restore caller's return address
    addi sp, sp, 8# Pop stack
    mul t0, t0, a2# T(n/2) = T(n/2) * 2
    mul x20, a0, a7# tem4 = n * 8
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