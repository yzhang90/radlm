# mark_description "Intel(R) C Intel(R) 64 Compiler XE for applications running on Intel(R) 64, Version 13.1.3.192 Build 2013060";
# mark_description "7";
# mark_description "-xAVX -O3 -c99 -c -fasm-blocks -S";
	.file "dwmonitor.c"
	.text
..TXTST0:
# -- Begin  dwmonitor
# mark_begin;
       .align    16,0x90
	.globl dwmonitor
dwmonitor:
# parameter 1: %rdi
# parameter 2: %rsi
..B1.1:                         # Preds ..B1.0
..___tag_value_dwmonitor.1:                                     #29.38
        pushq     %rsi                                          #29.38
..___tag_value_dwmonitor.3:                                     #
        vstmxcsr  (%rsp)                                        #35.24
        movl      (%rsp), %ecx                                  #35.24
                                # LOE rbx rbp rsi rdi r12 r13 r14 r15 ecx
..B1.2:                         # Preds ..B1.1
        movl      %ecx, %eax                                    #36.9
        xorl      %edx, %edx                                    #40.9
        andl      $-65536, %eax                                 #36.9
        orl       $57280, %eax                                  #36.9
        movl      %eax, (%rsp)                                  #36.9
        xorl      %eax, %eax                                    #
        vldmxcsr  (%rsp)                                        #36.9
        vbroadcastss (%rdi), %xmm0                              #38.63
        vxorpd    %xmm5, %xmm5, %xmm5                           #37.14
        vmovups   .L_2il0floatpacket.2(%rip), %xmm6             #38.41
        vmovapd   %xmm5, %xmm7                                  #37.14
        vmovupd   .L_2il0floatpacket.4(%rip), %xmm4             #41.55
        vmovupd   .L_2il0floatpacket.5(%rip), %xmm2             #46.117
        vaddsubps %xmm0, %xmm6, %xmm1                           #38.27
        vmovupd   .L_2il0floatpacket.3(%rip), %xmm0             #39.14
        vcvtps2pd %xmm1, %xmm3                                  #38.14
        .align    16,0x90
                                # LOE rax rdx rbx rbp rsi rdi r12 r13 r14 r15 ecx xmm0 xmm2 xmm3 xmm4 xmm5 xmm6 xmm7
..B1.3:                         # Preds ..B1.3 ..B1.2
        vmovddup  (%rax,%rsi), %xmm8                            #41.66
        incq      %rdx                                          #40.9
        vaddsubpd %xmm0, %xmm5, %xmm1                           #42.18
        vaddsubpd %xmm8, %xmm4, %xmm9                           #41.18
        vshufpd   $1, %xmm1, %xmm1, %xmm0                       #44.29
        addq      $8, %rax                                      #40.9
        vmulpd    %xmm9, %xmm1, %xmm13                          #43.18
        vmulpd    %xmm9, %xmm0, %xmm12                          #44.18
        vminpd    %xmm13, %xmm12, %xmm10                        #45.47
        vmaxpd    %xmm13, %xmm12, %xmm15                        #46.84
        vsubpd    %xmm10, %xmm5, %xmm11                         #45.18
        vmulpd    %xmm3, %xmm0, %xmm10                          #50.18
        vshufpd   $1, %xmm11, %xmm11, %xmm14                    #46.40
        vmulpd    %xmm3, %xmm1, %xmm11                          #49.18
        vmaxpd    %xmm15, %xmm14, %xmm8                         #46.29
        vminpd    %xmm11, %xmm10, %xmm0                         #51.48
        vmaxpd    %xmm11, %xmm10, %xmm13                        #52.86
        vaddpd    %xmm2, %xmm8, %xmm9                           #46.18
        vsubpd    %xmm0, %xmm5, %xmm1                           #51.19
        vaddpd    %xmm9, %xmm7, %xmm7                           #47.18
        vshufpd   $1, %xmm1, %xmm1, %xmm12                      #52.40
        vmaxpd    %xmm13, %xmm12, %xmm14                        #52.29
        vaddpd    %xmm2, %xmm14, %xmm0                          #52.18
        cmpq      $3, %rdx                                      #40.9
        jb        ..B1.3        # Prob 66%                      #40.9
                                # LOE rax rdx rbx rbp rsi rdi r12 r13 r14 r15 ecx xmm0 xmm2 xmm3 xmm4 xmm5 xmm6 xmm7
..B1.4:                         # Preds ..B1.3
        vbroadcastss 12(%rdi), %xmm3                            #57.67
        vbroadcastss 4(%rdi), %xmm1                             #56.67
        vaddsubps %xmm3, %xmm6, %xmm4                           #57.31
        vaddsubps %xmm1, %xmm6, %xmm2                           #56.31
        vcvtps2pd %xmm4, %xmm8                                  #57.18
        vcvtps2pd %xmm2, %xmm9                                  #56.18
        vbroadcastss 8(%rdi), %xmm2                             #56.67
        vbroadcastss 16(%rdi), %xmm4                            #57.67
        vshufpd   $1, %xmm8, %xmm8, %xmm10                      #58.34
        vaddpd    %xmm10, %xmm9, %xmm11                         #58.19
        vaddsubps %xmm2, %xmm6, %xmm3                           #56.31
        vaddsubps %xmm4, %xmm6, %xmm6                           #57.31
        vcvtps2pd %xmm3, %xmm8                                  #56.18
        vcvtps2pd %xmm6, %xmm4                                  #57.18
        vshufpd   $1, %xmm11, %xmm11, %xmm12                    #59.19
        vminpd    %xmm12, %xmm11, %xmm13                        #60.33
        vmaxpd    %xmm12, %xmm11, %xmm14                        #60.55
        vshufpd   $2, %xmm14, %xmm13, %xmm15                    #60.18
        vminpd    %xmm15, %xmm5, %xmm0                          #61.33
        vmaxpd    %xmm15, %xmm5, %xmm1                          #61.53
        vshufpd   $2, %xmm1, %xmm0, %xmm0                       #61.18
        vshufpd   $1, %xmm4, %xmm4, %xmm1                       #58.34
        vaddpd    %xmm1, %xmm8, %xmm2                           #58.19
        vshufpd   $1, %xmm2, %xmm2, %xmm3                       #59.19
        vminpd    %xmm3, %xmm2, %xmm6                           #60.33
        vmaxpd    %xmm3, %xmm2, %xmm9                           #60.55
        vshufpd   $2, %xmm9, %xmm6, %xmm10                      #60.18
        vminpd    %xmm10, %xmm0, %xmm11                         #61.33
        vmaxpd    %xmm10, %xmm0, %xmm0                          #61.53
        vshufpd   $2, %xmm0, %xmm11, %xmm12                     #61.18
        vaddsubpd %xmm12, %xmm5, %xmm13                         #63.15
        vaddsubpd %xmm7, %xmm5, %xmm5                           #64.15
        vshufpd   $1, %xmm5, %xmm5, %xmm7                       #65.33
        vcmpgepd  %xmm7, %xmm13, %xmm0                          #65.15
                                # LOE rbx rbp r12 r13 r14 r15 ecx xmm0
..B1.5:                         # Preds ..B1.4
# Begin ASM
        nop                                                     #67.15
# End ASM
                                # LOE rbx rbp r12 r13 r14 r15 ecx xmm0
..B1.6:                         # Preds ..B1.5
        vstmxcsr  (%rsp)                                        #68.13
        movl      (%rsp), %eax                                  #68.13
                                # LOE rbx rbp r12 r13 r14 r15 eax ecx xmm0
..B1.7:                         # Preds ..B1.6
        testb     $13, %al                                      #68.28
        je        ..B1.9        # Prob 50%                      #68.28
                                # LOE rbx rbp r12 r13 r14 r15 ecx xmm0
..B1.8:                         # Preds ..B1.7
        movl      %ecx, (%rsp)                                  #69.13
        movl      $-1, %eax                                     #70.20
        vldmxcsr  (%rsp)                                        #69.13
        popq      %rcx                                          #69.13
..___tag_value_dwmonitor.4:                                     #
        ret                                                     #69.13
..___tag_value_dwmonitor.5:                                     #
                                # LOE rbx rbp r12 r13 r14 r15 eax
..B1.9:                         # Preds ..B1.7
        movl      %ecx, (%rsp)                                  #72.9
        xorl      %ecx, %ecx                                    #66.15
        vpcmpeqd  %xmm1, %xmm1, %xmm1                           #66.15
        movl      $1, %edx                                      #66.15
        vldmxcsr  (%rsp)                                        #72.9
        xorl      %eax, %eax                                    #66.15
        vptest    %xmm1, %xmm0                                  #66.15
        cmovb     %edx, %eax                                    #66.15
        cmova     %edx, %ecx                                    #66.120
        subl      %ecx, %eax                                    #66.15
                                # LOE rbx rbp r12 r13 r14 r15 eax
..B1.10:                        # Preds ..B1.9
        popq      %rcx                                          #
..___tag_value_dwmonitor.6:                                     #
        ret                                                     #
        .align    16,0x90
..___tag_value_dwmonitor.7:                                     #
                                # LOE
# mark_end;
	.type	dwmonitor,@function
	.size	dwmonitor,.-dwmonitor
	.data
# -- End  dwmonitor
	.section .rodata, "a"
	.align 16
	.align 16
.L_2il0floatpacket.2:
	.long	0x00800000,0x00800000,0x00800000,0x00800000
	.type	.L_2il0floatpacket.2,@object
	.size	.L_2il0floatpacket.2,16
	.align 16
.L_2il0floatpacket.3:
	.long	0x00000000,0xbff00000,0x00000000,0x3ff00000
	.type	.L_2il0floatpacket.3,@object
	.size	.L_2il0floatpacket.3,16
	.align 16
.L_2il0floatpacket.4:
	.long	0x00000000,0x00200000,0x00000000,0x00200000
	.type	.L_2il0floatpacket.4,@object
	.size	.L_2il0floatpacket.4,16
	.align 16
.L_2il0floatpacket.5:
	.long	0x00000000,0x00100000,0x00000000,0x00100000
	.type	.L_2il0floatpacket.5,@object
	.size	.L_2il0floatpacket.5,16
	.data
	.section .note.GNU-stack, ""
// -- Begin DWARF2 SEGMENT .eh_frame
	.section .eh_frame,"a",@progbits
.eh_frame_seg:
	.align 8
	.4byte 0x00000014
	.8byte 0x7801000100000000
	.8byte 0x0000019008070c10
	.4byte 0x00000000
	.4byte 0x00000034
	.4byte 0x0000001c
	.8byte ..___tag_value_dwmonitor.1
	.8byte ..___tag_value_dwmonitor.7-..___tag_value_dwmonitor.1
	.byte 0x04
	.4byte ..___tag_value_dwmonitor.3-..___tag_value_dwmonitor.1
	.2byte 0x100e
	.byte 0x04
	.4byte ..___tag_value_dwmonitor.4-..___tag_value_dwmonitor.3
	.2byte 0x080e
	.byte 0x04
	.4byte ..___tag_value_dwmonitor.5-..___tag_value_dwmonitor.4
	.2byte 0x100e
	.byte 0x04
	.4byte ..___tag_value_dwmonitor.6-..___tag_value_dwmonitor.5
	.4byte 0x0000080e
	.2byte 0x0000
# End
