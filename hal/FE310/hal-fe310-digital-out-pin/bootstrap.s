/*
Based on entry.S and crt0.S from freedom-metal
*/

.section .text.init
.global _enter
.type _enter, @function

_enter:
  .cfi_startproc
  .cfi_undefined ra

  .option push
  .option norelax
  la gp, __global_pointer$ /* Load global pointer */
  .option pop

  /* Set up initial trap vector */
  la t0, early_trap_vector
  csrw mtvec, t0

  la sp, segment_stack_end
  andi sp, sp, -16

  /* Copy data segment */
  la t0, segment_data_source_start
  la t1, segment_data_target_start
  la t2, segment_data_target_end

  /* Skip if source and target are the same */
  beq t0, t1, 2f
  /* Skip copying if section is empty */
  bge t1, t2, 2f

1:
  lw   a0, 0(t0)
  addi t0, t0, 4
  sw   a0, 0(t1)
  addi t1, t1, 4
  blt  t1, t2, 1b
2:

  /* Copy itim segment */
  la t0, segment_itim_source_start
  la t1, segment_itim_target_start
  la t2, segment_itim_target_end

1:
  lw   a0, 0(t0)
  addi t0, t0, 4
  sw   a0, 0(t1)
  addi t1, t1, 4
  blt  t1, t2, 1b
2:

  /* Zero the BSS segment. */
  la t1, segment_bss_target_start
  la t2, segment_bss_target_end

  bge t1, t2, 2f

1:
  sw   x0, 0(t1)
  addi t1, t1, 4
  blt  t1, t2, 1b
2:

  /* Set atexit function */
  la a0, __libc_fini_array
  call atexit
  call __libc_init_array

  /* Call system init function */
  .weak _system_init
  la ra, _system_init
  beqz ra, 1f
  jalr ra
1:

  /* Call user init function */
  .weak _user_init
  la ra, _user_init
  beqz ra, 1f
  jalr ra
1:

  /* Set up C environment */
  li a0, 1
  la a1, argv
  la a2, envp
  call main

  call exit

  /* Trap infinitely */
  la t0, 1f
  csrw mtvec, t0
1:
  lw t1, 0(x0)
  j 1b

  .cfi_endproc

.global _init
.type   _init, @function
.global _fini
.type   _fini, @function
_init:
_fini:
  ret
.size _init, .-_init
.size _fini, .-_fini

.global _exit
.type   _exit. @function
_exit:
  .cfi_startproc
  j _exit
  .cfi_endproc

.align 2
early_trap_vector:
  .cfi_startproc
  j early_trap_vector
  .cfi_endproc

.weak __register_frame_info
.global __register_frame_info
__register_frame_info:
  .cfi_startproc
  ret
  .cfi_endproc

.section .rodata.init
argv:
  .dc.a name
envp:
  .dc.a

