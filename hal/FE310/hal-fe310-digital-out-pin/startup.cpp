#include <stdint.h>

#include <algorithm>

extern "C" void __start(void) __attribute__ ((noreturn));

void init_bss();
void init_data();

extern "C" void __start(void)
{

  /* Clear the BSS segment */
  init_bss();

  /* Perform the ROM-to-RAM initialisation */
  init_data();

  /* Jump to main */
  asm volatile("call main");

  for(;;)
  {

  }
}

extern uintptr_t  __bss_start;
extern uintptr_t  __bss_end;

void init_bss()
{
  std::fill(&__bss_start, &__bss_end, 0U);
}

extern uintptr_t  __data_src_start;
extern uintptr_t  __data_dst_start;
extern uintptr_t  __data_dst_end;

void init_data()
{
  /* Calculate the size of the data section */
  std::size_t const cnt = (&__data_dst_end - &__data_dst_start);

  /* Copy the data initialisation code from flash to RAM */
  std::copy(&__data_src_start,
            &__data_src_start + cnt,
            &__data_dst_start);
}