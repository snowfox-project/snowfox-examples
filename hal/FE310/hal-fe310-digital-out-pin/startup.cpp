#include <stdint.h>

#include <algorithm>

extern "C" void __start(void) __attribute__ ((noreturn));

void init_bss();
void init_data();
void preinit_array();
void init_array();
void fini_array();

extern "C" void __start(void)
{

  /* Clear the BSS segment */
  init_bss();

  /* Perform the ROM-to-RAM initialisation */
  init_data();

  /* Perform preinit/init steps - call static ctors */
  preinit_array();
  init_array();

  /* Jump to main */
  asm volatile("call main");

  /* In case we should return from main perform deinitialisation - calling static dtors */
  fini_array();

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

typedef void(*PreInitFuncType)(void);

extern PreInitFuncType __preinit_array_start[];
extern PreInitFuncType __preinit_array_end[];

void preinit_array()
{
  std::for_each(__preinit_array_start,
                __preinit_array_end,
                [](PreInitFuncType const func)
                {
                  func();
                });
}

typedef void(*InitFuncType)(void);

extern InitFuncType __init_array_start[];
extern InitFuncType __init_array_end[];

void init_array()
{
  std::for_each(__init_array_start,
                __init_array_end,
                [](InitFuncType const func)
                {
                  func();
                });
}

typedef void(*FiniFuncType)(void);

extern FiniFuncType __fini_array_start[];
extern FiniFuncType __fini_array_end[];

void fini_array()
{
  std::for_each(__fini_array_start,
                __fini_array_end,
                [](FiniFuncType const func)
                {
                  func();
                });
}
