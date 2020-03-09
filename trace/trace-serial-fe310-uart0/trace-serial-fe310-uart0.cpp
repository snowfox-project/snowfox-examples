/**
 * Snowfox is a modular RTOS with extensive IO support.
 * Copyright (C) 2017 - 2020 Alexander Entinger / LXRobotics GmbH
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**************************************************************************************
 * This example program is tailored for usage with SiFive HiFive 1 Rev. B
 *
 * Program via
 *   JLinkExe -device FE310 -if JTAG -speed 4000 -jtagconf -1,-1 -autoconnect 1
 *   > loadfile trace-serial-fe310-uart0.hex
 *   > exit
 **************************************************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <snowfox/hal/riscv64/FE310/Io.h>

#include <snowfox/hal/riscv64/FE310/Clock.h>
#include <snowfox/hal/riscv64/FE310/UART0.h>
#include <snowfox/hal/riscv64/FE310/CriticalSection.h>

#include <snowfox/blox/driver/serial/SerialUart.h>

#include <snowfox/trace/Trace.h>
#include <snowfox/trace/SerialTraceOutput.h>

/**************************************************************************************
 * NAMESPACES
 **************************************************************************************/

using namespace snowfox;
using namespace snowfox::hal;
using namespace snowfox::driver;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint32_t const HFXOSCIN_FREQ_Hz      =  16000000UL;
static uint32_t const CORECLK_FREQ_Hz       = 200000000UL;

static uint16_t const UART_RX_BUFFER_SIZE   =  0;
static uint16_t const UART_TX_BUFFER_SIZE   = 16;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int snowfox_main()
{
  /* HAL ******************************************************************************/

  FE310::Clock clock(&PRCI_HFXOSCCFG, &PRCI_PLLCFG, &PRCI_PLLOUTDIV, HFXOSCIN_FREQ_Hz);
  clock.setClockFreq(static_cast<uint8_t>(FE310::ClockId::coreclk), CORECLK_FREQ_Hz);

  FE310::CriticalSection crit_sec;

  FE310::UART0 uart0(&UART0_TXDATA,
                     &UART0_RXDATA,
                     &UART0_TXCTRL,
                     &UART0_RXCTRL,
                     &UART0_DIV,
                     CORECLK_FREQ_Hz,
                     &GPIO0_IOF_EN,
                     &GPIO0_IOF_SEL);

/*
  ATMEGA328P::InterruptController int_ctrl(&EIMSK, &PCICR, &PCMSK0, &PCMSK1, &PCMSK2, &WDTCSR, &TIMSK0, &TIMSK1, &TIMSK2, &UCSR0B, &SPCR, &TWCR, &EECR, &SPMCSR, &ACSR, &ADCSRA);
  ATMEGA328P::CriticalSection     crit_sec(&SREG);

  blox::ATMEGA328P::UART0         uart0   (&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0, int_ctrl, F_CPU);
*/

  /* DRIVER ***************************************************************************/

  blox::SerialUart serial(crit_sec,
                          uart0,
                          UART_RX_BUFFER_SIZE,
                          UART_TX_BUFFER_SIZE,
                          serial::interface::SerialBaudRate::B115200,
                          serial::interface::SerialParity::None,
                          serial::interface::SerialStopBit::_1);


  /* GLOBAL INTERRUPT *****************************************************************/
/*
  int_ctrl.enableInterrupt(ATMEGA328P::toIntNum(ATMEGA328P::Interrupt::GLOBAL));
*/

  /* APPLICATION **********************************************************************/

  trace::SerialTraceOutput serial_trace_output(serial());
  trace::Trace             trace              (serial_trace_output,trace::Level::Debug);

  for(uint32_t cnt = 0;; cnt++)
  {
    trace.println(trace::Level::Debug, "( %08X ) Hello SiFive FE310", cnt);
  }

  return 0;
}
