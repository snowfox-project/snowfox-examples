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
 * This example program is tailored for usage with Arduino Leonardo
 *
 * Upload via avrdude
 *   avrdude -p atmega32u4 -c avrisp2 -e -U flash:w:debug-serial-atmega32u4-uart1
 **************************************************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <avr/io.h>

#include <snowfox/hal/avr/ATMEGA32U4/CriticalSection.h>
#include <snowfox/hal/avr/ATMEGA32U4/InterruptController.h>

#include <snowfox/blox/hal/avr/ATMEGA32U4/UART1.h>

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

static uint16_t const UART_RX_BUFFER_SIZE =  0;
static uint16_t const UART_TX_BUFFER_SIZE = 16;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int snowfox_main()
{
  /* HAL ******************************************************************************/

  ATMEGA32U4::InterruptController int_ctrl(&EIMSK, &PCICR, &PCMSK0, &WDTCSR, &TIMSK0, &TIMSK1, &TIMSK3, &TIMSK4, &TCCR4D, &UCSR1B, &SPCR, &TWCR, &EECR, &SPMCSR, &ACSR, &ADCSRA);
  ATMEGA32U4::CriticalSection     crit_sec;

  blox::ATMEGA32U4::UART1         uart1   (&UDR1, &UCSR1A, &UCSR1B, &UCSR1C, &UBRR1, int_ctrl, F_CPU);


  /* DRIVER ***************************************************************************/

  blox::SerialUart serial(crit_sec,
                          uart1(),
                          UART_RX_BUFFER_SIZE,
                          UART_TX_BUFFER_SIZE,
                          serial::interface::SerialBaudRate::B115200,
                          serial::interface::SerialParity::None,
                          serial::interface::SerialStopBit::_1);


  /* GLOBAL INTERRUPT *****************************************************************/

  int_ctrl.enableInterrupt(ATMEGA16U4_32U4::toIntNum(ATMEGA32U4::Interrupt::GLOBAL));


  /* APPLICATION **********************************************************************/

  trace::SerialTraceOutput serial_trace_output(serial());
  trace::Trace             trace              (serial_trace_output,trace::Level::Debug);

  for(uint32_t cnt = 0;; cnt++)
  {
    trace.println(trace::Level::Debug, "( %08X ) Hello ATMEGA32U4", cnt);
  }

  return 0;
}
