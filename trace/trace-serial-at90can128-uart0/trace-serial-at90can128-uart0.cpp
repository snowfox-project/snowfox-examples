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
 * This example program is tailored for usage with Olimex AVR-CAN
 *
 * Upload via avrdude
 *   avrdude -p at90can128 -c avrisp2 -e -U flash:w:debug-serial-at90can128-uart0
 **************************************************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <avr/io.h>

#include <snowfox/hal/avr/AT90CAN128/CriticalSection.h>
#include <snowfox/hal/avr/AT90CAN128/InterruptController.h>

#include <snowfox/blox/hal/avr/AT90CAN128/UART0.h>

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

  AT90CAN128::InterruptController int_ctrl(&EIMSK, &TIMSK0, &TIMSK1, &TIMSK2, &TIMSK3, &UCSR0B, &UCSR1B, &CANGIE, &SPCR, &TWCR, &EECR, &SPMCSR, &ACSR, &ADCSRA);
  AT90CAN128::CriticalSection     crit_sec;

  blox::AT90CAN128::UART0         uart0   (&UDR0, &UCSR0A, &UCSR0B, &UCSR0C, &UBRR0, int_ctrl, F_CPU);


  /* DRIVER ***************************************************************************/

  blox::SerialUart serial(crit_sec,
                          uart0(),
                          UART_RX_BUFFER_SIZE,
                          UART_TX_BUFFER_SIZE,
                          serial::interface::SerialBaudRate::B115200,
                          serial::interface::SerialParity::None,
                          serial::interface::SerialStopBit::_1);


  /* GLOBAL INTERRUPT *****************************************************************/

  int_ctrl.enableInterrupt(AT90CAN32_64_128::toIntNum(AT90CAN128::Interrupt::GLOBAL));


  /* APPLICATION **********************************************************************/

  trace::SerialTraceOutput serial_trace_output(serial());
  trace::Trace             trace              (serial_trace_output,trace::Level::Debug);

  for(uint32_t cnt = 0;; cnt++)
  {
    trace.println(trace::Level::Debug, "( %08X ) Hello AT90CAN128", cnt);
  }

  return 0;
}
