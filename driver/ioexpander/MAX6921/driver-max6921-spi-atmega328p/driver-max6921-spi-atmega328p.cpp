/**
 * Snowfox is a modular RTOS with extensive IO support.
 * Copyright (C) 2017 - 2019 Alexander Entinger / LXRobotics GmbH
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
 * INCLUDE
 **************************************************************************************/

#include <avr/io.h>

#include <snowfox/hal/avr/ATMEGA328P/Delay.h>
#include <snowfox/hal/avr/ATMEGA328P/DigitalOutPin.h>
#include <snowfox/hal/avr/ATMEGA328P/CriticalSection.h>
#include <snowfox/hal/avr/ATMEGA328P/InterruptController.h>

#include <snowfox/blox/hal/avr/ATMEGA328P/UART0.h>
#include <snowfox/blox/hal/avr/ATMEGA328P/SpiMaster.h>

#include <snowfox/blox/driver/serial/SerialUart.h>

#include <snowfox/driver/ioexpander/MAX6921/MAX6921.h>
#include <snowfox/driver/ioexpander/MAX6921/MAX6921_IoSpi.h>
#include <snowfox/driver/ioexpander/MAX6921/MAX6921_Control.h>

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

static uint16_t                    const UART_RX_BUFFER_SIZE           = 0;
static uint16_t                    const UART_TX_BUFFER_SIZE           = 64;

static hal::interface::SpiMode     const MAX6921_SPI_MODE              = hal::interface::SpiMode::MODE_0;
static hal::interface::SpiBitOrder const MAX6921_SPI_BIT_ORDER         = hal::interface::SpiBitOrder::MSB_FIRST;
static uint32_t                    const MAX6921_SPI_PRESCALER         = 16; /* Arduino Uno Clk = 16 MHz -> SPI Clk = 1 MHz */

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main()
{
  /************************************************************************************
   * HAL
   ************************************************************************************/

  ATMEGA328P::Delay               delay;

  ATMEGA328P::InterruptController int_ctrl(&EIMSK, &PCICR, &PCMSK0, &PCMSK1, &PCMSK2, &WDTCSR, &TIMSK0, &TIMSK1, &TIMSK2, &UCSR0B, &SPCR, &TWCR, &EECR, &SPMCSR, &ACSR, &ADCSRA);
  ATMEGA328P::CriticalSection     crit_sec(&SREG);

  /* As the datasheet state: 'If SS is configured as an input and is driven low
   * while MSTR is set, MSTR will be cleared.'. This means that in this special
   * case where the CS pin is equal with SS pin we need to set it before configuring
   * the SPI interface.
   */
  ATMEGA328P::DigitalOutPin       max6921_cs   (&DDRB, &PORTB, 2); /* CS   = D10 = PB2 */
  ATMEGA328P::DigitalOutPin       max6921_sck  (&DDRB, &PORTB, 5); /* SCK  = D13 = PB5 */
  ATMEGA328P::DigitalOutPin       max6921_mosi (&DDRB, &PORTB, 3); /* MOSI = D11 = PB3 */

  max6921_cs.set();

  /* Digital outputs for controlling the MAX6921 control pins 'LOAD' and 'BLANK' */
  ATMEGA328P::DigitalOutPin       max6921_load (&DDRB, &PORTB, 3); /* LOAD  = Dx = Py */
  ATMEGA328P::DigitalOutPin       max6921_blank(&DDRB, &PORTB, 3); /* BLANK = Dx = Py */


  blox::ATMEGA328P::UART0         uart0     (&UDR0,
                                             &UCSR0A,
                                             &UCSR0B,
                                             &UCSR0C,
                                             &UBRR0,
                                             int_ctrl,
                                             F_CPU);

  blox::ATMEGA328P::SpiMaster     spi_master(&SPCR,
                                             &SPSR,
                                             &SPDR,
                                             crit_sec,
                                             int_ctrl,
                                             MAX6921_SPI_MODE,
                                             MAX6921_SPI_BIT_ORDER,
                                             MAX6921_SPI_PRESCALER);

  /* GLOBAL INTERRUPT *****************************************************************/
  int_ctrl.enableInterrupt(ATMEGA328P::toIntNum(ATMEGA328P::Interrupt::GLOBAL));


  /************************************************************************************
   * DRIVER
   ************************************************************************************/

  /* SERIAL ***************************************************************************/
  blox::SerialUart serial(crit_sec,
                          uart0(),
                          UART_RX_BUFFER_SIZE,
                          UART_TX_BUFFER_SIZE,
                          serial::interface::SerialBaudRate::B115200,
                          serial::interface::SerialParity::None,
                          serial::interface::SerialStopBit::_1);

  trace::SerialTraceOutput serial_trace_output(serial());
  trace::Trace             trace              (serial_trace_output,trace::Level::Debug);

  /* MAX6921 **************************************************************************/
  ioexpander::MAX6921::MAX6921_IoSpi   max6921_io     (spi_master(), max6921_load, max6921_blank);
  ioexpander::MAX6921::MAX6921_Control max6921_control(max6921_io, delay);
  ioexpander::MAX6921::MAX6921         max6921        (max6921_control);


  /************************************************************************************
   * APPLICATION
   ************************************************************************************/

  uint8_t seg_num = 0;
  max6921.ioctl(ioexpander::MAX6921::IOCTL_SET_SEGMENT, static_cast<void *>(&seg_num));
  max6921.ioctl(ioexpander::MAX6921::IOCTL_WRITE      , 0                            );
  max6921.ioctl(ioexpander::MAX6921::IOCTL_LOAD       , 0                            );
  max6921.ioctl(ioexpander::MAX6921::IOCTL_NO_BLANK   , 0                            );

  for(;;)
  {
  }

  return 0;
}
