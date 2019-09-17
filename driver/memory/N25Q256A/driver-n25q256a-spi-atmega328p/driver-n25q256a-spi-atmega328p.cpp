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
 * This example program is tailored for usage with Arduino Uno and Digilent
 * Pmod SF3 32 MB serial NOR flash N25Q256A breakout board.
 * 
 * ATTENTION: The N25Q256A must be operated at 3V3! (Seeeduino_v3.0 has an ATMEGA328P
 * + Arduino compatible pinout + can be operated either at 5V or at 3V3).
 *
 * Electrical interface:
 *   Pmod SF3 Pin (1) = ~CS  = D10 = PB2
 *   Pmod SF3 Pin (3) = MISO = D12 = PB4
 *   Pmod SF3 Pin (2) = MOSI = D11 = PB3
 *   Pmod SF3 Pin (4) = SCK  = D13 = PB5
 *
 * Upload via avrdude
 *   avrdude -p atmega328p -c avrisp2 -e -U flash:w:driver-n25q256a-spi-atmega328p
 **************************************************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <avr/io.h>

#include <snowfox/hal/avr/ATMEGA328P/Flash.h>
#include <snowfox/hal/avr/ATMEGA328P/DigitalInPin.h>
#include <snowfox/hal/avr/ATMEGA328P/DigitalOutPin.h>
#include <snowfox/hal/avr/ATMEGA328P/CriticalSection.h>
#include <snowfox/hal/avr/ATMEGA328P/InterruptController.h>

#include <snowfox/blox/hal/avr/ATMEGA328P/UART0.h>
#include <snowfox/blox/hal/avr/ATMEGA328P/SpiMaster.h>

#include <snowfox/blox/driver/serial/SerialUart.h>

#include <snowfox/driver/memory/N25Q256A/N25Q256A.h>
#include <snowfox/driver/memory/N25Q256A/N25Q256A_IoSpi.h>
#include <snowfox/driver/memory/N25Q256A/N25Q256A_Control.h>
#include <snowfox/driver/memory/N25Q256A/N25Q256A_Configuration.h>

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

static uint16_t                    const UART_RX_BUFFER_SIZE         = 0;
static uint16_t                    const UART_TX_BUFFER_SIZE         = 64;

static hal::interface::SpiMode     const N25Q256A_SPI_MODE           = hal::interface::SpiMode::MODE_0;
static hal::interface::SpiBitOrder const N25Q256A_SPI_BIT_ORDER      = hal::interface::SpiBitOrder::MSB_FIRST;
static uint32_t                    const N25Q256A_SPI_PRESCALER      = 16;       /* Arduino Uno Clk = 16 MHz -> SPI Clk = 1 MHz */

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool n25q256a_read (memory::NorDriver * n25q256a, uint8_t * read_buf, uint32_t const read_buf_size);
bool n25q256a_write(memory::NorDriver * n25q256a, uint8_t const * write_buf, uint32_t const write_buf_size);
bool n25q256a_erase(memory::NorDriver * n25q256a, uint32_t const erase_block_num);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main()
{
  /************************************************************************************
   * HAL
   ************************************************************************************/

  ATMEGA328P::Flash               flash;

  ATMEGA328P::InterruptController int_ctrl  (&EIMSK, &PCICR, &PCMSK0, &PCMSK1, &PCMSK2, &WDTCSR, &TIMSK0, &TIMSK1, &TIMSK2, &UCSR0B, &SPCR, &TWCR, &EECR, &SPMCSR, &ACSR, &ADCSRA);
  ATMEGA328P::CriticalSection     crit_sec  (&SREG);

  /* As the datasheet state: 'If SS is configured as an input and is driven low
   * while MSTR is set, MSTR will be cleared.'. This means that in this special
   * case where the CS pin is equal with SS pin we need to set it before configuring
   * the SPI interface.
   */
  ATMEGA328P::DigitalOutPin       n25q256a_cs  (&DDRB, &PORTB,        2); /* CS   = D10 = PB2 */
  ATMEGA328P::DigitalOutPin       n25q256a_sck (&DDRB, &PORTB,        5); /* SCK  = D13 = PB5 */
  ATMEGA328P::DigitalInPin        n25q256a_miso(&DDRB, &PORTB, &PINB, 4); /* MISO = D12 = PB4 */
  ATMEGA328P::DigitalOutPin       n25q256a_mosi(&DDRB, &PORTB,        3); /* MOSI = D11 = PB3 */

  n25q256a_cs.set();
  n25q256a_miso.setPullUpMode(hal::interface::PullUpMode::PULL_UP);

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
                                             N25Q256A_SPI_MODE,
                                             N25Q256A_SPI_BIT_ORDER,
                                             N25Q256A_SPI_PRESCALER);

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
  trace::Trace             trace              (serial_trace_output, trace::Level::Debug);

  /* N25Q256A *************************************************************************/
  memory::N25Q256A::N25Q256A_IoSpi         n25q256a_spi    (spi_master(), n25q256a_cs);
  memory::N25Q256A::N25Q256A_Configuration n25q256a_config (n25q256a_spi);
  memory::N25Q256A::N25Q256A_Control       n25q256a_control(n25q256a_spi);
  memory::N25Q256A::N25Q256A               n25q256a        (n25q256a_config, n25q256a_control);

  /************************************************************************************
   * APPLICATION
   ************************************************************************************/

  if(!n25q256a.open()) {
    trace.println(trace::Level::Error, "N25Q256A::open() failed");
  }

  memory::NorDriverCapabilities n25q256a_cap;
  if(!n25q256a.ioctl(memory::IOCTL_GET_CAPABILITIES, reinterpret_cast<void*>(&n25q256a_cap))) {
    trace.println(trace::Level::Error, "N25Q256A::ioctl(IOCTL_GET_CAPABILITIES) failed");
  } else {
    trace.println(trace::Level::Info, "N25Q256A read block size:        %d", n25q256a_cap.read_size);
    trace.println(trace::Level::Info, "N25Q256A prog block size:        %d", n25q256a_cap.prog_size);
    trace.println(trace::Level::Info, "N25Q256A erase block size:       %d", n25q256a_cap.erase_size);
    trace.println(trace::Level::Info, "N25Q256A number of erase blocks: %d", n25q256a_cap.erase_block_num);
  } 

  uint8_t read_buf[20] = {
    /* read address */
    0x00, 0x00, 0x00, 0xFF,
    /* actual read buffer */
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };

  uint8_t write_buf[20] = {
    /* read address */
    0x00, 0x00, 0x00, 0xFF,
    /* actual read buffer */
    0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF,
    0xAA, 0x55, 0xAA, 0x55,
    0xCA, 0xFF, 0xEE, 0xEE
  };

  for(;;)
  {
    /* READ ***************************************************************************/
    if(!n25q256a_read(&n25q256a, read_buf, 20)) {
      trace.println(trace::Level::Error, "Reading 16 bytes from N25Q256A failed");
    } else {
      trace.println(trace::Level::Info, "16 bytes successfully read from N25Q256A:");
      for(uint8_t i = 4; i < 20; i++) {
        trace.print(trace::Level::Info, "%02X ", read_buf[i]);
      }
      trace.println(trace::Level::Info);
    }
    /* WRITE **************************************************************************/
    if(!n25q256a_write(&n25q256a, write_buf, 20)) {
      trace.println(trace::Level::Error, "Writing 16 bytes to N25Q256A failed");
    }
    /* READ ***************************************************************************/
    if(!n25q256a_read(&n25q256a, read_buf, 20)) {
      trace.println(trace::Level::Error, "Reading 16 bytes from N25Q256A failed");
    } else {
      trace.println(trace::Level::Info, "16 bytes successfully read from N25Q256A:");
      for(uint8_t i = 4; i < 20; i++) {
        trace.print(trace::Level::Info, "%02X ", read_buf[i]);
      }
      trace.println(trace::Level::Info);
    }
    /* ERASE **************************************************************************/
    if(!n25q256a_erase(&n25q256a, 1)) {
      trace.println(trace::Level::Error, "Erasing of N25Q256A subsector '1' failed");
    }
    /* READ ***************************************************************************/
    if(!n25q256a_read(&n25q256a, read_buf, 20)) {
      trace.println(trace::Level::Error, "Reading 16 bytes from N25Q256A failedln");
    } else {
      trace.println(trace::Level::Info, "16 bytes successfully read from N25Q256A:");
      for(uint8_t i = 4; i < 20; i++) {
        trace.print(trace::Level::Info, "%02X ", read_buf[i]);
      }
      trace.println(trace::Level::Info);
    }
  }

  n25q256a.close();

  return 0;
}

/**************************************************************************************
 * FUNCTION IMPLEMENTATION
 **************************************************************************************/

bool n25q256a_read(memory::NorDriver * n25q256a, uint8_t * read_buf, uint32_t const read_buf_size)
{
  return (n25q256a->read(read_buf, static_cast<ssize_t>(read_buf_size)) == static_cast<ssize_t>(read_buf_size));
}

bool n25q256a_write(memory::NorDriver * n25q256a, uint8_t const * write_buf, uint32_t const write_buf_size)
{
  return (n25q256a->write(write_buf, static_cast<ssize_t>(write_buf_size)) == static_cast<ssize_t>(write_buf_size));
}

bool n25q256a_erase(memory::NorDriver * n25q256a, uint32_t const erase_block_num)
{
  return n25q256a->ioctl(memory::IOCTL_ERASE, (void*)&erase_block_num);
}
