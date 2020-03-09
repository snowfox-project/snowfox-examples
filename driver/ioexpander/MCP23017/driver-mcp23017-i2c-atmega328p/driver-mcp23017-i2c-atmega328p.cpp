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
 * INCLUDE
 **************************************************************************************/

#include <avr/io.h>

#include <snowfox/hal/avr/ATMEGA328P/CriticalSection.h>
#include <snowfox/hal/avr/ATMEGA328P/InterruptController.h>

#include <snowfox/blox/hal/avr/ATMEGA328P/I2cMaster.h>

#include <snowfox/driver/ioexpander/MCP23017/MCP23017.h>
#include <snowfox/driver/ioexpander/MCP23017/MCP23017_IoI2c.h>
#include <snowfox/driver/ioexpander/MCP23017/MCP23017_Control.h>
#include <snowfox/driver/ioexpander/MCP23017/MCP23017_Configuration.h>

/**************************************************************************************
 * NAMESPACES
 **************************************************************************************/

using namespace snowfox;
using namespace snowfox::hal;
using namespace snowfox::driver;

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint8_t const MCP23017_I2C_ADDR = (0x10 << 1);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int snowfox_main()
{
  /************************************************************************************
   * HAL
   ************************************************************************************/

  ATMEGA328P::InterruptController int_ctrl    (&EIMSK, &PCICR, &PCMSK0, &PCMSK1, &PCMSK2, &WDTCSR, &TIMSK0, &TIMSK1, &TIMSK2, &UCSR0B, &SPCR, &TWCR, &EECR, &SPMCSR, &ACSR, &ADCSRA);
  ATMEGA328P::CriticalSection     crit_sec    (&SREG);

  blox::ATMEGA328P::I2cMaster     i2c_master  (&TWCR,
                                               &TWDR,
                                               &TWSR,
                                               &TWBR,
                                               int_ctrl,
                                               hal::interface::I2cClock::F_100_kHz);

  /* GLOBAL INTERRUPT *****************************************************************/
  int_ctrl.enableInterrupt(ATMEGA328P::toIntNum(ATMEGA328P::Interrupt::GLOBAL));


  /************************************************************************************
   * DRIVER
   ************************************************************************************/

  /* MCP2307 **************************************************************************/
  ioexpander::MCP23017::MCP23017_IoI2c          mcp23017_i2c    (MCP23017_I2C_ADDR, i2c_master());
  ioexpander::MCP23017::MCP23017_Configuration  mcp23017_config (mcp23017_i2c);
  ioexpander::MCP23017::MCP23017_Control        mcp23017_control(mcp23017_i2c);
  ioexpander::MCP23017::MCP23017                mcp23017        (mcp23017_config, mcp23017_control);

  ioexpander::MCP23017::IoctlConfigInputArg     mcp23017_pa0_config_input_arg (ioexpander::MCP23017::interface::Port::A, ioexpander::MCP23017::interface::Pin::IO0, ioexpander::MCP23017::interface::PullUpMode::Enabled);
  ioexpander::MCP23017::IoctlConfigOutputArg    mcp23017_pb7_config_output_arg(ioexpander::MCP23017::interface::Port::B, ioexpander::MCP23017::interface::Pin::IO7);


  /************************************************************************************
   * APPLICATION
   ************************************************************************************/

  mcp23017.open();

  mcp23017.ioctl(ioexpander::MCP23017::IOCTL_CONFIG_INPUT,  static_cast<void *>(&mcp23017_pa0_config_input_arg ));
  mcp23017.ioctl(ioexpander::MCP23017::IOCTL_CONFIG_OUTPUT, static_cast<void *>(&mcp23017_pb7_config_output_arg));

  for(;;)
  {
    /* Reading from a GPIO pin */
    bool is_pa0_set = false;
    ioexpander::MCP23017::IoctlGetInputPinArg mcp23017_pa0_get_input_pin_arg(ioexpander::MCP23017::interface::Port::A, ioexpander::MCP23017::interface::Pin::IO0, &is_pa0_set);
    mcp23017.ioctl(ioexpander::MCP23017::IOCTL_GET_INPUT_PIN, static_cast<void *>(&mcp23017_pa0_get_input_pin_arg));

    /* Writing to a GPIO pin */
    bool set_pb7 = true;
    ioexpander::MCP23017::IoctlSetOutputPinArg mcp23017_pb7_set_output_pin_arg(ioexpander::MCP23017::interface::Port::B, ioexpander::MCP23017::interface::Pin::IO7, set_pb7);
    mcp23017.ioctl(ioexpander::MCP23017::IOCTL_SET_OUTPUT_PIN, static_cast<void *>(&mcp23017_pb7_set_output_pin_arg));
  }


  mcp23017.close();

  return 0;
}
