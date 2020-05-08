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
 * This example program is tailored for usage with SiFive HiFive 1 Rev. B which i
 * connected with a ST X-NUCLEO-NFC02A1 Arduino Shield which contains a M24LR04E-R
 * NFC EEPROM.
 *
 * Electrical interface:
 *   I2C_SDA = GPIO12
 *   I2C_SCL = GPIO13
 *
 * Program via
 *   JLinkExe -device FE310 -if JTAG -speed 4000 -jtagconf -1,-1 -autoconnect 1
 *   > loadfile hal-fe310-i2c.hex
 *   > exit
 **************************************************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cassert>

#include <snowfox/hal/riscv64/FE310/Io.h>
#include <snowfox/hal/riscv64/FE310/Clock.h>
#include <snowfox/hal/riscv64/FE310/Delay.h>
#include <snowfox/hal/riscv64/FE310/I2cMaster.h>

/**************************************************************************************
 * NAMESPACES
 **************************************************************************************/

using namespace snowfox::hal;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint32_t const HFXOSCIN_FREQ_Hz     =  16000000UL;
static uint32_t const CORECLK_FREQ_Hz      = 200000000UL;

static uint8_t  const M24LR_ADDRESS_SYSTEM = (0x57 << 1);
static uint8_t  const M24LR_ADDRESS_DATA   = (0x53 << 1);

static uint16_t const M24LR_REG_ICREF      = 0x091C;

static uint8_t  const M24LR_IC_REF         = 0x5A;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

uint8_t m24lr_readByte(uint16_t const reg_addr);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FE310::Delay     delay;
FE310::Clock     clock     (&PRCI_HFXOSCCFG, &PRCI_PLLCFG, &PRCI_PLLOUTDIV, HFXOSCIN_FREQ_Hz);
FE310::I2cMaster i2c_master(&I2C0_PRESC_LOW, &I2C0_PRESC_HIGH, &I2C0_CONTROL, &I2C0_DATA, &I2C0_CMD_STATUS, &GPIO0_IOF_EN, &GPIO0_IOF_SEL, CORECLK_FREQ_Hz);

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int snowfox_main()
{
  clock.setClockFreq(static_cast<uint8_t>(FE310::ClockId::coreclk), CORECLK_FREQ_Hz);

  i2c_master.setI2cClock(interface::I2cClock::F_100_kHz);

  /* Should be 0x5A for the M24LR04 mounted on the X-NUCLEO-NFC02A1 */
  uint8_t const m24lr_ref_id = m24lr_readByte(M24LR_REG_ICREF);
  assert(m24lr_ref_id == M24LR_IC_REF);

  for(;;) { }

  return 0;
}

/**************************************************************************************
 * FUNCTION IMPLEMENTATION
 **************************************************************************************/

uint8_t m24lr_readByte(uint16_t const reg_addr)
{
  if (!i2c_master.begin(M24LR_ADDRESS_SYSTEM, false)) {
    return 0;
  }

  uint8_t const reg_addr_msb = static_cast<uint8_t>((reg_addr & 0xFF00) / 256);
  uint8_t const reg_addr_lsb = static_cast<uint8_t>((reg_addr & 0x00FF));

  if (!i2c_master.write(reg_addr_msb)) {
    return 0;
  }

  if (!i2c_master.write(reg_addr_lsb)) {
    return 0;
  }

  uint8_t data = 0;
  if (!i2c_master.requestFrom(M24LR_ADDRESS_SYSTEM, &data, 1)) {
    return 0;
  }

  return data;
}
