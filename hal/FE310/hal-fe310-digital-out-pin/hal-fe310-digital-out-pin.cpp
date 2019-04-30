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
 * This example program is tailored for usage with SiFive HiFive 1 Rev. B
 *
 * Electrical interface:
 *   LED RED   = GPIO22
 *   LED GREEN = GPIO19
 *   LED BLUE  = GPIO21
 **************************************************************************************/

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <snowfox/hal/riscv64/FE310/Io.h>
#include <snowfox/hal/riscv64/FE310/Delay.h>
#include <snowfox/hal/riscv64/FE310/DigitalOutPin.h>

/**************************************************************************************
 * NAMESPACES
 **************************************************************************************/

using namespace snowfox::hal;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const LED_RED_GPIO_NUMBER   = 22;
static uint8_t const LED_GREEN_GPIO_NUMBER = 19;
static uint8_t const LED_BLUE_GPIO_NUMBER  = 21;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

FE310::Delay         delay;
FE310::DigitalOutPin led_red  (&GPIO0_INPUT_EN, &GPIO0_OUTPUT_EN, &GPIO0_IOF_EN, &GPIO0_OUTPUT_VAL, LED_RED_GPIO_NUMBER  );
FE310::DigitalOutPin led_green(&GPIO0_INPUT_EN, &GPIO0_OUTPUT_EN, &GPIO0_IOF_EN, &GPIO0_OUTPUT_VAL, LED_GREEN_GPIO_NUMBER);
FE310::DigitalOutPin led_blue (&GPIO0_INPUT_EN, &GPIO0_OUTPUT_EN, &GPIO0_IOF_EN, &GPIO0_OUTPUT_VAL, LED_BLUE_GPIO_NUMBER );

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main()
{
  for(uint8_t cnt = 0;; cnt++)
  {
    cnt = cnt % 8;
    
    switch(cnt)
    {
      case 0: led_blue.set(); led_green.set(); led_red.set(); break;
      case 1: led_blue.set(); led_green.set(); led_red.clr(); break;
      case 2: led_blue.set(); led_green.clr(); led_red.set(); break;
      case 3: led_blue.set(); led_green.clr(); led_red.clr(); break;
      case 4: led_blue.clr(); led_green.set(); led_red.set(); break;
      case 5: led_blue.clr(); led_green.set(); led_red.clr(); break;
      case 6: led_blue.clr(); led_green.clr(); led_red.set(); break;
      case 7: led_blue.clr(); led_green.clr(); led_red.clr(); break;
    }

    delay.delay_ms(100);
  }

  return 0;
}
