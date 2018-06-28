/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "eag.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"

#include "deck_spi.h"
#include "deck_digital.h"
#include "deck_constants.h"
#include "deck_analog.h"
#include "sleepus.h"
#include "deck.h"
#include "stm32fxxx.h"
#define CS0 DECK_GPIO_IO4
#define EAG_TASK_NAME "EAG_TASK"
#define EAG_TASK_STACKSIZE 128
#define EAG_TASK_PRI 0

static bool isInit;

// State variables for the eag
// static setpoint_t setpoint;
// static sensorData_t sensorData;
// static state_t state;
// static control_t control;
static uint16_t eag_value;

static void eagTask(void* param);

void eagInit(void)
{
  if(isInit)
    return;

  adcInit();

  xTaskCreate(eagTask, EAG_TASK_NAME,
              EAG_TASK_STACKSIZE, NULL, EAG_TASK_PRI, NULL);

  isInit = true;
}

bool eagTest(void)
{
  bool pass = true;
  return pass;
}

/* The eag loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void eagTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  // vTaskSetApplicationTaskTag(0, (void*)TASK_EAG_ID_NBR); what the fuck does this do?

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  /* Enable clock for the peripheral of the pin.*/
  uint16_t pin = DECK_GPIO_RX2;
  RCC_AHB1PeriphClockCmd(deckGPIOMapping[pin-1].periph, ENABLE);

  /* Populate structure with RESET values. */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  /* Initialise the GPIO pin to analog mode. */
  GPIO_InitStructure.GPIO_Pin   = deckGPIOMapping[pin-1].pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  GPIO_Init(deckGPIOMapping[pin-1].port, &GPIO_InitStructure);

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(200));
  }
  // Initialize tick to something else then 0
  // tick = 1;
  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(200));

    // uint8_t data[2] = {0,0};
    // const uint8_t dummy[2] = {0,0};
    // spiBeginTransactionWithMode(SPI_MODE3, SPI_BAUDRATE_2MHZ);
    // digitalWrite(CS0, 0);
    // sleepus(50);
    // spiExchange(2, &dummy[0], &data[0]);
    // sleepus(50);
    // digitalWrite(CS0, 1);
    // spiEndTransaction();
    // eag_value = (data[0]<<6) || (data[1]>>2);
    // sleepus(50);
    //
    // eag_value += 1
    eag_value = analogRead(DECK_GPIO_RX2);

    // tick++;
  }
}

LOG_GROUP_START(eag)
LOG_ADD(LOG_UINT16, eag, &eag_value)
LOG_GROUP_STOP(eag)
