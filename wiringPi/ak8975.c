/*
 * ak8975.c:
 *	Extend wiringPi with the AK8975 I2C 3-axis Magnetometer
 *	Copyright (c) 2023 Chris Auslander
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://github.com/WiringPi/WiringPi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <byteswap.h>
#include <stdio.h>
#include <stdint.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "ak8975.h"

// Register map

#define STATUS1_REG         (0x02)
#define XAXIS_DATA_REG      (0x03)
#define YAXIS_DATA_REG      (0x05)
#define ZAXIS_DATA_REG      (0x07)
#define STATUS2_REG         (0x09)
#define CONTROL_REG         (0x0A)
#define SELF_TEST_REG       (0x0C)
#define XAXIS_SENSE_ADJ_REG (0x10)
#define YAXIS_SENSE_ADJ_REG (0x11)
#define ZAXIS_SENSE_ADJ_REG (0x12)

// Status 1 Bits

#define STATUS1_DATA_READY_MASK (0x01)

// Status 2 Bits

#define STATUS2_DATA_ERROR_MASK      (0x04)
#define STATUS2_SENSOR_OVERFLOW_MASK (0x08)

// Control Bits

#define CONTROL_MODE_MASK (0x0F)

// Self Test Bits

#define SELF_TEST_ENABLE_BIT  (6)
#define SELF_TEST_ENABLE_MASK (0x40)


/*
 * analogRead:
 *	Pin is the channel to sample on the device.
 *	Channels 0-2 are x-axis, y-axis, z-axis measurements. Returns INT16_MAX on fail.
 *	Channels 3-5 are x-axis, y-axis, z-axis sensitivity.
 *********************************************************************************
 */

static int myAnalogRead (struct wiringPiNodeStruct *node, int pin)
{
  int chan = pin - node->pinBase ;
  int16_t result ;
  uint8_t control ;
  uint8_t status ;
  uint8_t validRetries = 3 ;
  uint8_t readyRetries = 15;
  int validReading = FALSE ;
  uint8_t reg = XAXIS_DATA_REG ;

  // Ensure valid channel

  if (chan > 5) 
  {
    chan = 5 ;
  }

  // Determine target register

  switch (chan)
  {
    case 0: reg = XAXIS_DATA_REG      ; break ;
    case 1: reg = YAXIS_DATA_REG      ; break ;
    case 2: reg = ZAXIS_DATA_REG      ; break ;

    case 3: reg = XAXIS_SENSE_ADJ_REG ; break ;
    case 4: reg = YAXIS_SENSE_ADJ_REG ; break ;
    case 5: reg = ZAXIS_SENSE_ADJ_REG ; break ;
  }

  // Perform actions based on channel

  if (chan < 3)
  {
    // Read from a measurement register. Verify data is
    // ready and valid after read

    control = CONTROL_SINGLE_MEASURE_MODE ;
    wiringPiI2CWriteReg8 (node->fd, CONTROL_REG, control) ;

    // Check until valid reading or number of
    // retries is reached

    while (validReading == FALSE && validRetries)
    {
      --validRetries ;

      // Wait for data ready

      readyRetries = 15 ;
      while (readyRetries)
      {
        --readyRetries ;

        status = wiringPiI2CReadReg8 (node->fd, STATUS1_REG) ;
        if ((status & STATUS1_DATA_READY_MASK) != 0)
        {
          break ;
        }
        delayMicroseconds (250) ;
      }

      if (readyRetries == 0)
      {
        continue ;
      }

      // Verify valid reading

      status = wiringPiI2CReadReg8 (node->fd, STATUS2_REG) ;
      if ((status & 
            (STATUS2_DATA_ERROR_MASK | STATUS2_SENSOR_OVERFLOW_MASK)) != 0)
      {
        continue ;
      }
      validReading = TRUE ;
      
      // Read the low and high data registers for measurement

      result = wiringPiI2CReadReg16 (node->fd, reg) ; 
      result = __bswap_16(result) ;
    }

    if (validReading == FALSE)
    {
      // Set result to max positive value to indicate failure

      result = INT16_MAX ;
    }
  }
  else
  {
    // Read from a FUSE ROM register

    control = CONTROL_FUSE_ROM_ACCESS_MODE ;
    wiringPiI2CWriteReg8 (node->fd, CONTROL_REG, control) ;
    result = wiringPiI2CReadReg8 (node->fd, reg) ;
  }

  return result ;
}


/*
 * digitalRead:
 *	Pin is the channel to sample on the device.
 *	Channel 0 is the control register.
 *	Channel 1 is status 1. 
 *  Channel 2 is status 2.
 *********************************************************************************
 */

static int myDigitalRead (struct wiringPiNodeStruct *node, int pin)
{
  int chan = pin - node->pinBase ;
  uint16_t result ;
  uint8_t reg = CONTROL_REG ;

  // Ensure valid channel

  if (chan > 2) 
  {
    chan = 2 ;
  }

  // Determine target register

  switch (chan)
  {
    case 0: reg = CONTROL_REG ; break ;
    case 1: reg = STATUS1_REG ; break ;
    case 2: reg = STATUS2_REG ; break ;
  }

  // Read the selected register

  result = wiringPiI2CReadReg8 (node->fd, reg) ;

  return result ;
}


/*
 * digitalWrite:
 *	Channel 0 is the control register.
 *  Channel 1 is the self test. 0 for self-test disabled. 1 for self-test enabled.
 *********************************************************************************
 */

static void myDigitalWrite (struct wiringPiNodeStruct *node, int pin, int data)
{
  int chan = pin - node->pinBase ;

  // Ensure valid channel

  if (chan > 1) 
  {
    chan = 1 ;
  }

  // Read the selected register

  if (chan == 0)
  {
    wiringPiI2CWriteReg8 (node->fd, CONTROL_REG, (data & CONTROL_MODE_MASK)) ;
  }
  else
  {
    wiringPiI2CWriteReg8 (node->fd, SELF_TEST_REG, 
                          (data << SELF_TEST_ENABLE_BIT)) ;
  }
}


/*
 * ak8975Setup:
 *	Create a new wiringPi device node for an ak8975 on the Pi's
 *	I2C interface.
 *********************************************************************************
 */

int ak8975Setup (const int pinBase, int i2cAddr)
{
  struct wiringPiNodeStruct *node ;
  int fd ;

  if ((fd = wiringPiI2CSetup (i2cAddr)) < 0)
    return FALSE ;

  node = wiringPiNewNode (pinBase, 8) ;

  node->fd           = fd ;
  node->analogRead   = myAnalogRead ;
  node->digitalRead  = myDigitalRead ;
  node->digitalWrite = myDigitalWrite ;

  return TRUE ;
}