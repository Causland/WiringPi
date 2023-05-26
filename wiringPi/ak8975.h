/*
 * ak8975.h:
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

// Control mode definitions

#define CONTROL_POWER_DOWN_MODE      (0x00)
#define CONTROL_SINGLE_MEASURE_MODE  (0x01)
#define CONTROL_SELF_TEST_MODE       (0x08)
#define CONTROL_FUSE_ROM_ACCESS_MODE (0x0F)

#ifdef __cplusplus
extern "C" {
#endif

extern int ak8975Setup (int pinBase, int i2cAddress) ;

#ifdef __cplusplus
}
#endif