/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file : part of VL53L1 Core and : dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document : strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/
/**
 * @file  vl53l1x_calibration.c
 * @brief Calibration functions implementation
 */
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"

#define ALGO__PART_TO_PART_RANGE_OFFSET_MM	0x001E
#define MM_CONFIG__INNER_OFFSET_MM			0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 			0x0022

int8_t VL53L1X_CalibrateOffset(VL53L1_Dev_t dev, uint16_t TargetDistInMm, int16_t *offset)
{
	uint8_t i = 0, tmp;
	int16_t AverageDistance = 0;
	uint16_t distance;
	VL53L1X_ERROR status = 0;

	status = VL53L1_WrWord(&dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
	status = VL53L1_WrWord(&dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	status = VL53L1_WrWord(&dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
	status = VL53L1X_StartRanging(dev);	/* Enable VL53L1X sensor */
	for (i = 0; i < 50; i++) {
		while (tmp == 0){
			status = VL53L1X_CheckForDataReady(dev, &tmp);
		}
		tmp = 0;
		status = VL53L1X_GetDistance(dev, &distance);
		status = VL53L1X_ClearInterrupt(dev);
		AverageDistance = AverageDistance + distance;
	}
	status = VL53L1X_StopRanging(dev);
	AverageDistance = AverageDistance / 50;
	*offset = TargetDistInMm - AverageDistance;
	status = VL53L1_WrWord(&dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset*4);
	return status;
}

int8_t VL53L1X_CalibrateXtalk(VL53L1_Dev_t dev, uint16_t TargetDistInMm, uint16_t *xtalk)
{
	uint8_t i, tmp= 0;
	float AverageSignalRate = 0;
	float AverageDistance = 0;
	float AverageSpadNb = 0;
	uint16_t distance = 0, spadNum;
	uint16_t sr;
	VL53L1X_ERROR status = 0;

	status = VL53L1_WrWord(&dev, 0x0016,0);
	status = VL53L1X_StartRanging(dev);
	for (i = 0; i < 50; i++) {
		while (tmp == 0){
			status = VL53L1X_CheckForDataReady(dev, &tmp);
		}
		tmp=0;
		status= VL53L1X_GetSignalRate(dev, &sr);
		status= VL53L1X_GetDistance(dev, &distance);
		status = VL53L1X_ClearInterrupt(dev);
		AverageDistance = AverageDistance + distance;
		status = VL53L1X_GetSpadNb(dev, &spadNum);
		AverageSpadNb = AverageSpadNb + spadNum;
		AverageSignalRate =
		    AverageSignalRate + sr;
	}
	status = VL53L1X_StopRanging(dev);
	AverageDistance = AverageDistance / 50;
	AverageSpadNb = AverageSpadNb / 50;
	AverageSignalRate = AverageSignalRate / 50;
	/* Calculate Xtalk value */
	*xtalk = (uint16_t)(512*(AverageSignalRate*(1-(AverageDistance/TargetDistInMm)))/AverageSpadNb);
	status = VL53L1_WrWord(&dev, 0x0016, *xtalk);
	return status;
}
