/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */ 
#include "stdio.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

enum {
	OD_BOXES = 0,
	OD_LABELS,
	OD_SCORES,
	OD_NUMS,
};

extern status_t MODEL_Init(void);
extern uint8_t* MODEL_Run();
extern void MODEL_RunOD();
extern void MODEL_AllocateTensor(void* tensorArena, uint32_t size);
extern float* MODEL_GetODTensorByIdx(uint32_t idx);