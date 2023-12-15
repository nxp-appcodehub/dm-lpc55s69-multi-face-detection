/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
	.section .rodata
	.global model_data
	.global model_data_end
		
model_data:
	.incbin "./eiq_lib/face_detect_64x64.tflite"
model_data_end: