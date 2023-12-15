/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "stdio.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_inputmux.h"

#include "fsl_power.h"

#include "lcd.h"
#include "ov7670.h"
#include "tflite_bundle.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PINFUNC_CAMERA 15 /* Pin function for Camera engine to be configured by ARM CPU*/

#define EOL "\r\n"
#define GRAY_IMG_SIZE (28)
#define ZOOM (4)
#define FILL_MUL (3) // One point fills three points
#define RGB_SIZE (28 * ZOOM)

#define CAMERA_ZOOM (8) // image from camera is (28*8)*(28*8)

#define MODEL_IMG_SIZE (64 * 64 * 3)
#define MODEL_IMG_WIDTH (64)
#define CAMERA_IMG_WIDTH (128)
#define LCD_DISPLAY_IMG_WIDTH (CAMERA_IMG_WIDTH)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/**
 * @brief	camera initialization
 * @return	Nothing
 */
extern void Camera_Init(void);
/**
 * @brief	camera start
 * @return	Nothing
 */
extern void Camera_Start(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
volatile uint32_t DataReadyFlag = 0;
volatile bool handle_done = true;
uint16_t *g_CameraImageBuf = (uint16_t *)(0x4000000);
// uint8_t g_model_image[64*64*3] = {0};
uint16_t *g_buf_image = (uint16_t *)(0x20033000);
uint8_t *g_model_image = (uint8_t *)(0x3003C000);

void Reserved46_IRQHandler()
{
    //		DataReadyFlag = 1; // tell ARM data is ready from Camera engine
    if (handle_done == true)
    {
        memcpy((uint8_t *)g_buf_image, (uint8_t *)g_CameraImageBuf, 128 * 128 * 2);
        handle_done = false;
        DataReadyFlag = 1; // tell ARM data is ready from Camera engine
    }
};
/*!
 * @brief Main function
 */


int GetImage(void *ptr, uint32_t w, uint32_t h, uint32_t c)
{
    uint32_t len = w * h * c;
    memcpy(ptr, g_model_image, len);

    return 0;
}

#define SYSTICK_PRESCALE 1U
#define TICK_PRIORITY 1U
void TIMER_Init(void)
{
    uint32_t priorityGroup = NVIC_GetPriorityGrouping();
    SysTick_Config(CLOCK_GetFreq(kCLOCK_CoreSysClk) / (SYSTICK_PRESCALE * 1000U));
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(priorityGroup, TICK_PRIORITY, 0U));
}

volatile uint32_t msTicks;
int64_t TIMER_GetTimeInUS(void)
{
    int64_t us = ((SystemCoreClock / 1000) - SysTick->VAL) / (SystemCoreClock / 1000000);
    us += msTicks * 1000;
    return us;
}

void SysTick_Handler(void)
{
    msTicks++;
}

extern status_t MODEL_Init(void);
extern uint8_t *MODEL_Run();
extern void MODEL_AllocateTensor(void *tensorArena, uint32_t size);
static uint32_t tensorArena[160 * 1024 / 4] __ALIGNED(16) = {0};

#define OUTPUT_DIMS 10

#define OD_MODEL
uint32_t g_fastRAM[8 * 1024 / 4];
volatile uint32_t g_fastSize = sizeof(g_fastRAM);
void *conv_helper_enter(uint32_t ndx, uint32_t maxSizeInBytes, uint32_t *pAllocedSizeInBytes)
{
    pAllocedSizeInBytes[0] = g_fastSize;
    return (void *)g_fastRAM;
}

#define OUTPUT_DIMS 10

void display_model_image(uint8_t *model_img)
{
    //    lcd_stream_win((240-MODEL_IMG_WIDTH)/2,50,(240-MODEL_IMG_WIDTH)/2 + MODEL_IMG_WIDTH-1,MODEL_IMG_WIDTH-1+50,g_image_rgb565,MODEL_IMG_WIDTH*MODEL_IMG_WIDTH);
}
void clean_camera_img(uint16_t *camera_img)
{
    uint8_t *p = (uint8_t *)camera_img;
    memset(p + 120 * CAMERA_IMG_WIDTH * 2, 0x0, CAMERA_IMG_WIDTH * 8 * 2);
}

void get_model_img_from_camera(uint16_t *camera_img, uint8_t *model_img)
{
    uint32_t idx_camera, idx_model = 0;
    uint32_t x_camera, y_camera, x_model, y_model = 0;
    uint8_t R, G, B = 0;
    for (idx_camera = 0; idx_camera < CAMERA_IMG_WIDTH * CAMERA_IMG_WIDTH; idx_camera++)
    {
        x_camera = idx_camera % CAMERA_IMG_WIDTH;
        y_camera = (idx_camera - (idx_camera % CAMERA_IMG_WIDTH)) / CAMERA_IMG_WIDTH;
        if ((x_camera % 2 == 0) && (y_camera % 2 == 0))
        {
            x_model = x_camera / 2;
            y_model = y_camera / 2;
            idx_model = (y_model * MODEL_IMG_WIDTH + x_model) * 3;
            R = ((camera_img[idx_camera] >> 11) & 0x1F) << 3;
            G = ((camera_img[idx_camera] >> 5) & 0x3F) << 2;
            B = (camera_img[idx_camera] & 0x1F) << 3;
            //            idx_model = idx_camera * 3;
            model_img[idx_model++] = B;
            model_img[idx_model++] = G;
            model_img[idx_model] = R;
        }
    }
}

uint8_t lcd_printbuf[256] = {0};

int main(void)
{
    char ch;

    /* Define the init structure for the output pin*/
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput,
        1,
    };

    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    CLOCK_EnableClock(kCLOCK_InputMux); /* Enables the clock for the kCLOCK_InputMux block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);    /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* attach 12 MHz clock to FLEXCOMM4 (I2C master) */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM4);
    GPIO_PortInit(GPIO, 0);
    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);

    /* Init output GPIO. */
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 6, &gpio_config);
    CLOCK_AttachClk(kMAIN_CLK_to_CLKOUT);        // main clock source to clkout
    CLOCK_SetClkDiv(kCLOCK_DivClkOut, 3, false); // 150MHz / 3 = 50MHz clkout to XCLK of camera module

    /* Connect trigger sources to camera engine */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[0] = 13; // set p0_13 as VSYNC input function pin, every edge will be responded
    INPUTMUX->CAMERA_ENGINE_INPUTMUX[1] = 15; // set p0_15 as pixel input function pin, every edge will be responded
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    // configure camera interface pins
    IOCON->PIO[0][0] = PINFUNC_CAMERA | 1 << 8 | 1 << 10 | 2 << 4 | 1 << 6; // set p0_0 D0 on the camera port
    IOCON->PIO[0][1] = PINFUNC_CAMERA | 1 << 8 | 2 << 4 | 1 << 6;           // set p0_1 D1 on the camera port
    IOCON->PIO[0][2] = PINFUNC_CAMERA | 1 << 8 | 2 << 4 | 1 << 6;           // set p0_2 D2 on the camera port
    IOCON->PIO[0][3] = PINFUNC_CAMERA | 1 << 8 | 1 << 6;                    // set p0_3 D3 on the camera port
    IOCON->PIO[0][4] = PINFUNC_CAMERA | 1 << 8 | 1 << 6;                    // set p0_4 D4 on the camera port
    IOCON->PIO[0][5] = PINFUNC_CAMERA | 1 << 8 | 2 << 4 | 1 << 6;           // set p0_5 D5 on the camera port
    IOCON->PIO[0][6] = PINFUNC_CAMERA | 1 << 8;                             // set p0_6 D6 on the camera port
    IOCON->PIO[0][7] = PINFUNC_CAMERA | 1 << 8;                             // set p0_7 D7 on the camera port
    IOCON->PIO[0][18] = PINFUNC_CAMERA | 1 << 8 | 1 << 10;                  // P0_18 will toggle when camera engine receive every VSYNC dege

    /* configure P1_11 as output gpio for measuring the timing */
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 11, &gpio_config);
    /* configure P0_17 as output gpio for power up the camera moudule */
    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 0, 17, &gpio_config);
    GPIO_PinWrite(GPIO, 0, 17, 0);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    TIMER_Init();

    lcd_init(); // LCD initialization

    Ov7670_Init(0); // outside camera module initialization

    DisableIRQ(Reserved46_IRQn);    // Camera engine irq NUMBER 30
    CLOCK_EnableClock(kCLOCK_Ezhb); // enable camera engine clock
    Camera_Init();                  // camera engine initialization
    Camera_Start();                 // start camera engine
    EnableIRQ(Reserved46_IRQn);     // Camera engine irq NUMBER 30

    PRINTF("hello world.\r\n");

    MODEL_AllocateTensor((void *)tensorArena, sizeof(tensorArena));

    if (MODEL_Init() != kStatus_Success)
    {
        PRINTF("Failed initializing model" EOL);
        for (;;)
        {
        }
    }

    while (1)
    {
        while (DataReadyFlag == 0); // Waiting for respond from EZH, the waiting time is about 7ms

        GPIO_PinWrite(GPIO, 1, 11, 0); // toggle P1_11
        clean_camera_img(g_buf_image);
        get_model_img_from_camera(g_buf_image, g_model_image);

        uint16_t x_offset = (240 - LCD_DISPLAY_IMG_WIDTH) / 2;
        uint16_t y_offset = (320 - LCD_DISPLAY_IMG_WIDTH) / 2;

        DataReadyFlag = 0; // clear the flag
#define INPUT_W (64)
#define INPUT_H INPUT_W
        uint32_t model_run = TIMER_GetTimeInUS();
        MODEL_RunOD();
        uint32_t model_run_end = TIMER_GetTimeInUS();
        PRINTF("cost: %d ms\r\n", ((model_run_end - model_run) / 1000));
        sprintf((char *)lcd_printbuf, "Model time: %d ms", (model_run_end - model_run) / 1000);

        float *boxes, *scores, *labels, *objects;
        boxes = MODEL_GetODTensorByIdx(OD_BOXES);
        scores = MODEL_GetODTensorByIdx(OD_SCORES);
        labels = MODEL_GetODTensorByIdx(OD_LABELS);
        objects = (MODEL_GetODTensorByIdx(OD_NUMS));

        set_lcd_window(0, 0, 240, 320);
        lcd_clear_block(0, 250, LCD_COLOR_BLACK);

        int x1, y1, x2, y2;
        float score;
        float label;
        lcd_stream_win(x_offset, y_offset, x_offset + LCD_DISPLAY_IMG_WIDTH - 1, y_offset + LCD_DISPLAY_IMG_WIDTH - 1, g_buf_image, LCD_DISPLAY_IMG_WIDTH * LCD_DISPLAY_IMG_WIDTH);
        handle_done = true;
        for (int i = 0; i < (*objects); i++)
        {
            boxes += 4 * i;
            x1 = (int)(boxes[0] * INPUT_W + 0.5);
            y1 = (int)(boxes[1] * INPUT_H + 0.5);
            x2 = (int)(boxes[2] * INPUT_W + 0.5);
            y2 = (int)(boxes[3] * INPUT_H + 0.5);
            score = scores[i];
            label = labels[i];

            if (score > 0.6)
            {
                PRINTF("x1: %d, y1: %d, x2: %d, y2: %d\r\n", x1, y1, x2, y2);
                lcd_draw_rect(x_offset + x1 * 2, y_offset + y1 * 2, x_offset + (x1 - 1) * 2 + (x2 - x1 - 1) * 2 - 1, y_offset + (y1 - 1) * 2 + (y2 - y1 - 1) * 2 - 1, LCD_COLOR_GREEN);
                set_lcd_window(0, 0, 240, 320);
                lcd_clear_block(0, 250, LCD_COLOR_BLACK);
            }
            //            break;
        }
    }
    return 0;
}
