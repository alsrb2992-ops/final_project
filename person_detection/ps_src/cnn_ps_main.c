// ============================================================
// cnn_ps_main.c: PS 소프트웨어 메인 (ZyboZ7-20)
// ------------------------------------------------------------
// 데이터 흐름:
//     OV7670 Camera -> DMA -> DDR (320x240 RGB565)
//     -> PS 읽기 및 전처리 (Resize + Q4.12 변환)
//     -> AXI GPIO -> PL CNN 가속기
//     -> AXI GPIO <- PL 결과 (13x13 probability map)
//     -> UART 전송
// ============================================================

#include "cam_interface.h"
#include "image_processing.h"

#include <stdio.h>
#include <stdint.h>
#include "xil_printf.h"
#include "xparameters.h"
#include "sleep.h"

// ========================= 전역 버퍼 =========================
static uint16_t cam_frame[CAM_FRAME_SIZE] __attribute__((aligned(32)));        // DDR에서 읽어온 원본 이미지 (RGB565, 16bit per pixel)
static uint16_t resized_frame[CNN_FRAME_SIZE] __attribute__((aligned(32)));    // Resize된 이미지 (RGB565, 16bit per pixel)

// ========================= 메인 함수 =========================
int main() {
    int status;

    xil_printf("\r\n");
    xil_printf("=============================================\r\n");
    xil_printf("CNN Person Detection - PS Software\r\n");
    xil_printf("---------------------------------------------\r\n");

    // ---------- Step 1: DDR에서 카메라 이미지 읽기 -----------
    xil_printf("\r\n[STEP 1] Reading camera frame from DDR...\r\n");
    status = cam_read_frame_from_ddr(cam_frame);
    if (status != 0) {
        xil_printf("[ERROR] Failed to read camera frame (status=%d)\r\n", status);
        return -1;
    }

    // ------ Step 2: 이미지 Resize (320x240 -> 128x128) -------
    xil_printf("\r\n[STEP 2] Image preprocessing (Resize)...\r\n");
    status = image_resize(cam_frame, resized_frame, CAM_WIDTH, CAM_HEIGHT, CNN_WIDTH, CNN_HEIGHT);
    if (status != 0) {
        xil_printf("[ERROR Failed to resize image\r\n");
        return -1;
    }

    xil_printf("\r\n[SUCCESS] Image resize complete!\r\n");
    xil_printf("=============================================\r\n");

    return 0;
}
