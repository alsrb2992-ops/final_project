// ====================================================
// cam_interface.c: Camera/DDR Interface Implementation
// ====================================================

#include "cam_interface.h"
#include "xil_printf.h"
#include "xil_cache.h"

// ============ DDR에서 카메라 프레임 읽기 ============
int cam_read_frame_from_ddr(uint16_t *frame_buff) {
    volatile uint16_t *ddr_ptr = (volatile uint16_t *)DDR_CAM_BASE;

    if (frame_buff == NULL) {
        xil_printf("[ERROR] Null frame buffer pointer\r\n");
        return -1;
    }

    xil_printf("[INFO] Reading camera frame from DDR @ 0x%08X\r\n", DDR_CAM_BASE);

    // DDR 캐시 무효화 - DMA가 쓴 데이터를 최신 상태로 읽기
    Xil_DCacheInvalidateRange(DDR_CAM_BASE, CAM_FRAME_SIZE * sizeof(uint16_t));

    // DDR -> 프레임 버퍼로 복사
    for (int i=0; i<CAM_FRAME_SIZE; i++) {
        frame_buff[i] = ddr_ptr[i];
    }

    xil_printf("[INFO] Camera frame read complete (%d pixels)\r\n", CAM_FRAME_SIZE);

    // 디버그: 첫 몇 픽셀 출력
    xil_printf("[DEBUG] First 8 pixels (RGB565):\r\n");
    for (int i=0; i<8; i++) {
        uint8_t r, g, b;
        rgb565_to_rgb888(frame_buff[i], &r, &g, &b);
        xil_printf("  [%d] 0x%04X -> R=%3d G=%3d B=%3d\r\n", i, frame_buff[i], r, g, b);
    }

    return 0;
}

// ============== RGB565 -> RGB888 변환 ===============
void rgb565_to_rgb888(uint16_t rgb565, uint8_t *r, uint8_t *g, uint8_t *b) {
    // RGB565 포맷: RRRR GGGG BBBB
    *r = (rgb565 >> 11) & 0x1F;    // 5bit R
    *g = (rgb565 >> 5) & 0x3F;     // 6bit G
    *b = rgb565 & 0x1F;            // 5bit B

    // 8bit로 스케일링 (왼쪽 시프트 후 MSB 복사)
    *r = (*r << 3) | (*r >> 2);    // 5bit -> 8bit
    *g = (*g << 2) | (*g >> 4);    // 6bit -> 8bit
    *b = (*b << 3) | (*b >> 2);    // 5bit -> 8bit
}
