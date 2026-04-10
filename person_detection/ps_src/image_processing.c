// ==========================================================================
// image_processing.c: Image Preprocessing Implementation
// ==========================================================================

#include "image_processing.h"
#include "cam_interface.h"
#include "xil_printf.h"

// ======= 이미지 Reszie (선택 가능: Nearest Neighbor 또는 Bilinear) ========
// Resize 방식 선택
#define USE_BILINEAR 1    // 0: Nearest Neighbor (빠름), 1: Bilinear (품질 좋음)

// ----------------------------- Bilinear 방식 -----------------------------
#if USE_BILINEAR
static inline uint16_t bilinear_interpolate_rgb565(const uint16_t *src, int src_width, int x_int, int y_int, int x_frac, int y_frac) {
    // 4개 픽셀 읽기 (고정소수점 좌표의 주변 픽셀)
    uint16_t p00 = src[y_int * src_width + x_int];              // 좌상
    uint16_t p10 = src[y_int * src_width + x_int + 1];          // 우상
    uint16_t p01 = src[(y_int + 1) * src_width + x_int];        // 좌하
    uint16_t p11 = src[(y_int + 1) * src_width + x_int + 1];    // 우하

    // RGB565 -> R/G/B 분리
    int r00 = (p00 >> 11) & 0x1F, g00 = (p00 >> 5) & 0x3F, b00 = p00 & 0x1F;
    int r10 = (p10 >> 11) & 0x1F, g10 = (p10 >> 5) & 0x3F, b10 = p10 & 0x1F;
    int r01 = (p01 >> 11) & 0x1F, g01 = (p01 >> 5) & 0x3F, b01 = p01 & 0x1F;
    int r11 = (p11 >> 11) & 0x1F, g11 = (p11 >> 5) & 0x3F, b11 = p11 & 0x1F;

    // Bilinear 보간 (고정소수점 8bit fraction)
    int inv_x = 256 - x_frac;
    int inv_y = 256 - y_frac;

    int r = (((r00 * inv_x * inv_y) + (r10 * x_frac * inv_y) + (r01 * inv_x * y_frac) + (r11 * x_frac * y_frac)) >> 16) & 0x1F;
    int g = (((g00 * inv_x * inv_y) + (g10 * x_frac * inv_y) + (g01 * inv_x * y_frac) + (g11 * x_frac * y_frac)) >> 16) & 0x3F;
    int b = (((b00 * inv_x * inv_y) + (b10 * x_frac * inv_y) + (b01 * inv_x * y_frac) + (b11 * x_frac * y_frac)) >> 16) & 0x1F;

    // RGB565로 재조립
    return (r << 11) | (g << 5) | b;
}

int image_resize(const uint16_t *src_frame, uint16_t *dst_frame, int src_width, int src_height, int dst_width, int dst_height) {
    int dst_row, dst_col;

    if ((src_frame == NULL) || (dst_frame == NULL)) {
        xil_printf("[ERROR] Null pointer in resize\r\n");
        return -1;
    }

    xil_printf("[INFO] Resizing image (%dx%d -> %dx%d)...\r\n", src_width, src_height, dst_width, dst_height);

    // 고정소수점 스케일 팩터 (x256)
    const int scale_x = ((src_width - 1) << 8) / (dst_width - 1);
    const int scale_y = ((src_height - 1) << 8) / (dst_height - 1);

    for (dst_row = 0; dst_row < dst_height; dst_row++) {
        int src_y_fp = dst_row + scale_y;    // 고정소수점 y 좌표
        int src_y_int = src_y_fp >> 8;       // 정수 부분
        int src_y_frac = src_y_fp & 0xFF;    // 소수 부분

        // 경계 체크
        if (src_y_int >= src_height - 1) {
            src_y_int = src_height - 2;
            src_y_frac = 255;
        }

        for (dst_col = 0; dst_col < dst_width; dst_col++) {
            int src_x_fp = dst_col * scale_x;
            int src_x_int = src_x_fp >> 8;
            int src_x_frac = src_x_fp & 0xFF;

            if (src_x_int >= src_width - 1) {
                src_x_int = src_width - 2;
                src_x_frac = 255;
            }

            // Bilinear 보간
            dst_frame[dst_row * dst_width + dst_col] = bilinear_interpolate_rgb565(src_frame, src_width, src_x_int, src_y_int, src_x_frac, src_y_frac);
        }
    }

    xil_printf("[INFO] Resize complete\r\n");

    // 디버그: 리사이징 후 첫 몇 픽셀 출력
    xil_printf("[DEBUG] Resized image first 8 pixels:\r\n");
    for (int i=0; i<8; i++) {
        uint8_t r, g, b;
        rgb565_to_rgb888(dst_frame[i], &r, &g, &b);
        xil_printf("  [%d] 0x%04X -> R=%3d G=%3d B=%3d\r\n", i, dst_frame[i], r, g, b);
    }

    return 0;
}

// ------------------------- Nearest Neighbor 방식 --------------------------
#else
int image_resize(const uint16_t *src_frame, uint16_t *dst_frame, int src_width, int src_height, int dst_width, int dst_height) {
    int dst_row, dst_col;
    int src_row, src_col;

    if ((src_frame == NULL) || (dst_frame == NULL)) {
        xil_printf("[ERROR] Null pointer in resize\r\n");
        return -1;
    }

    xil_printf("[INFO] Resizing image (%dx%d -> %dx%d) using Nearest Neighbor...\r\n",
               src_width, src_height, dst_width, dst_height);

    // 고정소수점 스케일 팩터 (x256)
    const int scale_x = (src_width << 8) / dst_width;
    const int scale_y = (src_height << 8) / dst_height;

    for (dst_row = 0; dst_row < dst_height; dst_row++) {
        src_row = (dst_row * scale_y) >> 8;                                                         // 소스 행 계산

        for (dst_col = 0; dst_col < dst_width; dst_col++) {
            src_col = (dst_col * scale_x) >> 8;                                                     // 소스 열 계산

            dst_frame[dst_row * dst_width + dst_col] = src_frame[src_row * src_width + src_col];    // 소스 픽셀 복사
        }
    }

    xil_printf("[INFO] Resize complete\r\n");

    // 디버그: 리사이징 후 첫 몇 픽셀 출력
    xil_printf("[DEBUG] Resized image first 8 pixels:\r\n");
    for (int i=0; i<8; i++) {
        uint8_t r, g, b;
        rgb565_to_rgb888(dst_frame[i], &r, &g, &b);
        xil_printf("  [%d] 0x%04X -> R=%3d G=%3d B=%3d\r\n", i, dst_frame[i], r, g, b);
    }

    return 0;
}
#endif

// ==================== RGB565 -> Q4.12 변환 (3채널 분리) ====================
int image_rgb565_to_q412(const uint16_t *rgb565_frame, int16_t *q412_r, int16_t *q412_g, int16_t *q412_b, int width, int height) {
    if (!rgb565_frame || !q412_r || !q412_g || !q412_b) {
        xil_printf("[ERROR] Null pointer in Q4.12 conversion\r\n");
        return -1;
    }

    xil_printf("[INFO] Converting RGB565 to Q4.12 (%dx%d)...\r\n", width, height);

    int total_pixels = width * height;

    for (int i=0; i<total_pixels; i++) {
        uint16_t rgb565 = rgb565_frame[i];

        // RGB565 -> RGB888 추출
        uint8_t r5 = (rgb565 >> 11) & 0x1F;    // 5bit R
        uint8_t g6 = (rgb565 >> 5) & 0x3F;     // 6bit G
        uint8_t b5 = rgb565 & 0x1F;            // 5bit B

        // 8bit로 확장
        uint8_t r8 = (r5 << 3) | (r5 >> 2);    // 5bit -> 8bit
        uint8_t g8 = (g6 << 2) | (g6 >> 4);    // 6bit -> 8bit
        uint8_t b8 = (b5 << 3) | (b5 >> 2);    // 5bit -> 8bit

        // 정규화 (0-255 -> 0.0-1.0) 및 Q4.12 변환
        // q412 = (value / 255.0) * 4096 = value * (4096 / 255) = (value * 4096) / 255
        q412_r[i] = ((int32_t)r8 * 4096) / 255;
        q412_g[i] = ((int32_t)g8 * 4096) / 255;
        q412_b[i] = ((int32_t)b8 * 4096) / 255;
    }

    xil_printf("[INFO] Q4.12 conversion complete\r\n");

    // 디버그: 첫 몇 픽셀 Q4.12 값 출력
    xil_printf("[DEBUG] First 8 pixels (Q4.12):\r\n");
    for (int i=0; i<8; i++) {
        xil_printf("  [%d] R=0x%04X G=0x%04X B=0x%04X\r\n", i, (uint16_t)q412_r[i], (uint16_t)q412_g[i], (uint16_t)q412_b[i]);
    }

    return 0;
}
