// =======================================
// image_processing.h: Image Preprocessing
// ---------------------------------------
// Resize, Q4.12 변환 등 이미지 전처리 기능
// =======================================

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <stdint.h>

// CNN 입력 사양
#define CNN_WIDTH      128
#define CNN_HEIGHT     128
#define CNN_CHANNELS   3
#define CNN_FRAME_SIZE (CNN_WIDTH * CNN_HEIGHT)    // pixels

// Q4.12 고정소수점 포맷
#define Q4_12_SCALE 4096    // 16bit signed: 부호 1bit + 정수 3bit + 소수 12bit

/**
  * @brief 이미지 Resize
  * 구현 방식:
  *     - Bilinear Interpolation (기본, 품질 좋음, 느림)
  *     - Nearest Neighbor (선택, 빠름, 정수 연산만)
  *     image_processing.c에서 USE_BILINEAR 플래그로 선택
  *
  * @param src_frame 소스 이미지 (RGB565)
  * @param dst_frame 목적 이미지 (RGB565)
  * @param src_width 소스 가로
  * @param src_height 소스 세로
  * @param dst_width 목적 가로
  * @param dst_height 목적 세로
  * @return 0: 성공, -1: 실패
  */
int image_resize(const uint16_t *src_frame, uint16_t *dst_frame,
                 int src_width, int src_height, int dst_width, int dst_height);

/**
  * @brief RGB565 -> Q4.12 변환 (3채널 분리)
  *
  * @param rgb565_frame RGB565 이미지
  * @param q412_r R 채널 Q4.12 배열
  * @param q412_g G 채널 Q4.12 배열
  * @param q412_b B 채널 Q4.12 배열
  * @param width 이미지 가로
  * @param height 이미지 세로
  * @return 0: 성공, -1: 실패
  */

#endif
