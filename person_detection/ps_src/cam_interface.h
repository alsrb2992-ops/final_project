// =================================================
// cam_interface.h: Camera/DDR interface (ZyboZ7-20)
// -------------------------------------------------
// 카메라 이미지 DDR 읽기 및 관리
// =================================================

#ifndef CAM_INTERFACE_H
#define CAM_INTERFACE_H

#include <stdint.h>

// DMA가 카메라 이미지를 저장하는 DDR 시작 주소
#define DDR_CAM_BASE 0x10000000

// 카메라 이미지 사양
#define CAM_WIDTH      320
#define CAM_HEIGHT     240
#define CAM_FRAME_SIZE (CAM_WIDTH * CAM_HEIGHT)    // 76,800 pixels

/**
  * @brief DDR에서 카메라 프레임 읽기
  *
  * @param frame_buff 읽어온 데이터를 저장할 버퍼 (최소 CAM_FRAME_SIZE 크기)
  * @return 0: 성공, -1: 실패
  */
int cam_read_frame_from_ddr(uint16_t *frame_buff);

/**
  * @brief RGB565 -> RGB888 변환
  *
  * @param rgb565 입력 RGB565 값
  * @param r 출력 R 채널 (8bit)
  * @param g 출력 G 채널 (8bit)
  * @param b 출력 B 채널 (8bit)
  */
void rgb565_to_rgb888(uint16_t rgb565, uint8_t *r, uint8_t *g, uint8_t *b);

#endif
