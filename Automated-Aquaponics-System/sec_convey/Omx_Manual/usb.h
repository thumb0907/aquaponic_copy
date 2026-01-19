#ifndef USB_H
#define USB_H

#include <Arduino.h>

// USB(Serial)로 들어오는 "ANG,xx.xx\n" 라인을 파싱해서 angle 값을 얻는 모듈

// 사용할 시리얼 포트 지정(보통 Serial)
void usb_begin(Stream &port);

// 수신 버퍼를 갱신하고, 새 angle이 들어오면 true 반환
// outAngleDeg : 파싱된 각도(deg)
bool usb_poll_angle(float &outAngleDeg);

// 마지막으로 성공 파싱된 angle 값 (없으면 0.0f)
float usb_last_angle(void);

// 마지막 파싱 시각(ms)
uint32_t usb_last_rx_ms(void);

#endif
