#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#define MOTOR_UART   &huart1  
#define CMD_BUFFER_SIZE 64
#define HALL_PER_REV      390           // 30*13
#define WHEEL_DIAMETER_MM 80.0f
#define PI                3.14f
#define PERIOD_COUNT      50.0f         // 1秒内的20ms周期数

// 电机控制函数
void Motor_OpenLoop(uint8_t id, uint16_t pwm, uint16_t time);
void Motor_CloseLoop(uint8_t id, uint16_t pwm, uint16_t time);
void Motor_SetID(uint8_t old_id, uint8_t new_id);
void Motor_ReadID(uint8_t id);
void Motor_ReadVersion(uint8_t id);
void Motor_SetBaudrate(uint8_t id, uint8_t baud_index);
void Motor_EnableDebug(uint8_t id);
void Motor_SetSpeed(uint8_t id, float speed_mm_s, uint16_t time_ms);
void Motor_SpeedInit(uint8_t id);


// 调试信息解析函数
void Motor_ParseDebugMessage(const char *msg);

#endif
