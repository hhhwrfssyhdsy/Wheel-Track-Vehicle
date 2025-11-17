#include "motor_ctrl.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

/**
 * @brief 发送一个字节
 */

void hhSerialSendByte(uint8_t Byte){
    HAL_UART_Transmit(&huart1,&Byte,1,HAL_MAX_DELAY);
}

/**
 * @brief  发送一条字符串指令到电机
 */
static void Motor_SendCommand(const char *cmd)
{
    for (uint16_t i=0;cmd[i]!='\0';i++){
        hhSerialSendByte(cmd[i]);
    }

}

/**
 * @brief  格式化为固定宽度字符串（补0）
 */
static void format_number(char *buf, uint16_t num, uint8_t width)
{
    sprintf(buf, "%0*u", width, num);
}

/**
 * @brief  开环控制
 * @param  id: 电机ID (0-254)
 * @param  pwm: PWM值 (0500-2500)
 * @param  time: 时间(0000-9999)，0000为持续
 */
void Motor_OpenLoop(uint8_t id, uint16_t pwm, uint16_t time)
{
    char cmd[CMD_BUFFER_SIZE];
    char id_str[4], pwm_str[5], time_str[5];

    format_number(id_str, id, 3);
    format_number(pwm_str, pwm, 4);
    format_number(time_str, time, 4);

    sprintf(cmd, "#%sP%sT%s!", id_str, pwm_str, time_str);
    Motor_SendCommand(cmd);
}

/**
 * @brief  闭环控制
 */
void Motor_CloseLoop(uint8_t id, uint16_t pwm, uint16_t time)
{
    char cmd[CMD_BUFFER_SIZE];
    char id_str[4], pwm_str[5], time_str[5];

    format_number(id_str, id, 3);
    format_number(pwm_str, pwm, 4);
    format_number(time_str, time, 4);

    sprintf(cmd, "#%sP%sB%s!", id_str, pwm_str, time_str);
    Motor_SendCommand(cmd);
}

/**
 * @brief  修改电机ID
 */
void Motor_SetID(uint8_t old_id, uint8_t new_id)
{
    char cmd[CMD_BUFFER_SIZE];
    char old_str[4], new_str[4];
    format_number(old_str, old_id, 3);
    format_number(new_str, new_id, 3);
    sprintf(cmd, "#%sPID%s!", old_str, new_str);
    Motor_SendCommand(cmd);
}

/**
 * @brief  读取电机ID
 */
void Motor_ReadID(uint8_t id)
{
    char cmd[CMD_BUFFER_SIZE];
    char id_str[4];
    format_number(id_str, id, 3);
    sprintf(cmd, "#%sPID!", id_str);
    Motor_SendCommand(cmd);
}

/**
 * @brief  读取版本号
 */
void Motor_ReadVersion(uint8_t id)
{
    char cmd[CMD_BUFFER_SIZE];
    char id_str[4];
    format_number(id_str, id, 3);
    sprintf(cmd, "#%sPVER!", id_str);
    Motor_SendCommand(cmd);
}

/**
 * @brief  设置波特率
 * @param  baud_index: 1~8 对应不同速率
 */
void Motor_SetBaudrate(uint8_t id, uint8_t baud_index)
{
    char cmd[CMD_BUFFER_SIZE];
    char id_str[4], baud_str[2];
    format_number(id_str, id, 3);
    sprintf(baud_str, "%d", baud_index);
    sprintf(cmd, "#%sPBD%s!", id_str, baud_str);
    Motor_SendCommand(cmd);
}

/**
 * @brief  解析调试信息
 * 示例输入: "+0840+008"
 */
void Motor_ParseDebugMessage(char *msg)
{
    int pwm_percent = 0;
    int pulse_count = 0;

    if (sscanf(msg, "+%d+%d", &pwm_percent, &pulse_count) == 2)
    {
        printf("调试信息: PWM=%d%%, 采样=%d 脉冲\r\n", pwm_percent, pulse_count);
    }
    else
    {
        printf("调试信息解析失败: %s\r\n", msg);
    }
}

void Motor_EnableDebug(uint8_t id)
{
    char cmd[CMD_BUFFER_SIZE];
    char id_str[4];
    format_number(id_str, id, 3);
    sprintf(cmd, "#%sPDebug!", id_str);
    Motor_SendCommand(cmd);
}