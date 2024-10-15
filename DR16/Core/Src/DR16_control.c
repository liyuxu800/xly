#include "stm32f4xx_hal.h"
#include "DR16_control.h"

/* ----------------------- Data Struct ------------------------------------- */
/* ----------------------- Internal Data ----------------------------------- */
volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; // double sbus rx buffer to save data
RC_Ctl_t RC_CtrlData;

/* ----------------------- Function Implements  ---------------------------- */

/******************************************************************************
 * @fn      RC_Init
 *
 * @brief   configure stm32 usart2 port
 *          -   USART Parameters
 *              -   100Kbps
 *              -   8-N-1
 *          -   DMA Mode
 *
 * @return  None.
 *
 * @note    This code is fully tested on STM32F405RGT6 Platform, You can port
it
 *          to the other platform. Using doube buffer to receive data prevent
losing data.
 */

// 此处官方实例代码为标准库的各个外设初始化，此处删除。

/******************************************************************************
 * @fn      RemoteDataProcess
 *
 * @brief   resolution rc protocol data.
 * @pData   a point to rc receive buffer.
 * @return  None.
 * @note    RC_CtrlData is a global variable.you can deal with it in other place.
 */
void RemoteDataProcess(uint8_t *pData) // 用于解析数据，pData指向接收缓冲区的指针，存储了从遥控器接收到的数据
{
    if (pData == NULL) 
    {
        return;
    }

    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                          ((int16_t)pData[4] << 10)) &
                         0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) &
                         0x07FF;

    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    RC_CtrlData.key.v = ((int16_t)pData[14]); // | ((int16_t)pData[15] << 8);

    
}

/******************************************************************************/

// 此处的官方源码是UART2的中断服务程序，主要关注空闲中断，利用DMA收发数据
