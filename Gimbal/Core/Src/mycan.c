#include "stm32f4xx.h"

extern CAN_HandleTypeDef hcan1;

void FilterInit(void)       //配置过滤器
{
	CAN_FilterTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.FilterActivation = ENABLE;  //使能
	CAN_FilterInitStructure.FilterBank = 0;        //指定使用的过滤器银行编号
	CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;  //选择FIFO
	CAN_FilterInitStructure.FilterIdHigh =0x0000;       //设置要匹配的高十六位ID
	CAN_FilterInitStructure.FilterIdLow = 0x0000;       //设置要匹配的低十六位ID
	CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;   //设置高十六位掩码，给1为要匹配
	CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;  //设置低十六位掩码
	CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK; //过滤器模式，IDMASK为掩码模式
	CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;//设置ID长度
	CAN_FilterInitStructure.SlaveStartFilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);
	
	 if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure) != HAL_OK)  //判断是否接收到数据
    {
     //printf("CAN1 ConfigFilter Fail!!\r\n");
    }
    else
    {
      //printf("CAN1 ConfigFilter SUCCESS!!\r\n");
    }
}

void CAN1_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)   //can发送函数
{
	CAN_TxHeaderTypeDef TxMessage = {0};
	uint8_t Tx_Buffer[8] = {0};
	uint32_t box = 0;
	TxMessage.StdId = ID;   //设置标准ID
	TxMessage.ExtId = ID;   //设置扩展ID
	TxMessage.IDE = CAN_ID_STD;   // 设置标识符类型为标准ID
	TxMessage.RTR = CAN_RTR_DATA; // 设置RTR标志为数据帧
	TxMessage.DLC = Length;    // 设置数据长度码
	TxMessage.TransmitGlobalTime = DISABLE;   // 禁用全局时间传输
	for (uint8_t i = 0; i < Length; i ++)
	{
		Tx_Buffer[i] = Data[i];
	}

	HAL_StatusTypeDef TransmitMailbox;    //判断是否发送成功
	TransmitMailbox = HAL_CAN_AddTxMessage(&hcan1,&TxMessage,Tx_Buffer,&box);
	if(TransmitMailbox != HAL_OK )
	{
		//printf("Transmit Error!");
	}
	else
	{
		//printf("Transmit Success!\r\n");
	}
}

void CAN1_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
//		FilterInit();
	
		CAN_RxHeaderTypeDef rceStu = {0};
		uint8_t data[8] = {0};
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))    //判断FIFO中是否有数据
		{
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rceStu, data)==HAL_OK) //如果接收到数据
		{
    //printf("rceStu.DLC: %d\r\n",rceStu.DLC);
    //printf("rceStu.ExtId: %d\r\n",rceStu.ExtId);
    //printf("rceStu.StdId: %x\r\n",rceStu.StdId);
    //printf("rceStu.Timestamp: %d\r\n",rceStu.Timestamp);
		for(uint8_t i = 0;i<rceStu.DLC;i++)
    {
      //printf(" %x",data[i]);
		}
     //printf("\r\n");
		}
		}
		else
		{
			//printf("No CAN1 INFO!\r\n");
		}
}

