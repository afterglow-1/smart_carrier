#include "SunUPFLOW.h"

UPFLOW ::UPFLOW(HardwareSerial *uartPort, int8_t rxPin, int8_t txPin, unsigned long baud)
	: _uartPort(uartPort), _rxPin(rxPin), _txPin(txPin), _baud(baud)
{
	this->initUART(); // 串口初始化
}

uint8_t UPFLOW ::readData(uint8_t ucData)
{
	static uint8_t rxBuffer[FLOW_RX_BUFFER_SIZE];
	static uint8_t rxCnt = 0;
	//static uint8_t xor = 0;

	rxBuffer[rxCnt++] = ucData;
	// 判断帧头，不是帧头则跳出
	if (rxBuffer[0] != FLOW_HEADER )
	{
		rxCnt = 0;
		return UPFLOW_STATUS_WRONG_HEADER;
	}
	// 判断第二字节，不是FLOWLEN则跳出
	if ((2 == rxCnt) && (rxBuffer[1] != FLOWLEN))
	{
		rxCnt = 0;
		return UPFLOW_STATUS_WRONG_LEN;
	}
	// 判断数据长度，未达到长度则跳出
	if (rxCnt < FLOW_DATA_SIZE)
	{
		return UPFLOW_STATUS_SIZE_SHORT;
	}
	else
	{
		// 校验通过
		//还需添加校验位判断
		if (rxBuffer[13]==FLOW_FRAME_END)
		{			
				packData.flow_x_integral= rxBuffer[2] + rxBuffer[3] * 256;
				packData.flow_y_integral = rxBuffer[4] + rxBuffer[5] * 256;
				packData.integration_timespan= rxBuffer[6] + rxBuffer[7] * 256;
				packData.ground_distance= rxBuffer[8] + rxBuffer[9] * 256;
				packData.quality=rxBuffer[10] ;
				packData.version=rxBuffer[11] ;
		
		//低通滤波
		flow_x_integral_filtered=flow_x_LPF((float)packData.flow_x_integral);
		flow_y_integral_filtered=flow_y_LPF((float)packData.flow_y_integral);
		//累加 单位um
		x_Offset+=(flow_x_integral_filtered*FLOW_HEIGHT)/10;
		y_Offset+=(flow_y_integral_filtered*FLOW_HEIGHT)/10;

		rxCnt = 0;
		return UPFLOW_STATUS_SUCCESS;
		}
		else{
			rxCnt = 0;
			return UPFLOW_STATUS_WRONG_END;
		}
		
	}
}

/**
 * @brief 初始化串口
 *
 */
void UPFLOW::initUART()
{
	(*_uartPort).begin(_baud, SERIAL_8N1, _rxPin, _txPin);
	//(*_uartPort).setRxBufferSize(512);
}



// 清空缓冲区
void UPFLOW::emptyCache()
{
	// 清空UART接收缓冲区
	while ((*_uartPort).available())
	{
		(*_uartPort).read();
	}
}