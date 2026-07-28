#include "SunSTP23.h"

STP23 ::STP23(HardwareSerial *uartPort, int8_t rxPin, int8_t txPin, unsigned long baud)
	: _uartPort(uartPort), _rxPin(rxPin), _txPin(txPin), _baud(baud)
{
	this->initUART(); // 串口初始化
}

void STP23 ::readData(uint8_t ucData)
{
	static uint8_t rxBuffer[STP23_RX_BUFFER_SIZE];
	static uint8_t rxCnt = 0;
	static uint8_t crc = 0;
// i++ 返回原来的值，++i 返回加1后的值。
	rxBuffer[rxCnt++] = ucData;
	// 判断帧头，不是帧头则跳出
	//rxCnt+1了
	if (rxBuffer[0] != STP23_HEADER)
	{
		rxCnt = 0;
		return;
	}
	// 判断VERLEN帧，不是VERLEN则跳出
	if ((2 == rxCnt) && (rxBuffer[1] != STP23_VERLEN))
	{
		rxCnt = 0;
		return;
	}
	// 判断数据长度，未达到长度则跳出
	if (rxCnt < STP23_DATA_SIZE)
	{
		return;
	}
	else
	{
		packData.crc8 = rxBuffer[46]; // 雷达传过来的校验和
		crc = CalCRC8(rxBuffer, 46);
		// 校验通过
		 if (packData.crc8 == crc)
		//if (packData.crc8 == packData.crc8)
		{
			for (int i = 0; i < STP23_POINT_PER_PACK; i++)
			{
				packData.point[i].distance = rxBuffer[6 + 3 * i] + rxBuffer[7 + 3 * i] * 256;
				packData.point[i].intensity = rxBuffer[8 + 3 * i];
			}
			dataProcess();
		}

		rxCnt = 0;
		// crc = 0;
	}
}

/**
 * @brief 初始化串口
 *
 */
void STP23::initUART()
{
	(*_uartPort).begin(_baud, SERIAL_8N1, _rxPin, _txPin);
	//(*_uartPort).setRxBufferSize(512);
}
uint8_t STP23::CalCRC8(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		crc = CrcTable[(crc ^ *p++) & 0xff];
	}
	return crc;
}
/**
 * @brief 数据处理函数，接受完整一帧后进行处理
 *
 */
void STP23::dataProcess()
{

	uint32_t sum = 0;
	uint8_t count = 0;
	for (int i = 0; i < STP23_POINT_PER_PACK; i++)
	{
		if (packData.point[i].distance != 0) // 去除距离为零的点
		{
			count++;
			sum += packData.point[i].distance;
		}
	}
	average_distance = sum / count;
}

// 清空缓冲区
void STP23::emptyCache()
{
	// 清空UART接收缓冲区
	while ((*_uartPort).available())
	{
		(*_uartPort).read();
	}
}