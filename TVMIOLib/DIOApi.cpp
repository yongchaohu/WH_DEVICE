#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "DIOApi.h"
#include "SerialPort.h"


#define AFC_INVALID_RS232	 -1
#define afc_rs232_t          int

// 串口通讯数据单次传输最大长度
#define AFC_RS232_DEF_LEN	0x0440
// 串口通讯数据传输包最大长度
#define AFC_RS232_MAX_LEN	0x0880


#define iTYPE_2    2

#define iERROR_CODE_DIO_NOT_SUPPORT     0x4E20  

typedef unsigned char UCHAR;
typedef unsigned int  UINT;
typedef unsigned short USHORT;
typedef unsigned char BYTE;

int m_drvRS232 = -1;
/**
 * 发送消息缓冲区
 */
char DIO_m_outBuf[128] = { 0 };
/**
 * 接收消息缓冲区
 */
char DIO_m_inBuf[128] = { 0 };


UCHAR DIO_SOFT_VERSION[] = "DIO_20190304_DIO_V01";		//版本

/*-----------------------------------------
-----------------结构体--------------------
-------------------------------------------*/
//灯三色结构体
struct status_LED
{
	bool R;
	bool G;
	bool B; //保留，正常红、绿、黄(红绿同时开)不会修改，始终为0;
};

/*-----------------------------------------
-------定义各项类型,包括灯，维修门---------
-------------------------------------------*/
#define DIO_OUT_BUZZER 		0x0013		//蜂鸣器


#define DIO_COLOR_R			1			//红
#define DIO_COLOR_G			2			//绿
#define DIO_COLOR_Y			4			//黄

/**
 * 保存各项灯状态
 1.回收模块投票口信号灯R, G
 2.进站通行指示灯R, G
 3.出站通行指示灯R, G
 4.进站刷卡提示灯R, G
 5.出站刷卡提示灯R, G
 6.进站顶灯R, G, Y(R和G同时亮)
 7.出站顶灯R, G, Y(R和G同时亮)
 */
status_LED DIO_allLED[] =
{
	{0,		0,		0},
	{0,		0,		0},
	{0,		0,		0},
	{0,		0,		0},
	{0,		0,		0},
	{0,		0,		0},
	{0,		0,		0},
};

/*---------------------------------------------------------------------------*/
/*  	                       所有指令                                  	 */
/*---------------------------------------------------------------------------*/
long long DIO_Init_CMD = 0x0165000055AAF2ED;		//初始化命令
long long DIO_Init_Recv = 0x01650000AA55F35D;		//初始化命令正确返回
long long DIO_Get_ver = 0x016400000005B001;			//获取版本命令

/**
01 05 03 3e ff 00 ed b2 开纸币指示灯
01 05 03 3e 00 00 ac 42 关纸币指示灯
01 05 03 35 ff 00 9c 70 开照明灯
01 05 03 3a 00 00 ed 83 关照明灯
01 05 03 3b ff 00 fd b3 开投币指示灯
01 05 03 3b 00 00 bc 43 关投币指示灯
01 05 03 3c 00 00 0d 82 关取票指示灯
01 05 03 44 ff 00 cc 6b 开出票口电磁铁
01 05 03 44 00 00 8d 9b 关出票口电磁铁
01 05 03 42 ff 00 2c 6a 开硬币电磁铁
01 05 03 42 00 00 6d 9a 关硬币电磁铁
 */


long long DIO_LED_CMD[][5] =
{
	//		openR						openG				closeR					 closeG				B(保留)

	{0x01050356FF006C6E,		0x01050357FF003DAE,		0x0105035600002D9E,		0x0105035700007C5E,		0},			//回收模块投票口信号灯

	{0x01050341FF00DC6A,		0x01050342FF002C6A,		0x0105034100009D9A,		0x0105034200006D9A,		0},			//进站通行指示灯

	{0x01050353FF007C6F,		0x01050354FF00CDAE,		0x0105035300003D9F,		0x0105035400008C5E,		0},			//出站通行指示灯

	{0x0105033DFF001DB2,		0x0105033FFF00BC72,		0x0105033D00005C42,		0x0105033F0000FD82,		0},			//进站刷卡提示灯	

	{0x01050349FF005DA8,		0x01050351FF00DDAF,		0x0105034900001C58,		0x0105035100009C5F,		0},			//出站刷卡提示灯

	{0x0105033AFF00AC73,		0x0105033CFF004C72,		0x0105033A0000ED83,		0x0105033C00000D82,		0},			//进站顶灯

	{0x01050346FF006DAB,		0x01050348FF000C68,		0x0105034600002C5B,		0x0105034800004D98,		0}			//出站顶灯

};

#define DIO_HEAD 0x55
#define DIO_END  0xAA
#define DIO_DLE 0x10
#define DIO_QUERY 0x01
#define DIO_CONTROL 0x02

byte m_status[3] = { 0 };
//按位取Byte值
void DIO_GetByteBits(byte mByte, byte* pByte, int len)
{
	for (int i = 0; i < len; i++) {
		*pByte = ((mByte >> 7) & 1);
		mByte <<= 1;
		pByte++;
	}
}
byte DIO_BccCalc(byte * pData, WORD dwLength)
{
	byte Bcc = 0;
	while (dwLength-- != 0) {
		Bcc = Bcc ^ *pData++;
	}
	return Bcc;
}
int DIO_Escape(byte * data, const int& len)
{
	int i, j;

	byte* buf = (byte*) new byte[len * 2];
	memset(buf, 0x0, len * 2);

	for (i = 0, j = 0; i < len; ++i) {
		if (data[i] == DIO_HEAD || data[i] == DIO_END || data[i] == DIO_DLE) {
			buf[j++] = DIO_DLE;
			buf[j++] = data[i];
		}
		else {
			buf[j++] = data[i];
		}
	}
	memset(data, 0x0, len);
	memcpy(data, buf, j);
	delete[] buf;
	return j;
}
//接收到的数据体，移除转义
int DIO_Reverse_Escape(byte * data, const int& len)
{
	int i, j;

	byte* buf = (byte*) new byte[len * 2];
	memset(buf, 0x0, len * 2);

	for (i = 0, j = 0; i < len; ++i) {
		if ((data[i] == DIO_DLE)) {
			++i;
			buf[j++] = data[i];
		}
		else {
			buf[j++] = data[i];
		}
	}
	memset(data, 0x0, len);
	memcpy(data, buf, j);
	delete[] buf;
	return j;
}
/**
* 设置命令通用一次流程
* 设置命令的正确返回值，是他本身。
* 返回值:
*  0 - 执行成功; 1 - 执行错误;
*/

int DIO_Send_flow(long long sendCMD, UCHAR * recvCMD, int recvlength)
{
	int cmdLength = sizeof(sendCMD);
	memcpy(DIO_m_inBuf, "0", 128);
	memcpy(DIO_m_inBuf, &sendCMD, cmdLength);
	sp_write(DIO_m_inBuf, cmdLength);
	memcpy(DIO_m_outBuf, "0", 128);
	sp_read(DIO_m_outBuf, recvlength);
	if (0 != memcmp(DIO_m_outBuf, recvCMD, recvlength))
	{
		//LOG_ERROR("<DIO_Set_CMD_flow> Receive is err. Send CMD is %llx, receive is %llx", sendCMD, DIO_m_outBuf);
		return 1;
	}

	return 0;
}

/**
* 开关灯命令
* 返回值:
opeColor 操作的灯色
opeBit	 操作的哪个模块
open	 是否是打开操作
*  0 - 执行成功; 1 - 执行错误;
*/
int DIO_LED(UCHAR opeColor, UCHAR opeBit, bool open)
{
	int offset = open ? 0 : 2;
	UCHAR recv1[8], recv2[8];
	if (opeColor != 2)
	{
		memcpy(recv1, &DIO_LED_CMD[opeBit][opeColor + offset], 8);
		if (DIO_Send_flow(DIO_LED_CMD[opeBit][opeColor + offset], recv1, 8))
		{
			//	LOG_ERROR("<DIO_SetLampSignal> opeBit = %d, Color = %d. %s LED err.", opeBit, opeColor,open ? "Open" : "close");
			return 1;
		}
		opeColor == 0 ? DIO_allLED[opeBit].R = open : DIO_allLED[opeBit].G = open;
	}
	else
	{
		if (opeBit != 5 && opeBit != 6)
		{
			//	LOG_ERROR("<DIO_SetLampSignal>  opeBit = %d can not set Color == yellow", opeBit);
			return 1;
		}
		memcpy(recv1, &DIO_LED_CMD[opeBit][0 + offset], 8);
		memcpy(recv1, &DIO_LED_CMD[opeBit][1 + offset], 8);
		if (DIO_Send_flow(DIO_LED_CMD[opeBit][0 + offset], recv1, 8) ||
			DIO_Send_flow(DIO_LED_CMD[opeBit][1 + offset], recv2, 8))
		{
			//LOG_ERROR("<DIO_SetLampSignal> opeBit = %d, Color = %d. %s LED err.", opeBit, opeColor, open ? "Open" : "close");
			return 1;
		}
		DIO_allLED[opeBit].R = open;
		DIO_allLED[opeBit].G = open;
	}
	return 0;
}

/**
* 设置单个灯函数
*  pLampConfig - 设置信息
* 返回值:
*  0 - 执行成功; 1 - 输入出错; 2-执行出错
*/
int DIO_Set_LED(STDioLampConfig pLampConfig)
{
	char openLED = pLampConfig.ucOperType;
	UCHAR opeBit = 7, opeColor = 3, opeNum = pLampConfig.ucBlinkTimes;

	if (opeBit == 7)
	{
		//LOG_ERROR("<DIO_SetLampSignal> pLampConfig.usLampType set err!");
		return 1;
	}
	switch (pLampConfig.ucLampColor)
	{
	case DIO_COLOR_G: opeColor = 0;	break;
	case DIO_COLOR_R: opeColor = 1;	break;
	case DIO_COLOR_Y: opeColor = 2;	break;
	}
	if (opeColor == 3)
	{
		//LOG_ERROR("<DIO_SetLampSignal> pLampConfig.ucLampColor set err!");
		return 1;
	}
	if (openLED)			//开灯操作 
	{
		if (opeColor == 0 && DIO_allLED[opeBit].G == 1)
		{
			if (DIO_LED(1, opeBit, 0))
			{
				return 2;
			}
		}
		else if (opeColor == 1 && DIO_allLED[opeBit].R == 1)
		{
			if (DIO_LED(0, opeBit, 0))
			{
				return 2;
			}
		}
		if (opeNum)
		{
			clock_t start;
			while (opeNum--)
			{
				if (DIO_LED(opeColor, opeBit, 1))
				{
					return 2;
				}
				start = clock();

				while ((clock() - start) < 350);

				if (DIO_LED(opeColor, opeBit, 0))
				{
					return 2;
				}
			}
		}
		else
		{
			if (DIO_LED(opeColor, opeBit, 1))
			{
				return 2;
			}
		}

	}
	else					//关灯操作
	{
		if (DIO_LED(opeColor, opeBit, 0))
		{
			return 1;
		}
	}

	return 0;
}

/**
	* DIO模块打开
	*  uiCommPort - 串口号(非串口的DIO模块无意义)
	*  uiBaudRate - 串口通讯的波特率(非串口的DIO模块无意义)
	* 返回值:
	*  0 - 执行成功; 其他 - 执行失败;
	*/

int DIO_Open(int nNum, tSerialConfigInfo * pSerialConfigInfo)
{
	//LOG_DEBUG("start.");
	int iRet = iERROR_CODE_DIO_0;
	if (nNum <= 0) {
		return iERROR_CODE_DIO_OPEN_FAIL;
	}

	if (m_drvRS232 != -1)
		DIO_Close();

	m_drvRS232 = -1;
	//chSerialPort = 1;
	//nIoSpeed = 115200;
	//chParity = 'N';
	//nDatabits = 8;
	//nStopbits = 1;
	//LOG_DEBUG("<DIO_Open> befor fd=%d, SerialPort=%d", (int)m_drvRS232,pSerialConfigInfo->chSerialPort);

	m_drvRS232 = sp_open(pSerialConfigInfo->chSerialPort,	// 串口号
		pSerialConfigInfo->nIoSpeed,		// 波特率
		//1,									//0. tty  1.usb
		pSerialConfigInfo->nParity,			// 校验位(N/O/E)
		pSerialConfigInfo->nDatabits,		// 数据位
		pSerialConfigInfo->nStopbits,
		EV_RXCHAR);		// 停止位

	if (m_drvRS232 == 0)
	{
		/*LOG_DEBUG("<DIO_Open> openCom[%d, %d, %d, %d, %d] error!",
			pSerialConfigInfo->chSerialPort, pSerialConfigInfo->nIoSpeed,
			pSerialConfigInfo->nDatabits, pSerialConfigInfo->nStopbits, pSerialConfigInfo->nParity);*/
		m_drvRS232 = -1;
		return iERROR_CODE_DIO_OPEN_FAIL;
	}

	return iRet;
}

/**
	 * 初始化模块，检查模块是否可用
	 *  tReturn - 返回的模块状态信息
	 * 返回值:
	 *  0 - 执行成功; 其他 - 执行失败;
	 */

int DIO_Init(STDioDevReturn & tReturn)
{
	//	LOG_DEBUG("start.");
	int iRet = iERROR_CODE_DIO_0;
	/*UCHAR recvCMD[8];
	memcpy(recvCMD, &DIO_Init_Recv, 8);
	if (DIO_Send_flow(DIO_Init_CMD, recvCMD, 8))
	{
		tReturn.iType = 2;
		tReturn.iStdErrorCode = iERROR_CODE_DIO_INIT_FAIL;
		return iERROR_CODE_DIO_INIT_FAIL;
	}*/
	//LOG_DEBUG("end.");
	return iRet;
}
/**
 * DIO模块关闭
 * 返回值:
 *  0 - 执行成功; 其他 - 执行失败;
 */
int DIO_Close()
{
	//LOG_DEBUG("start.");
	if (m_drvRS232 != -1)
	{
		//sp_close();
		m_drvRS232 = -1;
	}
	sp_close();
	//	LOG_DEBUG("end.");
	return 0;
}

/**
 * 设置提示灯信号
 *  pLampConfig - 需要设置的灯信息
 *  ucLampCnt - 需要设置提示灯的数量（tDioLampConfig的个数）
 *  tReturn - 返回的模块状态信息
 * 返回值:
 *  0 - 执行成功; 其他 - 执行失败;
 */
int DIO_SetLampSignal(STDioLampConfig * pLampConfig, unsigned char ucLampCnt, STDioDevReturn & tReturn)
{
	//LOG_DEBUG("start.");
	int iRet = iERROR_CODE_DIO_0;
	if (!(ucLampCnt > 0))
	{
		tReturn.iType = 1;
		tReturn.iStdErrorCode = iERROR_CODE_DIO_PARAM_LAMP;
		return iERROR_CODE_DIO_PARAM_LAMP;
	}
	for (int i = 0; i < ucLampCnt; i++)
	{
		int errFlag = DIO_Set_LED(pLampConfig[i]);
		if (1 == errFlag)
		{
			tReturn.iType = 1;
			tReturn.iStdErrorCode = iERROR_CODE_DIO_PARAM_LAMP;
			return iERROR_CODE_DIO_PARAM_LAMP;
		}
		else if (2 == errFlag)
		{
			tReturn.iType = 2;
			tReturn.iStdErrorCode = iERROR_CODE_DIO_CONFIG_LAMP;
			return iERROR_CODE_DIO_CONFIG_LAMP;
		}
	}
	//LOG_DEBUG("end.");
	return iRet;
}
/**
 * 设置提示灯信号
 *  pSignalConfig - 需要设置的开关信号
 *  ucSignalCnt - 需要设置的开关信号的数量（tDioSignalConfig的个数）
 *  tReturn - 返回的模块状态信息
 * 返回值:
 *  0 - 执行成功; 其他 - 执行失败;
 */
int DIO_SetSwitchSignal(STDioSignalConfig * pSignalConfig, unsigned char ucSignalCnt, STDioDevReturn & tReturn)
{
	byte p_data[3] = { 0 };
	//把之前状态复制过来
	memcpy(p_data, m_status, sizeof(m_status));

	//LOG_DEBUG("start.");
	//LOG_ERROR("<DIO_SetSwitchSignal> Haven't is function, only get swirch signal");
	if (pSignalConfig == NULL)
	{
		tReturn.iType = 1;
		tReturn.iStdErrorCode = iERROR_CODE_DIO_NOT_SUPPORT;
	}
	for (int i = 0; i < ucSignalCnt; i++)
	{
		switch (pSignalConfig[i].usSignalType)
		{
		case IO_SIGNAL_PICKUPTICKET_INDICATOR:
			//取票找零指示灯
			if (pSignalConfig[i].ucOperType)
			{
				p_data[0] ^= 0x01;//00000001
			}
			else
			{
				p_data[0] &= 0xFE;//11111110
			}
			break;
		case IO_SIGNAL_COIN_OPERATIING_LIGHT:
			//硬币投币口指示灯
			if (pSignalConfig[i].ucOperType)
			{
				p_data[0] ^= 0x02;//00000010
			}
			else
			{
				p_data[0] &= 0xFD;//11111101
			}
			break;
		case IO_SIGNAL_BH_RECEIVING_LIGHT:
			//纸币投币口指示灯
			if (pSignalConfig[i].ucOperType)
			{
				p_data[0] ^= 0x40;//01000000
			}
			else
			{
				p_data[0] &= 0xBF;//10111111
			}
			break;
		case IO_SIGNAL_RECHARGE_LIGHT:
			//储值票指示灯
			if (pSignalConfig[i].ucOperType)
			{
				p_data[1] ^= 0x80;//10000000
			}
			else
			{
				p_data[1] &= 0x7F;//011111111
			}
			break;
		case IO_SIGNAL_PRINT_LIGHT:
			//凭条打印机
			if (pSignalConfig[i].ucOperType)
			{
				p_data[1] ^= 0x40;//01000000
			}
			else
			{
				p_data[1] &= 0xBF;//10111111
			}
			break;
		case IO_SIGNAL_BUZZER:
			//峰鸣器
			if (pSignalConfig[i].ucOperType)
			{
				p_data[1] ^= 0x10;//00010000
			}
			else
			{
				p_data[1] &= 0xEF;//11101111
			}
			break;
		case IO_SIGNAL_COIN_SLOT:
			//电磁铁
			if (pSignalConfig[i].ucOperType)
			{
				p_data[2] ^= 0x04;//00000100
			}
			else
			{
				p_data[2] &= 0xFB;//11111011
			}
		default:
			break;
		}
	}
	byte pWriteBuff[50] = { DIO_HEAD,0x04,DIO_CONTROL };
	int pWriteLength = 3;
	byte pEscapeBuff[10] = { 0 };
	memcpy(pWriteBuff + 3, p_data, sizeof(p_data));
	//计算BCC
	byte bcc = DIO_BccCalc(pWriteBuff, pWriteLength + sizeof(p_data));
	//把数据体转义
	memcpy(pEscapeBuff, p_data, sizeof(p_data));
	pEscapeBuff[3] = bcc;
	int escapeLength = DIO_Escape(pEscapeBuff, sizeof(p_data) + 1);
	pWriteLength += escapeLength;
	memcpy(pWriteBuff + 3, pEscapeBuff, escapeLength);
	pWriteBuff[pWriteLength] = DIO_END;
	pWriteLength++;

	int ret = 1;
	for (int i = 0; i < 3; i++)
	{
		//备份现有状态
		memcpy(m_status, p_data, sizeof(p_data));
		sp_write((char*)pWriteBuff, pWriteLength);
		byte readBuffer[20] = { 0 };
		sp_read((char*)readBuffer, sizeof(readBuffer));
		if (readBuffer[0] == DIO_HEAD && readBuffer[2] == 0x02 && readBuffer[4] == DIO_END)
		{
			ret = 0;
			break;
		}
	}
	if (ret == 0)
	{
		tReturn.iStdErrorCode = iERROR_CODE_DIO_CONFIG_SIGNAL;
		tReturn.iType = 2;
	}
	else
	{
		tReturn.iStdErrorCode = iERROR_CODE_DIO_0;
		tReturn.iType = 0;
	}
	//LOG_DEBUG("end");
	return ret;
}
/**
 * 获取开关信号
 *  pSignalConfig - 需要返回的开关信号配置结构（需要调用者开辟返回数据内存空间）
 *  ucSignalCnt - 返回开关信号的数量（返回tDioSignalConfig的个数，个数不超过传入的数量；当
					传入值为0时，返回所有的，调用方应保障调用前为pSignalConfig申请足够的空间）
 *  tReturn - 返回的模块状态信息
 * 返回值:
 *  0 - 执行成功; 其他 - 执行失败;
 */
int DIO_GetSwitchSignal(STDioSignalConfig * pSignalConfig, unsigned char& ucSignalCnt, STDioDevReturn & tReturn)
{
	//漫反射L
	//漫反射R
	//前门到位
	//前门面板到位
	//票卡模块到位
	//硬币模块到位
	//硬币回收箱到位
	//纸币模块到位

	//查询状态固定指令
	byte queryBuffer[6] = { 0x55,0x01,0x01,0x10,0x55,0xAA };
	byte replyBuffer[20] = { 0 };

	sp_write((char*)queryBuffer, sizeof(queryBuffer));
	sp_read((char*)replyBuffer, sizeof(replyBuffer));
	int readLen = 0; strlen((char*)replyBuffer);
	int index = 0;
	while (*(replyBuffer + readLen) != DIO_END)
	{
		readLen++;
	}
	readLen++;
	if (replyBuffer[0] == DIO_HEAD && replyBuffer[readLen - 1] == DIO_END)
	{
		byte statusBuffer[6] = { 0 };
		memcpy(statusBuffer, replyBuffer + 3, readLen - 5);
		int statusLen = DIO_Reverse_Escape(statusBuffer, readLen - 5);
		if (replyBuffer[2] == 0xFE)
		{
			//异常响应
			tReturn.iStdErrorCode = iERROR_CODE_DIO_GET_SIGNAL;
			tReturn.iType = 2;
			return 1;
		}
		else
		{
			ucSignalCnt = 8;
			byte stats[8] = { 0 };
			DIO_GetByteBits(statusBuffer[0], stats, sizeof(stats));
			//漫反射L
			pSignalConfig[0].usSignalType = IO_SIGNAL_HUMAN_SENSOR1;
			pSignalConfig[0].ucOperType = stats[7];
			//漫反射R
			pSignalConfig[1].usSignalType = IO_SIGNAL_HUMAN_SENSOR2;
			pSignalConfig[1].ucOperType = stats[6];
			//前门到位
			pSignalConfig[2].usSignalType = IO_SIGNAL_MAINTENANCE_DOOR1;
			pSignalConfig[2].ucOperType = stats[5];
			//前门面板到位
			pSignalConfig[3].usSignalType = IO_SIGNAL_MAINTENANCE_DOOR2;
			pSignalConfig[3].ucOperType = stats[4];
			//票卡模块到位
			pSignalConfig[4].usSignalType = IO_SIGNAL_TH_INPLACE;
			pSignalConfig[4].ucOperType = stats[3];
			//硬币模块到位
			pSignalConfig[5].usSignalType = IO_SIGNAL_CH_INPLACE;
			pSignalConfig[5].ucOperType = stats[0];
			//硬币回收箱到位
			pSignalConfig[6].usSignalType = IO_SIGNAL_COIN_COLECT_BOX_READY;
			pSignalConfig[6].ucOperType = stats[1];
			memset(stats, 0x00, sizeof(stats));
			DIO_GetByteBits(statusBuffer[1], stats, sizeof(stats));
			//纸币模块到位
			pSignalConfig[7].usSignalType = IO_SIGNAL_BH_RECEIVING_INPLACE;
			pSignalConfig[7].ucOperType = stats[1];

			tReturn.iStdErrorCode = iERROR_CODE_DIO_0;
			tReturn.iType = 0;
			return 0;
		}
	}
	else
	{
		//报文不符合格式
		tReturn.iStdErrorCode = iERROR_CODE_DIO_GET_SIGNAL;
		tReturn.iType = 2;
		return 1;
	}

	//if (ucSignalCnt > 4 || ucSignalCnt == 0)
	//{
	//	ucSignalCnt = 4;
	//}
	//for (int i = 0; i < ucSignalCnt; i++)
	//{
	//	memcpy(DIO_m_inBuf, "0", 128);
	//	//	memcpy(DIO_m_inBuf, &DIO_Get_Gate[i], 8);
	//	sp_write(DIO_m_inBuf, 8);
	//	memcpy(DIO_m_outBuf, "0", 128);
	//	sp_read(DIO_m_outBuf, 7);
	//	//if(memcmp(DIO_m_outBuf, &DIO_Gate_Open, 7))
	//	{
	//		pSignalConfig[i].ucOperType = 1;
	//	}
	//	//else if(memcmp(DIO_m_outBuf, &DIO_Gate_Close, 7))
	//	{
	//		pSignalConfig[i].ucOperType = 0;
	//	}
	//	//	else
	//	{
	//		//	LOG_ERROR("<DIO_GetSwitchSignal> receive CMD is %llx", DIO_m_outBuf);
	//		tReturn.iStdErrorCode = iERROR_CODE_DIO_GET_SIGNAL;
	//		tReturn.iType = 2;
	//		return 1;
	//	}
	//	pSignalConfig[i].usSignalType = DIO_OUT_BUZZER + i;
	//}
	//return 0;
}
/**
 * 获取模块固件版本
 *  tVersion - 模块固件版本信息
 *  tReturn - 返回的模块状态信息
 * 返回值:
 *  0 - 执行成功; 其他 - 执行失败;
 */
int DIO_GetVersion(STDioVerInfo & tVersion, STDioDevReturn & tReturn)
{
	//LOG_DEBUG("start.");
	unsigned char hardVer[11];
	memcpy(hardVer, "0", 11);
	memcpy(DIO_m_inBuf, "0", 128);
	memcpy(DIO_m_inBuf, &DIO_Get_ver, 8);
	sp_write(DIO_m_inBuf, 8);

	memcpy(DIO_m_outBuf, "0", 128);
	sp_read(DIO_m_outBuf, 15);
	for (int i = 3; i < 13; i++)
	{
		hardVer[i - 3] = DIO_m_outBuf[i];
	}
	memcpy(tVersion.pDriverVerInfo, DIO_SOFT_VERSION, 21);
	memcpy(tVersion.pHardwareVerInfo, hardVer, 11);
	//LOG_DEBUG("end");
	return 0;
}

