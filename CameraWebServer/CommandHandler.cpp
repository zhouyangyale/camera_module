#include "CommandHandler.h"

int debug = 0;

int M_UPLOAD(const uint8_t command[]){ 

  true_flag = 0;  
  if(debug)Serial.println("M_UPLOAD");
  
  return 0;
}

typedef int (*CommandHandlerType)(const uint8_t command[]);

const CommandHandlerType commandHandlers[] = {
    NULL,
    NULL,
    NULL,
    M_UPLOAD,     //3

};
#define NUM_COMMAND_HANDLERS (sizeof(commandHandlers) / sizeof(commandHandlers[0]))

/****************************Info********************************************** 
 * Name:    InvertUint8 
 * Note: 	把字节颠倒过来，如0x12变成0x48
			0x12: 0001 0010
			0x48: 0100 1000
 *****************************************************************************/
void CommandHandlerClass::InvertUint8(unsigned char *dBuf, unsigned char *srcBuf)
{
    int i;
    unsigned char tmp[4] = {0};

    for (i = 0; i < 8; i++)
    {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (7 - i);
    }
    dBuf[0] = tmp[0];
}
void CommandHandlerClass::InvertUint16(unsigned short *dBuf, unsigned short *srcBuf)
{
    int i;
    unsigned short tmp[4] = {0};

    for (i = 0; i < 16; i++)
    {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (15 - i);
    }
    dBuf[0] = tmp[0];
}
void CommandHandlerClass::InvertUint32(unsigned int *dBuf, unsigned int *srcBuf)
{
    int i;
    unsigned int tmp[4] = {0};

    for (i = 0; i < 32; i++)
    {
        if (srcBuf[0] & (1 << i))
            tmp[0] |= 1 << (31 - i);
    }
    dBuf[0] = tmp[0];
}
#if 0
unsigned short CommandHandlerClass::CRC16_USB(unsigned char *data, unsigned int datalen)
{
	unsigned short wCRCin = 0xFFFF;
	unsigned short wCPoly = 0x8005;
	unsigned char wChar = 0;
	
	while (datalen--) 	
	{
		wChar = *(data++);
		InvertUint8(&wChar,&wChar);
		wCRCin ^= (wChar << 8);
		for(int i = 0;i < 8;i++)
		{
			if(wCRCin & 0x8000)
				wCRCin = (wCRCin << 1) ^ wCPoly;
			else
				wCRCin = wCRCin << 1;
		}
	}
	InvertUint16(&wCRCin,&wCRCin);
	return (wCRCin^0xFFFF);
}
#else
unsigned short CommandHandlerClass::CRC16_USB(unsigned char *data, unsigned int datalen)
{
    unsigned short wCRCin = 0xFFFF;
    unsigned short wCPoly = 0x8005;

    InvertUint16(&wCPoly, &wCPoly);
    while (datalen--)
    {
        wCRCin ^= *(data++);
        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x01)
                wCRCin = (wCRCin >> 1) ^ wCPoly;
            else
                wCRCin = wCRCin >> 1;
        }
    }
    return (wCRCin ^ 0xFFFF);
}
#endif
uint16_t cmd;
#if 1
/*串口1接收完成回调函数*/
//uint8_t RxCpltCallback(protocolComType_t *pUartHandle)
void CommandHandlerClass::RxCpltCallback(protocolComType_t *pUartHandle, uint8_t data)
{
    //if(debug)Serial.printf("\r\npUartHandle->step = %d,data = %d\r\n",pUartHandle->step,data);
    switch (pUartHandle->step)
    {
    case 0:
        //if(debug) Serial.println("case 0");
        if (data == FRAME_HEAD_SAME_81) /*帧头正确*/
        {
            pUartHandle->step++; /*跳转下一步骤*/
            //pUartHandle->aRxBuf[pUartHandle->aRxBufIndex++] = data;
        }
        break;

    case 1:                             //if(debug)Serial.println("case 1");
        if (data == FRAME_HEAD_DIFF_AA) /*帧头正确*/
        {
            pUartHandle->step++; /*跳转下一步骤*/
            //pUartHandle->aRxBuf[pUartHandle->aRxBufIndex++] = data;
        }
        else if (data == FRAME_HEAD_SAME_81)
            pUartHandle->step = 1; /*第一帧头重复，回到第二帧头判断处,AA AA 情况*/
        else
            InitPtcStruct(pUartHandle); /*初始化结构体值,准备下一次接收*/
        break;
    case 2:
        if (data == FRAME_HEAD_SAME_81)
        {
            pUartHandle->step = 1; /*第一帧头重复，回到第二帧头判断处,AA 55 AA 55的情况*/
            //pUartHandle->aRxBufIndex = 1; /*更新第二帧头*/
        }
        else
        {
            //Serial.println(data + 48);
            pUartHandle->tmpCnt = data; /*临时计数值*/
            pUartHandle->step++;        /*跳转下一步骤*/
            //pUartHandle->aRxBuf[pUartHandle->aRxBufIndex++] = data; /*压入缓冲区*/
            //if (((RX_BUF_LENGTH - pUartHandle->aRxBufIndex) < data) || (data == 0))
            //{                               /*缓冲区溢出或数据长度为 0*/
            //    InitPtcStruct(pUartHandle); /*初始化结构体值,准备下一次接收*/
            //}
        }
        break;
    case 3:
        //pUartHandle->tmpCnt = pUartHandle->tmpCnt * 256 + data;                               /*临时计数值*/
        pUartHandle->tmpCnt = (pUartHandle->tmpCnt << 8) | data; /*临时计数值*/
        //Serial.println(pUartHandle->tmpCnt);
        pUartHandle->step++;
        //Serial.println(pUartHandle->aRxBufIndex);                                     /*跳转下一步骤*/
        pUartHandle->aRxBuf[pUartHandle->aRxBufIndex++] = pUartHandle->tmpCnt >> 8; /*压入缓冲区*/
        //Serial.println(pUartHandle->aRxBufIndex);
        pUartHandle->aRxBuf[pUartHandle->aRxBufIndex++] = pUartHandle->tmpCnt & 0x00FF; /*压入缓冲区*/
        //Serial.println(pUartHandle->tmpCnt>>8);
        //Serial.println(pUartHandle->tmpCnt & 0x00FF);
        pUartHandle->tmpCnt = pUartHandle->tmpCnt + 4;
        //Serial.println(pUartHandle->tmpCnt);
        //Serial.println(pUartHandle->aRxBuf[1]);
        //if (((RX_BUF_LENGTH - pUartHandle->aRxBufIndex) < data) || (data == 0))
        // {                               /*缓冲区溢出或数据长度为 0*/
        //InitPtcStruct(pUartHandle); /*初始化结构体值,准备下一次接收*/
        // }
        break;
    case 4:
        //Serial.println(pUartHandle->tmpCnt);

        if ((--pUartHandle->tmpCnt) >= 0)
        { /*接收数据到缓冲区*/
            //Serial.printf("\r\npUartHandle->aRxBuf[%d] = %d\r\n",pUartHandle->aRxBufIndex,data);
            pUartHandle->aRxBuf[pUartHandle->aRxBufIndex++] = data;
            if (pUartHandle->aRxBufIndex >= RX_BUF_LENGTH)
            {                               /*长度被意外修改，导致缓冲区溢出*/
                InitPtcStruct(pUartHandle); /*初始化结构体值,准备下一次接收*/
            }
        }
        else
        {
            //mySerial.write(data + 48);
            /*检查校验和并写入缓冲区*/
#if 0
                if(CheckSumCal(pUartHandle->aRxBuf_1,pUartHandle->aRxBufIndex + 1) == data)
                {
//                PRINTF("uart1\n");
//                for(uint32_t i = 0;i<pUartHandle->aRxBufIndex + 1;i++)
//                PRINTF("%02x\t",pUartHandle->aRxBuf_1[i]);PRINTF("\n");
            
                /*这里可结合上篇文章将数据存入环形缓冲区(也可不存直接处理这里接收的数据，不过数据量太大时可能会丢帧)，并且设置标志位或是发送信号量给任务以处理接收到的数据*/

                //解析数据
                 //mySerial.write(pUartHandle->tmpCnt);
                 //mySerial.write(pUartHandle->tmpCnt);
                 //mySerial.write(pUartHandle->tmpCnt);
                 //receiveCmd(protocolComType_t *pUartHandle)
                 //receiveCmd(pUartHandle->aRxBuf_1,pUartHandle->aRxBufIndex);
                 mySerial.write('2');
                }
#else
            if (pUartHandle->aRxCrcIndex < RX_END_LENGTH)
            {
                pUartHandle->aRxCrc[pUartHandle->aRxCrcIndex++] = data;
                if (debug)
                    Serial.printf("\r\npUartHandle->aRxCrc = %d,data = %d\r\n", pUartHandle->aRxCrcIndex, data);
            }
            //if(DEBUG)
            //Serial.printf("\r\npUartHandle->aRxCrc = %d,data = %d\r\n",pUartHandle->aRxCrcIndex,data);
            //Serial.println(pUartHandle->aRxCrcIndex);
            if (pUartHandle->aRxCrcIndex == RX_END_LENGTH)
            {
                uint16_t crc = CRC16_USB(pUartHandle->aRxBuf, pUartHandle->aRxBufIndex);
                if (debug)
                    Serial.printf("\r\npUartHandle->aRxCrc = %x\r\n", crc);
                if (debug)
                    Serial.printf("\r\nppUartHandle->aRxCrc[0] = %x,XXX%x\r\n", pUartHandle->aRxCrc[0], pUartHandle->aRxCrc[1]);
                if (debug)
                    Serial.printf("\r\nppUartHandle->aRxCrc[2] = %x,XXX%x\r\n", pUartHandle->aRxCrc[2], pUartHandle->aRxCrc[3]);

                if (((crc >> 8) == pUartHandle->aRxCrc[0]) && ((crc & 0xff) == pUartHandle->aRxCrc[1]) && (pUartHandle->aRxCrc[2] == FRAME_TAIL_SAME_55) && (pUartHandle->aRxCrc[3] == FRAME_TAIL_DIFF_81))
                //if ((pUartHandle->aRxCrc[2] == FRAME_TAIL_SAME_55) && (pUartHandle->aRxCrc[3] == FRAME_TAIL_DIFF_81))
                {
                    //验证正确
                    //Serial.println(FRAME_TAIL_SAME_55);
                    //Serial.println(FRAME_TAIL_DIFF_81);
                    //true_flag = 0;
                    // Serial.printf("\r\npUartHandle->aRxBuf[2] = %d,aRxBuf[3] = %d,aRxBuf[4] = %d,aRxBuf[5] = %d,\r\n",pUartHandle->aRxBuf[2],pUartHandle->aRxBuf[3],pUartHandle->aRxBuf[4],pUartHandle->aRxBuf[5]);
                    //Serial.printf("\r\npUartHandle->aRxBuf[6] = %d,aRxBuf[7] = %d,aRxBuf[8] = %d,aRxBuf[9] = %d,\r\n",pUartHandle->aRxBuf[6],pUartHandle->aRxBuf[7],pUartHandle->aRxBuf[4],pUartHandle->aRxBuf[5]);
                    //cmd = pUartHandle->aRxBuf[4] * 256 + pUartHandle->aRxBuf[5];
                    //Serial.println(cmd);
                    //handle(cmd);
                    handle(&pUartHandle->aRxBuf[4]);
                    //Serial.printf("\r\npUartHandle->aRxBuf[6] = %d\r\n",cmd);
                    //mySerial.write(pUartHandle->aRxBuf_1[2]);
                    //receiveCmd(pUartHandle->aRxBuf_1, pUartHandle->aRxBufIndex);
                }
                else
                {
                    //Serial.println("fail ");
                }
                InitPtcStruct(pUartHandle); /*初始化结构体值,准备下一次接收*/
                //uint8_t response[2] = {0x00, 0x00};
                //send_data(response, 2);
                //mySerial.flush();
                // while(mySerial.available()){
                // mySerial.read();
                // }
            }
#endif
        }
        break;

    default:
        InitPtcStruct(pUartHandle); /*初始化结构体值,准备下一次接收*/
        break;
    }
}
#endif
protocolComType_t pUartHandle;

void CommandHandlerClass::InitPtcStruct(protocolComType_t *pUartHandle)
{
    pUartHandle->step = 0;
    pUartHandle->tmpCnt = 0;
    pUartHandle->aRxBufIndex = 0;
    pUartHandle->aRxCrcIndex = 0;
}

void CommandHandlerClass::Serial2Event(void)
{
    uint8_t data;
    int numdata = 0;
    int index = 0;
    pUartHandle.step = 0;
    InitPtcStruct(&pUartHandle);
    uint8_t count = 0;
    if (Serial2.available())
    {
        delay(100);
        int numdata = Serial2.available();
        if (debug)
            Serial.println(numdata);
        for (index = 0; index < numdata; index++)
        {
            data = Serial2.read();
            if (debug)
                Serial.println(data, HEX);
            RxCpltCallback(&pUartHandle, data);
        }
    }
}
//int CommandHandlerClass::handle(const uint16_t command)
int CommandHandlerClass::handle(const uint8_t command[])
{
    if (debug)
        Serial.println(command[0] * 256 + command[1]);
    ///if (command[0] * 256 + command[1] == 3)
    //{
     //   true_flag = 0;
   /// }
    
    //Serial.printf("\r\nhandle over\r\n");
    int responseLength = 0;
    if (command[0] * 256 + command[1] < NUM_COMMAND_HANDLERS){
        CommandHandlerType commandHandlerType = commandHandlers[command[0] * 256 + command[1]];
        if (commandHandlerType)
        {

            responseLength = commandHandlerType(command);
        }
    }

    return responseLength;
}

CommandHandlerClass::CommandHandlerClass()
{
}

void CommandHandlerClass::begin()
{
    Serial2.begin(115200, SERIAL_8N1, CAM_PIN_TX, CAM_PIN_RX);
}

CommandHandlerClass CommandHandler;