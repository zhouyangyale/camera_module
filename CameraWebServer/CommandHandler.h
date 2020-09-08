#ifndef __COMMAND_HANDLER_H__
#define __COMMAND_HANDLER_H__
#include "Arduino.h"
#include "esp_camera.h"
#define CAM_PIN_TX 4
#define CAM_PIN_RX 13

//81AA00000001000304045581
/*帧头长度*/
#define FRAME_HEADER_LENGTH 2U
/*帧尾长度(即校验和)*/
#define FRAME_TAIL_LENGTH 2U
/*结尾长度*/
#define FRAME_END_LENGTH 2U

/*帧头相同字节(第一字节)*/
#define FRAME_HEAD_SAME_81 0x81
/*帧头相同字节(第二字节)*/
#define FRAME_HEAD_DIFF_AA 0xAA

/*帧头相同字节(第一字节)*/
#define FRAME_TAIL_SAME_55 0x55
/*帧头相同字节(第二字节)*/
#define FRAME_TAIL_DIFF_81 0x81

/*接收缓冲区长度*/
#define RX_BUF_LENGTH 64U
#define RX_END_LENGTH 4U
extern int true_flag;
typedef struct
{
  volatile uint8_t step;        /*switch语句跳转条件*/
  volatile int16_t tmpCnt;      /*用于计数的临时变量*/
  volatile uint8_t aRxBufIndex; /*接收数据缓冲区索引*/
  volatile uint8_t aRxCrcIndex; /*结尾数据缓冲区索引*/
  uint8_t aRxCrc[RX_END_LENGTH];
  uint8_t aRxBuf[RX_BUF_LENGTH];
} protocolComType_t;

class CommandHandlerClass
{

public:
public:
  CommandHandlerClass();
  void begin();
  //int  handle(const uint8_t command[], uint8_t response[]);
  int handle(const uint8_t command[]);
  void Serial2Event(void);

private:
private:
  void InitPtcStruct(protocolComType_t *pUartHandle);
  void RxCpltCallback(protocolComType_t *pUartHandle, uint8_t data);
  int handle(const uint16_t command);
  void InvertUint8(unsigned char *dBuf, unsigned char *srcBuf);
  void InvertUint16(unsigned short *dBuf, unsigned short *srcBuf);
  void InvertUint32(unsigned int *dBuf, unsigned int *srcBuf);
  unsigned short CRC16_USB(unsigned char *data, unsigned int datalen);
};
extern CommandHandlerClass CommandHandler;
#endif