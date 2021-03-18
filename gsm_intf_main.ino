#include "driver/uart.h"
#include "coord_intf.h"
#include "dis.h"
#include "gw.h"
#include "pltfrm.h"
#include "wsuart.h"
#include "node_attr.h"

#define TINY_GSM_MODEM_SIM868

/*
   There are three serial ports on the ESP32 known as U0UXD, U1UXD and U2UXD all
   work at 3.3V TTL Level. There are three hardware supported serial interfaces
   on the ESP32 known as UART0, UART1 and UART2. Like all peripherals, the pins
   for the UARTs can be logically mapped to any of the available pins on the ESP32.
   However, the UARTs can also have direct access which marginally improves
   performance. The pin mapping table for this hardware assistance is as follows.

    ----------------------------------------
    UART    RX Pin   TX Pin   CTS     RTS
    ----------------------------------------
    UART0   GPIO3    GPIO1    N/A     N/A
    UART1   GPIO9    GPIO10   GPIO6   GPIO11
    UART2   GPIO16   GPIO17   GPIO8   GPIO7
    ----------------------------------------
*/

#define SerialMon Serial
#define GSM_serialObj Serial2
#define GSM_RX_PIN  16
#define GSM_TX_PIN  17

#define COORD_INTF_SERIAL_PORT_NR  UART_NUM_1
#define COORD_INTF_RX_PIN  18
#define COORD_INTF_TX_PIN  19

#define TINY_GSM_DEBUG  SerialMon
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 57600

#define LED_PIN 13
#define EVENT_FMT_TYPE_MERGED_DATA_JSON

const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* get_attr = "get_attr";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

TinyGsm modem(GSM_serialObj);

/*
   Create one or more TinyGSM client instances
   For a single connection, use "TinyGsmClient client(modem);".
*/
TinyGsmClient TinyGSM_client_1(modem, 0);
TinyGsmClient TinyGSM_client_2(modem, 1);
TinyGsmClient TinyGSM_client_coord(modem, 2);
TinyGsmClient TinyGSM_client_coordResp(modem, 3);
TinyGsmClient tinyGsmCmnClient(modem, 4);

typedef struct
{
  uint32_t pubCnt;
  uint32_t pubFlrCnt;
  uint32_t pubSuccessCnt;
  uint32_t pubFlrConnUpCnt;
  uint32_t pubFlrConnDownCnt;
} MQTT_stats_t;

MQTT_stats_t MQTT_stats = {0, 0, 0, 0, 0};

uint32_t GW_txToCloudPendCnt = 10;

int ledStatus = LOW;
int hdrAcked = 0, 
    hdrAckSts = 0;
    
uint32_t lastReconnectAttempt = 0;

float __latestVcc;
int __latestVccSet = 0;

typedef struct
{
  bool gsmCntSts;
  bool gsmNetwrkCntSts;
  bool gsmGprsCntSts;
  bool gsmCloudCntSts;
}GSM_status_t;

GSM_status_t GSM_status = {false, false, false, false};

#define QUEUE_Q2_ENTRY_CNT  8
#define QUEUE_Q3_ENTRY_CNT  8
#define QUEUE_Q4_ENTRY_CNT  8
#define QUEUE_Q5_ENTRY_CNT  1
#define QUEUE_Q6_ENTRY_CNT  1

QueueHandle_t CoordIntf_uartRxQEvtHndl;
QueueHandle_t Q2Hndl;
QueueHandle_t Q3Hndl;
QueueHandle_t Q4Hndl;
QueueHandle_t Q5Hndl;
QueueHandle_t Q6Hndl;

int node_rssi,
    node_lqi,
    node_process_sts = 0,
    GW_DataBufLen = 0,
    GW_relaySnsrDataToCloud = 1;
char Node_macIdBuff[64],
     GW_devName[1024],
     GW_macBuff[1024];
uint32_t seq_nr = 0;
int merge_store[10], merge_count = 0;
int snsrIdStore[10];
char merge_event[100];
char GW_snsrDataBuff[512] = {'\0'};

char snsrIdStore0[16],
     snsrIdStore1[16],
     snsrIdStore2[16],
     snsrIdStore3[16];
     
char GW_snsrDataBuff_0[150] = {'\0'},
     GW_snsrDataBuff_1[150] = {'\0'},
     GW_snsrDataBuff_2[150] = {'\0'},
     GW_snsrDataBuff_3[150] = {'\0'};
     
char GW_get_time[150] = {'\0'};
float snsrDataBuff[10] = {0};

char GW_jsonMsg[1024];

#define MAX_CLOUD_ATTR_FIELD_LEN 100

char attrArgv1[MAX_CLOUD_ATTR_FIELD_LEN],
     attrArgv2[MAX_CLOUD_ATTR_FIELD_LEN],
     attrArgv3[MAX_CLOUD_ATTR_FIELD_LEN],
     attrArgv4[MAX_CLOUD_ATTR_FIELD_LEN],
     attrArgv5[MAX_CLOUD_ATTR_FIELD_LEN];

char attrName[MAX_CLOUD_ATTR_FIELD_LEN];

byte rcvdMqttMsg[512];
char rcvdCoordAttrCpy[50],
     rcvdMqttMsgCpy[512],
     rcvdNodeName[13];
     
char rcvdNodePyld[1024];
char rcvdCoordResp[256],
     GW_coordMsg[256];     

int mqttMsgPyldLen = 0;
     
#if 0
      StaticJsonBuffer<1024> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
#else
      DynamicJsonDocument jsonDoc(1024);
      DynamicJsonDocument jsonDoc_2(1024);
      DynamicJsonDocument jsonDoc_coordResp(1024);
      JsonObject root = jsonDoc.to<JsonObject>();
      JsonObject root_coordResp = jsonDoc_coordResp.to<JsonObject>();
#endif

#define SER_BUFF_LEN 256
unsigned char serTxBuff[SER_BUFF_LEN];
unsigned char serRxBuff[SER_BUFF_LEN];

boolean MODEM_restarted = false;

#define COORD_INTF_UART_DRIVER_BUFF_SIZE (1024 * 2)
#define COORD_INTF_EVT_QUEUE_SIZE  2

#define COORD_INTF_MSG_BUFF_LEN  256
#define COORD_INTF_RAW_BUFF_LEN  256

uint8_t COORD_INTF_taskRxRawBuff[COORD_INTF_RAW_BUFF_LEN];
uint8_t COORD_INTF_taskRxMsgBuff[COORD_INTF_MSG_BUFF_LEN];

struct CoordAttributs
{
  int cnt_que = 0;
  unsigned int CoordAttrMsg[256];
} COORD_ATTR_SEND_MSG, COORD_ATTR_RCVD_MSG;

uart_config_t CoordIntf_uartConfigInfo =
{
  .baud_rate = 38400,
  .data_bits = UART_DATA_8_BITS,
  .parity = UART_PARITY_DISABLE,
  .stop_bits = UART_STOP_BITS_1,
  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

int COORD_INTF_rxMsgOff = 0,
    COORD_INTF_rcvdFramePyldLen = 0;

uint32_t COORD_IF_nodeMsgCnt = 0;

unsigned int GW_coordPendingAttrId = 0;

char CoordAttrRcvdData[100];

boolean COORD_INTF_checkForHdr(uint8_t *hdr_p)
{
  boolean rc = false;
  const uint16_t calcCkSum = COORD_INTF_calcCkSum16(hdr_p,
                             COORD_INTF_FRAME_HDR_LEN - COORD_INTF_FRAME_HDR_CRC_FIELD_LEN * 2);
  const uint16_t rcvdCkSum = UTIL_ntohs(hdr_p + COORD_INTF_FRAME_HDR_HDR_CRC_FIELD_OFF);
  if (calcCkSum == rcvdCkSum)
  {
    COORD_INTF_rcvdFramePyldLen = UTIL_ntohs(hdr_p + COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_OFF);
    if (COORD_INTF_rcvdFramePyldLen <= 255)
      rc = true;
    else
      Serial.printf("Bad P-L %d !! \n", COORD_INTF_rcvdFramePyldLen);
  }
  else
  {
    Serial.printf("Hdr CKSM MM R:%d C:%d !! \n", rcvdCkSum, calcCkSum);
  }
  return rc;
}


void GW_appendToEventBuffer(unsigned char *extAddr_p,
                            int snsrId,
                            char *unit_p,
                            float valF)
{
  int strLen = strlen(GW_snsrDataBuff);
  if (merge_count == 0)
  {
    snsrIdStore[merge_count] = snsrId;
    snsrDataBuff[merge_count] = valF;
  }
  if (merge_count == 1)
  {
    snsrIdStore[merge_count] = snsrId;
    snsrDataBuff[merge_count] = valF;
  }
  if (merge_count == 2)
  {
    snsrIdStore[merge_count] = snsrId;
    snsrDataBuff[merge_count] = valF;
  }
  if (merge_count == 3)
  {
    snsrIdStore[merge_count] = snsrId;
    snsrDataBuff[merge_count] = valF;
  }
  if (merge_count == 4)
  {
    snsrIdStore[merge_count] = snsrId;
    snsrDataBuff[merge_count] = valF;
  }
  if (strLen == 0)
  {
    strLen = sprintf(GW_snsrDataBuff, "ID_%02x%02x%02x%02x",
                     extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
    merge_store[merge_count] = strLen;
    snsrIdStore[merge_count] = snsrId;
  }
  
  merge_count++;
  sprintf(GW_devName, "WSN-%02x%02x%02x%02x", extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
  sprintf(GW_macBuff, "%02x:%02x:%02x:%02x", extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
  Serial.printf("\nGW_devName <%s> GW_macBuff <%s>\n", GW_devName, GW_macBuff);
  
  GW_DataBufLen = GW_DataBufLen + strLen;
  sprintf(GW_snsrDataBuff + strLen,  "__S%d_V%.3f_U%s", snsrId, valF, unit_p);
}


void GW_appendToJsonBuff()
{
  int jsonMsgLen;
  char* jsonMsgPtr_p;
  Serial.printf("\nPacket Received... \n");
  seq_nr++;
  
  memset(GW_snsrDataBuff, 0, sizeof(GW_snsrDataBuff));
  memset(GW_snsrDataBuff_0, 0, sizeof(GW_snsrDataBuff_0));
  memset(GW_snsrDataBuff_1, 0, sizeof(GW_snsrDataBuff_1));
  memset(GW_snsrDataBuff_2, 0, sizeof(GW_snsrDataBuff_2));
  memset(GW_snsrDataBuff_3, 0, sizeof(GW_snsrDataBuff_3));

  sprintf(GW_snsrDataBuff_0, "%.2f", snsrDataBuff[0]);
  sprintf(GW_snsrDataBuff_1, "%.2f", snsrDataBuff[1]);
  sprintf(GW_snsrDataBuff_2, "%.2f", snsrDataBuff[2]);
  sprintf(GW_snsrDataBuff_3, "%.2f", snsrDataBuff[3]);

  sprintf(snsrIdStore3, "0x%x", snsrIdStore[3]);
  sprintf(snsrIdStore2, "0x%x", snsrIdStore[2]);
  sprintf(snsrIdStore1, "0x%x", snsrIdStore[1]);
  sprintf(snsrIdStore0, "0x%x", snsrIdStore[0]);
  
  root["Name"] = GW_devName;
  root["MacId"] = GW_macBuff;
  root["RSSI"] = node_rssi;
  root["LQI"] = node_lqi;
  root[snsrIdStore0] = GW_snsrDataBuff_0;
  root[snsrIdStore1] = GW_snsrDataBuff_1;
  if (snsrIdStore[2] != 0x0)
  {
    root[snsrIdStore2] = GW_snsrDataBuff_2;
  }
  if (snsrIdStore[3] != 0x0)
  {
    root[snsrIdStore3] = GW_snsrDataBuff_3;
  }
  root["Sequence_number"] = seq_nr;
  
#if 0
  root.printTo(GW_jsonMsg);
#else
  serializeJson(jsonDoc, GW_jsonMsg);
#endif
  Serial.printf("\nComplete JSON MSG: ");
  Serial.printf(GW_jsonMsg);
  Serial.printf("\n");

  jsonMsgLen = strlen(GW_jsonMsg);
  jsonMsgPtr_p = (char*)malloc(sizeof(GW_jsonMsg));
  if (jsonMsgPtr_p == NULL)
    return;
  memcpy(jsonMsgPtr_p, GW_jsonMsg, jsonMsgLen + 1);
  Serial.printf("\nThe jsonMsgLen <%d>\n", jsonMsgLen);
  Serial.printf("\nThe jsonMsgPtr_p:");
  Serial.printf(jsonMsgPtr_p);
  Serial.printf("\n");
  if (xQueueSend(Q3Hndl, (void*)jsonMsgPtr_p, (TickType_t)5) == pdPASS)
  {
    Serial.printf("\nData Enqueued in Q3- success\n");
  }
  else
  {
    Serial.printf("\n No Queue space in Q3Hndl...\n");
  }

  free(jsonMsgPtr_p);
  jsonMsgPtr_p = NULL;

  for (int i = 0; i < 4; i++)
  {
    snsrDataBuff[i] = '\0';
    snsrIdStore[i] = '\0';
  }
  merge_count = 0;
}


void COORD_IF_procNodeMsg(uint8_t *buff_p, int msgPyldLen)
{
  int srcShortAddr, off = 0;
  unsigned int disMsgType;
  uint8_t *extAddr_p;

  COORD_IF_nodeMsgCnt ++;

  if (msgPyldLen < DIS_LPWMN_MAC_SHORT_ADDR_LEN
      + DIS_LPWMN_MAC_EXT_ADDR_LEN
      + DIS_LPWMN_MSG_RSSI_LEN
      + DIS_LPWMN_MSG_CORR_LQI_LEN)
    return;

  srcShortAddr = buff_p[off];
  srcShortAddr = (srcShortAddr << 8) | buff_p[off + 1];

  off += DIS_LPWMN_MAC_SHORT_ADDR_LEN;

#if 1
  Serial.printf("C_I_pNM> [%u] Received msg from node <%05u / %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x>\n",
                COORD_IF_nodeMsgCnt,
                srcShortAddr,
                (unsigned int)buff_p[off],
                (unsigned int)buff_p[off + 1],
                (unsigned int)buff_p[off + 2],
                (unsigned int)buff_p[off + 3],
                (unsigned int)buff_p[off + 4],
                (unsigned int)buff_p[off + 5],
                (unsigned int)buff_p[off + 6],
                (unsigned int)buff_p[off + 7]);
  sprintf(Node_macIdBuff, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
          buff_p[off], buff_p[off + 1], buff_p[off + 2], buff_p[off + 3], 
          buff_p[off + 4], buff_p[off + 5], buff_p[off + 6], buff_p[off + 7]);
#endif

  extAddr_p = buff_p + off;
  buff_p += DIS_LPWMN_MAC_EXT_ADDR_LEN;

  {
     int rssi;
     unsigned int lqi_corr;

     rssi = (signed char)buff_p[off];
     lqi_corr = buff_p[off + 1];
     Serial.printf("C_I_pNM> RSSI %d dBm / LQI %u\n", (int)rssi, lqi_corr);
     node_rssi = rssi;
     node_lqi = lqi_corr;
  }
  
  off += (LPWMN_MSG_RSSI_LEN + LPWMN_MSG_CORR_LQI_LEN);

  msgPyldLen -= (DIS_LPWMN_MAC_SHORT_ADDR_LEN
                 + DIS_LPWMN_MAC_EXT_ADDR_LEN
                 + DIS_LPWMN_MSG_RSSI_LEN
                 + DIS_LPWMN_MSG_CORR_LQI_LEN);

  if (msgPyldLen < 1)
      return;

  disMsgType = buff_p[off];


  off += DIS_MSG_TYPE_SZ;
  msgPyldLen -= DIS_MSG_TYPE_SZ;
  buff_p += off;
    
  if (disMsgType == expDisMsgType && srcShortAddr == expDisMsgSrcShortAddr)
  {
      Serial.printf("<%s> dis-msg-type <0x%x> \n", __FUNCTION__, disMsgType);
      switch(disMsgType)
      {
          case DIS_MSG_TYPE_GET_ATTR_VAL:
              {
                  Serial.printf("Msg Type - DIS_MSG_TYPE_GET_ATTR_VAL !! \n");
                  Serial.printf("<%s> rcvd dis msg type GET_NODE_ATTR_VAL \n", __FUNCTION__);
                  if (msgPyldLen >= DIS_TLV_HDR_SZ)
                  {
                      unsigned char tlvLen1, *buff1_p;
                      Serial.printf("\n****************************************************************\n");
                      Serial.printf("buff: ");
                      for(int idx = 0; idx < msgPyldLen; idx++ )
                      {
                          Serial.printf("%x ", buff_p[idx]);   
                      }
                      Serial.printf("\n*****************************************************************\n");
                                                               
                      int rc = TLV_get(buff_p, msgPyldLen, DIS_TLV_TYPE_ATTR_INFO, &tlvLen1, &buff1_p);
                      if (rc == 0)
                      {
                          Serial.printf("\n Could not find DIS_TLV_TYPE_ATTR_INFO  !!\n");
                      }
                      else
                      {
                          Serial.printf("Found DIS_TLV_TYPE_ATTR_INFO - pyldLen<%d>/tlvLen1<%d> \n", msgPyldLen, tlvLen1);
  
                          msgPyldLen -= DIS_TLV_HDR_SZ;
                          if (msgPyldLen >= tlvLen1)
                          {
                              unsigned char tlvLen2, *buff2_p;
                              int rc = TLV_get(buff1_p, msgPyldLen, DIS_TLV_TYPE_ATTR_ID, &tlvLen2, &buff2_p);
                              if (rc == 0)
                              {
                                   Serial.printf("\n Could not find DIS_TLV_TYPE_ATTR_ID  !!\n");
                              }
                              else
                              {
                                   Serial.printf("\n Found DIS_TLV_TYPE_ATTR_ID ... \n");
    
                                  if (tlvLen2 == DIS_ATTR_ID_FIELD_SZ)
                                  {
                                      unsigned int attrId = UTIL_ntohs(buff2_p);
  
                                      Serial.printf("AttrId<%u>  ", attrId);
  
                                      int rc = TLV_get(buff1_p, msgPyldLen, DIS_TLV_TYPE_ATTR_VAL, &tlvLen2, &buff2_p);
                                      if (rc == 0)
                                      {
                                          Serial.printf("\n Could not find DIS_TLV_TYPE_ATTR_ID  !!\n");
                                      }
                                      else
                                      {
                                          Serial.printf("\n Found DIS_TLV_TYPE_ATTR_VAL ... %u \n", tlvLen2);
  
                                          switch (tlvLen2)
                                          {
                                              memset(CoordAttrRcvdData, 0, sizeof(CoordAttrRcvdData));
                                              case 1:
                                                if (attrId == CC1120_AGC_CS_THR_ATTR_ID)
                                                {
                                                    Serial.printf("AttrVal<%d> \n", (char)(*buff2_p));
                                                    sprintf(CoordAttrRcvdData, "%d", (char)(*buff2_p));
                                                    if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                                    {
                                                         Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                                    }
                                                    else
                                                    {
                                                         Serial.printf("\nNo queue space in Q6Hndl...\n");
                                                    }
                                                }
                                                else
                                                {
                                                    Serial.printf("AttrVal<%u / 0x%x> \n", *buff2_p, *buff2_p);
                                                    sprintf(CoordAttrRcvdData, "%u", *buff2_p);
                                                    if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                                    {
                                                         Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                                    }
                                                    else
                                                    {
                                                         Serial.printf("\nNo queue space in Q6Hndl...\n");
                                                    }
                                                }
                                                break;
    
                                              case 2:
                                                if (attrId == APP_MAG3110_X_AXIS_BASE_VAL_MFS_ATTR_ID
                                                    || attrId == APP_MAG3110_Y_AXIS_BASE_VAL_MFS_ATTR_ID
                                                    || attrId == APP_MAG3110_Z_AXIS_BASE_VAL_MFS_ATTR_ID
                                                    || attrId == RADIO_FREQ_OFFSET_ATTR_ID
                                                    || attrId == PHY_TX_POWER_ATTR_ID
                                                    || attrId == APP_LOAD_CELL_ZERO_BALANCE_FIGURE_ATTR_ID)
                                                {
                                                    short attrVal = UTIL_ntohs(buff2_p);
                                                    Serial.printf("AttrVal<%d> \n", attrVal);
                                                    sprintf(CoordAttrRcvdData, "%d", attrVal);
                                                    if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                                    {
                                                         Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                                    }
                                                    else
                                                    {
                                                         Serial.printf("\nNo queue space in Q6Hndl...\n");
                                                    }                                                  
                                                }
                                                else
                                                {
                                                    Serial.printf("AttrVal<%u / 0x%x> \n", UTIL_ntohs(buff2_p), UTIL_ntohs(buff2_p));
                                                    sprintf(CoordAttrRcvdData, "%u", UTIL_ntohs(buff2_p));
                                                    if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                                    {
                                                         Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                                    }
                                                    else
                                                    {
                                                         Serial.printf("\nNo queue space in Q6Hndl...\n");
                                                    }
                                                }
                                                break;
    
                                           case 4:
                                            {
                                               switch (attrId)
                                               {
                                                  case FU_IMAGE_STORE_IMAGE_FLAGS_ATTR_ID:
                                                  case FU_IMAGE_STORE_IMAGE_1_IMAGE_RCVD_TIME_STAMP_ATTR_ID:
                                                  case FU_IMAGE_STORE_IMAGE_2_IMAGE_RCVD_TIME_STAMP_ATTR_ID:
                                                       GW_procImageAttrVal(attrId, buff2_p);
                                                       break;
  
                                                  default:
                                                       Serial.printf("AttrVal<%u / 0x%x> \n", UTIL_ntohl(buff2_p), UTIL_ntohl(buff2_p));
                                                       sprintf(CoordAttrRcvdData, "%u", UTIL_ntohl(buff2_p));
                                                       if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                                       {
                                                          Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                                       } 
                                                       else
                                                       { 
                                                           Serial.printf("\nNo queue space in Q6Hndl...\n");
                                                       }
                                                       break;
                                               }
                                            }
                                            break;
  
                                            default:
                                                if (attrId == FW_BUILD_DATE_ATTR_ID || attrId  == FW_BUILD_TIME_ATTR_ID)
                                                {
                                                    Serial.printf("%s%s%s \n",
                                                           attrId == FW_BUILD_DATE_ATTR_ID ? "build date <" : "build time <" , buff2_p, ">");
                                                }
                                                else
                                                {
                                                    if (attrId == APP_LOAD_CELL_SERIAL_NR_ATTR_ID)
                                                    {
                                                        char loadCellSrNr[8 + 1];
                                                        memcpy(loadCellSrNr, buff2_p, 8);
                                                        loadCellSrNr[8] = '\0';
                                                        printf("%s \n", loadCellSrNr);
                                                    }
                                                    else
                                                    {
                                                        Serial.printf("AttrVal<%d byte response> \n", tlvLen2);
                                                        sprintf(CoordAttrRcvdData, "%d", tlvLen2);
                                                        if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                                        {
                                                             Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                                        }
                                                        else
                                                        {
                                                             Serial.printf("\nNo queue space in Q6Hndl...\n");
                                                        }
                                                    }  
                                                    break;
                                                }
                                          }
                                      }
                                   }
                                   else
                                   {
                                       Serial.printf("\n Bad DIS_TLV_TYPE_ATTR_ID TLV !!\n");
                                   }
                                }
                            }
                      }
                  }
                  else
                  {
                      Serial.printf("\n Could not find DIS_TLV_TYPE_ATTR_INFO  !!\n");
                  }
             }
             break;
      }    
  }
  if (msgPyldLen <= 0xff)
  {
      uint8_t rc, tlvLen1, *buff1_p;
      rc = TLV_get(buff_p, msgPyldLen,
                   DIS_TLV_TYPE_SENSOR_OUTPUT_LIST,
                   &tlvLen1, &buff1_p);
      if (rc == 0)
      {
          Serial.printf("C_I_pNM> No SENSOR_OUTPUT_LIST TLV !!\n");
          return;
      }
      else
      {
          Serial.printf("C_I_pNM> Found SENSOR_OUTPUT_LIST TL\n");

          while (1)
          {
             uint8_t tlvLen2, *buff2_p;

             rc = TLV_get(buff1_p, tlvLen1,
                          DIS_TLV_TYPE_SENSOR_OUTPUT,
                          &tlvLen2, &buff2_p);

                          
             if (rc == 0)
             {
                 Serial.printf("C_I_pNM> no more SENSOR_OUTPUT TLV\n");
                 merge_count = 0;
                 break;
             }
             else
             {
                 uint8_t tlvLen3, *buff3_p;
                 int snsrId, scaleFactor = DIS_DATA_SCALE_CENTI;
      
                 buff1_p += (tlvLen2 + DIS_TLV_HDR_SZ);
                 if (tlvLen1 >= (tlvLen2 + DIS_TLV_HDR_SZ))
                     tlvLen1 -= (tlvLen2 + DIS_TLV_HDR_SZ);
                 else
                 {
                     Serial.printf("C_I_pNM> Malformed TLV fnd <%d/%d>\n !!\n",
                                   tlvLen1, tlvLen2 + DIS_TLV_HDR_SZ);
                     break;
                 }

                 Serial.printf("C_I_pNM> Found _SENSOR_OUTPUT TLV / vfl<%d>\n", tlvLen2);

                 rc = TLV_get(buff2_p, tlvLen2,
                              DIS_TLV_TYPE_SENSOR_ID,
                              &tlvLen3, &buff3_p);
                 if (rc == 0)
                     continue;
                 else
                 {
                     if (tlvLen3 == DIS_SENSOR_ID_FIELD_SZ)
                     {
                         snsrId = *buff3_p;
                         Serial.printf("C_I_pNM> Snsr Id <0x%x>\n", snsrId);
                     }
                     else
                         continue;
                 }

                 rc = TLV_get(buff2_p, tlvLen2, DIS_TLV_TYPE_DATA_SCALE_FACTOR, &tlvLen3, &buff3_p);
                 if (rc)
                 {
                     if (tlvLen3 == DIS_DATA_SCALE_FACTOR_FIELD_SZ)
                     {
                         scaleFactor = *buff3_p;
                         if (!(scaleFactor >= DIS_DATA_SCALE_TERA && scaleFactor <= DIS_DATA_SCALE_FEMTO))
                             scaleFactor = DIS_DATA_SCALE_NONE;
                     }
                 } 

                 rc = TLV_get(buff2_p, tlvLen2, DIS_TLV_TYPE_VALUE, &tlvLen3, &buff3_p);
                 if (rc == 0)
                     continue;
                 else
                 {
                     int snsrOp;
                     signed short snsrOp16;
                     char *unit_p = " ";

                     Serial.printf("C_I_pNM> Found VALUE TLV .. v-f-l<%d>\n", tlvLen3);

                     switch (tlvLen3)
                     {
                         case 1:
                              {
                                 snsrOp = (int)(*buff3_p);
                              }
                              break;
              
                         case 2:
                              {
                                 snsrOp16 = UTIL_ntohs(buff3_p);
                                 snsrOp = snsrOp16;
                              }
                              break;
              
                         case 3:
                              {
                                 uint8_t lBuff[4];
                                 lBuff[0] = 0;
                                 memcpy(lBuff + 1, buff3_p, 3);
                                 snsrOp = (int)UTIL_ntohl(lBuff);
                                 printf("C_I_pNM>  <1b> snsrOp : %d\n", snsrOp);
                              }
                              break;
           
                        case 4:
                              {
                                 snsrOp = (int)UTIL_ntohl(buff3_p);
                              }
                              break;
                              
                        default:
                              break;
                     }
                      
                     switch (snsrId)
                     {
                         case PLTFRM_DUMMY_DEV_ID:
                              unit_p = "";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;

                         case PLTFRM_DS18B20_1_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_100MICRO;
                              break;

                         case PLTFRM_DS18B20_2_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_100MICRO;
                              break;

                         case PLTFRM_DS18B20_3_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_100MICRO;
                              break;

                         case PLTFRM_DS18B20_4_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_100MICRO;
                              break;

                         case PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID:
                              unit_p = "Volts";
                              break;

                         case PLTFRM_LM75B_1_DEV_ID:
                              unit_p = "Deg C";
                              break;

                         case PLTFRM_TEPT5700_1_DEV_ID:
                              unit_p = "uA";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;
                               
                         case PLTFRM_DUAL_FS_LVL_MON_1_DEV_ID:
                              unit_p = "";
                              break;

                         case PLTFRM_ON_CHIP_TEMP_SENSOR_DEV_ID:
                              unit_p = "deg C";
                              break;

                         case PLTFRM_BATT_1_DEV_ID:
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "Volts";
                              break;

                         case PLTFRM_TMP75C_1_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_HIH8121_1_RH_DEV_ID:
                              unit_p = "%";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_HIH8121_1_TEMP_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_RTD_1000_1_DEV_ID:
                              unit_p = "deg C";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;
                               
                         default:
                              Serial.printf("[Unknown]");
                              break;
                     }
 
                     if (snsrId == PLTFRM_GEN_VOLT_MON_DEV_ID
                         || snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID)
                     {
                         scaleFactor = DIS_DATA_SCALE_MILLI;
                     }
                      
                     if (snsrId == PLTFRM_GEN_CURRENT_MON_DEV_ID)
                         scaleFactor = DIS_DATA_SCALE_NONE;

                     if (snsrId == PLTFRM_LM75B_1_DEV_ID
                         || snsrId == PLTFRM_BATT_1_DEV_ID
                         || snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID
                         || snsrId == PLTFRM_ON_CHIP_TEMP_SENSOR_DEV_ID
                         || snsrId == PLTFRM_EXT_VOLTAGE_MON_DEV_ID
                         || snsrId == PLTFRM_TMP75C_1_DEV_ID
                         || snsrId == PLTFRM_HIH8121_1_RH_DEV_ID
                         || snsrId == PLTFRM_HIH8121_1_TEMP_DEV_ID
                         || snsrId == PLTFRM_DS18B20_1_DEV_ID
                         || snsrId == PLTFRM_DS18B20_2_DEV_ID
                         || snsrId == PLTFRM_DS18B20_3_DEV_ID
                         || snsrId == PLTFRM_DS18B20_4_DEV_ID
                         || snsrId == PLTFRM_RTD_1000_1_DEV_ID)
                     {
                         float valF = snsrOp;

                         switch (scaleFactor)
                         {
                            case DIS_DATA_SCALE_MICRO:
                                 valF /= 1000;
                                 valF /= 1000;
                                 break;

                            case DIS_DATA_SCALE_MILLI:
                                 valF /= 1000;
                                 break;

                            case DIS_DATA_SCALE_CENTI:
                                 valF /= 100;
                                 break;

                            case DIS_DATA_SCALE_DECI:
                                 valF /= 10;
                                 break;

                            case DIS_DATA_SCALE_100MICRO:
                                 valF /= 10000;
                                 break;

                            default:
                                 break;
                         }
                      
                         if (snsrId != PLTFRM_AD7797_1_DEV_ID
                             && snsrId != PLTFRM_MP3V5050GP_1_DEV_ID
                             && snsrId != PLTFRM_MP3V5004GP_1_DEV_ID)
                         {
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                             GW_appendToEventBuffer(extAddr_p, snsrId, unit_p, valF);
#endif
                         }
                      
                         if (snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID)
                         {
                             __latestVcc = valF;
                             __latestVccSet = 1;
                         }
                     }
                 }
             }
         }
     }
  }
  
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
  GW_appendToJsonBuff();
  merge_count = 0;
#endif
  
  return;
}


uint8_t COORD_IF_q2MsgTempBuff[COORD_INTF_MSG_BUFF_LEN];

void COORD_INTF_coordMsgProcTask(void *params_p)
{
   static uint32_t C_I_msgOnQ2RxCnt = 0;
   
   while (1)
   {
      if (xQueueReceive(Q2Hndl, (void *)COORD_IF_q2MsgTempBuff, (portTickType)portMAX_DELAY))
      {
          int coordMsgTotLen = UTIL_ntohs(COORD_IF_q2MsgTempBuff + COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_OFF);
          coordMsgTotLen += COORD_INTF_FRAME_HDR_LEN;
          C_I_msgOnQ2RxCnt ++;
          Serial.printf("C_I_cMPT> #[%u] rcvd msg len[%d] on Q2\n", 
                        C_I_msgOnQ2RxCnt, coordMsgTotLen);              
          COORD_IF_procNodeMsg(COORD_IF_q2MsgTempBuff + COORD_INTF_FRAME_HDR_LEN, 
                               coordMsgTotLen - COORD_INTF_FRAME_HDR_LEN);
      }
   }
}


void COORD_INTF_procRcvdFrame(void)
{
   static uint32_t C_I_qToQ2DoneCnt = 0, C_I_qToQ2FlrCnt = 0;
   uint16_t msgType = UTIL_ntohs(COORD_INTF_taskRxMsgBuff);

   Serial.printf("C_I_pRF> T:%d PL:%d\n", (int)msgType, COORD_INTF_rcvdFramePyldLen);
   memset(CoordAttrRcvdData, 0, sizeof(CoordAttrRcvdData));
 
   switch (msgType)
   {
      case LPWMN_GW_MSG_TYPE_RELAY_FROM_NODE:
           {
           
               /*
                * BaseType_t xQueueSend(
                *                        QueueHandle_t  xQueue,
                *                        const void *  pvItemToQueue,
                *                        TickType_t  xTicksToWait
                *                      );
                * Post an item on a queue. The item is queued by copy, not by reference. 
                * 
                * pvItemToQueue: A pointer to the item that is to be placed on the queue. The size of the items the 
                *                queue will hold was defined when the queue was created, so this many bytes will be 
                *                copied from pvItemToQueue into the queue storage area.
                * xTicksToWait: The maximum amount of time the task should block waiting for space to become available 
                *               on the queue, should it already be full. The call will return immediately if the queue 
                *               is full and xTicksToWait is set to 0. The time is defined in tick periods so the constant 
                *               portTICK_PERIOD_MS should be used to convert to real time if this is required.
                */
               if (xQueueSend(Q2Hndl, COORD_INTF_taskRxMsgBuff, 0) == pdPASS)
               {
                   C_I_qToQ2DoneCnt ++;
                   Serial.printf("C_I_pRF> xQS(Q2) done [%u]\n", C_I_qToQ2DoneCnt);
               }
               else
               {
                   C_I_qToQ2FlrCnt ++;
                   Serial.printf("C_I_pRF> xQS(Q2) flr [%u]!!\n", C_I_qToQ2FlrCnt);
               }
           }
           break;

      case UART_MSG_TYPE_ACK:
           {
               unsigned char hdrFlags = COORD_INTF_taskRxMsgBuff[UART_FRAME_HDR_FLAGS_FIELD_OFF];
     
               if (hdrFlags & UART_ACK_STS_OK_BM)
               {
                   hdrAcked = 1;
                   Serial.printf("Header Acked hdr flag<0x%02x> !! \n", hdrFlags);
                   if (xQueueSend(Q5Hndl, (void*)&hdrAcked, (TickType_t)0) == pdPASS)
                   {
                       Serial.printf("\nHDR Acked resp passed to Q5Hndl..\n");
                   }
                   else
                   {
                       Serial.printf("\nNo queue space in Q5Hndl...\n");
                   }
               }
               else
               {
                   Serial.printf("<%s> hdr flag <0x%02x> hdrAcked <0> !! \n", __FUNCTION__, hdrFlags);
                   hdrAcked = 0;
               }
           }
           break;

      case LPWMN_GW_MSG_TYPE_GET_COORD_ATTR_VAL:
           {
               switch (COORD_INTF_rcvdFramePyldLen)
               {
                   case 1:
                        {
                           if (GW_coordPendingAttrId == CC1120_AGC_CS_THR_ATTR_ID)
                               Serial.printf("GET_COORD_ATTR_VAL: %d \n",  
                                             *((char *)(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN)));
                           else
                               Serial.printf("GET_COORD_ATTR_VAL: %u / 0x%x \n",
                                             *(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN),
                                             *(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN));
     
                           sprintf(CoordAttrRcvdData, "0x%x", *(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN));
                           if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                           {
                               Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                           }
                           else
                           {
                               Serial.printf("\nNo queue space in Q6Hndl...\n");
                           }
                        }
                        break;

                   case 2:
                        {
                           if (GW_coordPendingAttrId == RADIO_FREQ_OFFSET_ATTR_ID)
                           {
                               short freqOff = (short)UTIL_ntohs(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);
                           
                               Serial.printf("%d\n", (int)freqOff);
                               sprintf(CoordAttrRcvdData, "%d", (int)freqOff);
                               if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                               {
                                   Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                               }
                               else
                               {
                                   Serial.printf("\nNo queue space in Q6Hndl...\n");
                               }
                           }
                           else
                               Serial.printf("%u / 0x%x\n", 
                                             UTIL_ntohs(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN), 
                                             UTIL_ntohs(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN));
          
                           sprintf(CoordAttrRcvdData, "0x%x", 
                                   UTIL_ntohs(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN));
                           if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                           {
                               Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                           }
                           else
                           {
                               Serial.printf("\nNo queue space in Q6Hndl...\n");
                           }
                        }
                        break;

                   case 4:
                        {
                           switch (GW_coordPendingAttrId)
                           {
                               default:
                                      Serial.printf("Coord_Attr_Val: <%u / 0x%x> \n",
                                                    UTIL_ntohl(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN),
                                                    UTIL_ntohl(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN));
                
                                      sprintf(CoordAttrRcvdData, "%u", 
                                              UTIL_ntohl(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN));
                                      if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                                      {
                                          Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                                      }
                                      else
                                      {
                                          Serial.printf("\nNo queue space in Q6Hndl...\n");
                                      }
                                      break;
                           }
                        }
                        break;

                   default:
                        if (GW_coordPendingAttrId == FW_BUILD_DATE_ATTR_ID  
                            || GW_coordPendingAttrId == FW_BUILD_TIME_ATTR_ID)
                        {
                            sprintf(CoordAttrRcvdData, "%s%s%s", 
                                    GW_coordPendingAttrId == FW_BUILD_DATE_ATTR_ID ? "build date <" : "build time <" , 
                                    COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN, ">");
                            if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                            {
                                Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                            }
                            else
                            {
                                Serial.printf("\nNo queue space in Q6Hndl...\n");
                            }       
                        }
                        else
                            Serial.printf("%s\n", COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);

                        sprintf(CoordAttrRcvdData, "%s", COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);
                        if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
                        {
                            Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
                        }
                        else
                        {
                            Serial.printf("\nNo queue space in Q6Hndl...\n");
                        }
                        break;
               }
           }
           break;

      case LPWMN_GW_MSG_TYPE_GET_RADIO_BAUD_RATE:
           {
               int br = 0;
               Serial.printf("\nRADIO_BAUD_RATE\n");
               if (COORD_INTF_rcvdFramePyldLen == LPWMN_GW_MSG_RADIO_BAUD_RATE_FIELD_LEN)
               {
                   br = UTIL_ntohl(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);
                   Serial.printf("%d baud \n", br);
               }
       
               sprintf(CoordAttrRcvdData, "%d", br);
               if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
               {
                   Serial.printf("\nCoord Resp passed into Q6Hndl..\n");
               }
               else
               {
                   Serial.printf("\nNo queue space in Q6Hndl...\n");
               }
           }
           break;

      case LPWMN_GW_MSG_TYPE_GET_NWK_CHANNEL:
           {
               unsigned int chann = 0;
               Serial.printf("\nRADIO Channel Ack: \n");
               if (COORD_INTF_rcvdFramePyldLen == LPWMN_GW_MSG_RADIO_CHANN_FIELD_LEN)
               {
                   chann = UTIL_ntohs(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);
                   Serial.printf("Radio Channel - %d\n", chann);
               }
               sprintf(CoordAttrRcvdData, "%d", chann);
               if (xQueueSend(Q6Hndl, (void*)&CoordAttrRcvdData, (TickType_t)0) == pdPASS)
               {
                   Serial.printf("\nCoord Resppassed into Q6Hndl...\n");
               }
               else
               {
                   Serial.printf("\nNo queue space in Q6Hndl...\n");
               }
           }
           break;

      default:
           {
               Serial.printf("C_I_pRF> Dropping msg !!\n");
           }
           break;
   }
}


boolean COORD_IF_checkProcPyld(void)
{
   boolean rc = true;
   const uint16_t calcCkSum = COORD_INTF_calcCkSum16(COORD_INTF_taskRxMsgBuff + COORD_INTF_FRAME_HDR_LEN,
                                                     COORD_INTF_rcvdFramePyldLen);
   const uint16_t rcvdCkSum = UTIL_ntohs(COORD_INTF_taskRxMsgBuff
                                         + COORD_INTF_FRAME_HDR_PYLD_CRC_FIELD_OFF);
   if (calcCkSum == rcvdCkSum)
   {
       COORD_INTF_procRcvdFrame();
   }
   else
   {
       // Bad checksum !!
       Serial.printf("Pyld CKSM MM R:%d C:%d \n", rcvdCkSum, calcCkSum);
       rc = false;
   }

   return rc;
}


int COORD_IF_procOneMsgFromCoord(uint8_t *rawBuff_p, const int rxByteCnt)
{
   int currMsgBytesTotalRcvdCnt = COORD_INTF_rxMsgOff + rxByteCnt,
       consumedCnt = 0;
       
   Serial.printf("POM En rxBC:%d c-rxMO:%d FPL:%d cMBTRC:%d \n",
                 rxByteCnt, COORD_INTF_rxMsgOff,
                 COORD_INTF_rcvdFramePyldLen,
                 currMsgBytesTotalRcvdCnt);

   if (COORD_INTF_rxMsgOff < COORD_INTF_FRAME_HDR_LEN)
   {
       if (currMsgBytesTotalRcvdCnt >= COORD_INTF_FRAME_HDR_LEN)
       {
           int toCopy = COORD_INTF_FRAME_HDR_LEN - COORD_INTF_rxMsgOff;
           Serial.printf("POM tC:%d \n", toCopy);
           memcpy(COORD_INTF_taskRxMsgBuff + COORD_INTF_rxMsgOff, rawBuff_p, toCopy);
           consumedCnt += toCopy;
           COORD_INTF_rxMsgOff += toCopy;
           if (COORD_INTF_checkForHdr(COORD_INTF_taskRxMsgBuff) == true)
           {
               if (COORD_INTF_rcvdFramePyldLen == 0)
               {
                   COORD_INTF_procRcvdFrame();
                   COORD_INTF_rcvdFramePyldLen = 0;
                   COORD_INTF_rxMsgOff = 0;
               }
           }
           else
           {
               consumedCnt = -1;
               COORD_INTF_rcvdFramePyldLen = 0;
               COORD_INTF_rxMsgOff = 0;
           }
       }
       else
       {
           memcpy(COORD_INTF_taskRxMsgBuff + COORD_INTF_rxMsgOff,
                  rawBuff_p,
                  rxByteCnt);
           consumedCnt += rxByteCnt;
       }
   }
   else
   {
       int currMsgTotCalcLen = COORD_INTF_FRAME_HDR_LEN + COORD_INTF_rcvdFramePyldLen,
                               toCopy = rxByteCnt;
       if (currMsgBytesTotalRcvdCnt > currMsgTotCalcLen)
           toCopy = currMsgTotCalcLen - COORD_INTF_rxMsgOff;

       memcpy(COORD_INTF_taskRxMsgBuff + COORD_INTF_rxMsgOff,
              rawBuff_p,
              toCopy);
       consumedCnt += toCopy;
       COORD_INTF_rxMsgOff += toCopy;

       if (COORD_INTF_rxMsgOff >= currMsgTotCalcLen)
       {
           if (COORD_IF_checkProcPyld() == false)
               consumedCnt = -1;
           COORD_INTF_rcvdFramePyldLen = 0;
           COORD_INTF_rxMsgOff = 0;
       }
   }
   
   Serial.printf("POM Ex FPL:%d c-rxMO:%d cC:%d \n",
                 COORD_INTF_rcvdFramePyldLen, COORD_INTF_rxMsgOff,
                 consumedCnt);

   return consumedCnt;
}


void COORD_IF_procBytesFromCoord(int rxByteCnt)
{
   int totConsumedCnt = 0;
   
   Serial.printf("PRB rBC:%d\n", rxByteCnt);
   
   while (rxByteCnt > 0)
   {
      Serial.printf("PRB L1 rBC:%d tCC:%d \n", rxByteCnt, totConsumedCnt);
      int procCnt = COORD_IF_procOneMsgFromCoord(COORD_INTF_taskRxRawBuff + totConsumedCnt, 
                                                 rxByteCnt);
      Serial.printf("PRB L2 pC:%d\n", procCnt);
      if (procCnt < 0)
          break;
      rxByteCnt -= procCnt;
      totConsumedCnt += procCnt;
      Serial.printf("PRB L3 rBC:%d tCC:%d \n", rxByteCnt, totConsumedCnt);
      if (rxByteCnt < 0)
      {
          Serial.printf("PRB Bug hit rBC:%d !!!!  \n", rxByteCnt);
          // Should never happen - blink the LEDs to indicate a bug !!
          for (;;);
      }
   }
   
   return;
}


static void COORD_INTF_uartRxProcTask(void *params_p)
{
   uart_event_t event;

   while (1)
   {
      if (xQueueReceive(CoordIntf_uartRxQEvtHndl, (void * )&event, (portTickType)portMAX_DELAY))
      {
          switch (event.type)
          {
            case UART_DATA:
                 {
                    do
                    {
                       int toReadCnt = 0, readCnt = 0, rcvdByteCnt = 0;
                       ESP_ERROR_CHECK(uart_get_buffered_data_len(COORD_INTF_SERIAL_PORT_NR,
                                                                  (size_t *)&rcvdByteCnt));
                       if (rcvdByteCnt == 0)
                           break;
                       toReadCnt = rcvdByteCnt > COORD_INTF_RAW_BUFF_LEN ? \
                                   rcvdByteCnt : COORD_INTF_RAW_BUFF_LEN;
                       readCnt = uart_read_bytes(COORD_INTF_SERIAL_PORT_NR,
                                                 COORD_INTF_taskRxRawBuff,
                                                 toReadCnt, 100);
                       COORD_IF_procBytesFromCoord(readCnt);
                    } while (1);
                 }
                 break;

            case UART_FRAME_ERR:
                 {
                 }
                 break;

            default:
                 {
                 }
                 break;
          }
      } 
   }  
   
   return;
}


int UTIL_writeToUART(unsigned char *buff_p, unsigned int cnt)
{
   int rc;
  
   Serial.printf("\nCnt: <%d> \n", cnt);
   Serial.printf("Buff data: ");
   for (int i = 0; i < cnt; i++)
       Serial.printf("0x%02x |", buff_p[i]);
   rc = uart_write_bytes(COORD_INTF_SERIAL_PORT_NR, (const char*) buff_p, cnt );
   Serial.printf("\nUART RC: <%d> !! \n", rc);
   if (rc != cnt)
      return -1;
   return 1;
}


int CoordIf_buildSendHdrToCoord(int msgType, unsigned char *pyldBuff_p, int pyldLen)
{
   unsigned char *buff_p = serTxBuff;
   unsigned short calcCrc16;
   static unsigned char seqNr = 0x0;
   int rc;

   Serial.printf("\nmsgType <0x%x>", msgType);

   UTIL_htons(buff_p, msgType);
   buff_p += UART_FRAME_HDR_MSG_TYPE_FIELD_LEN;

   *buff_p = 0x0;
   buff_p += UART_FRAME_HDR_FLAGS_FIELD_LEN;

   *buff_p = seqNr ++;
   buff_p += UART_FRAME_HDR_SEQ_NR_FIELD_LEN;

   UTIL_htons(buff_p, pyldLen);
   buff_p += UART_FRAME_HDR_PYLD_LEN_FIELD_LEN;

   calcCrc16 = COORD_INTF_calcCkSum16(serTxBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
   UTIL_htons(buff_p, calcCrc16);
   buff_p += UART_FRAME_HDR_HDR_CRC_FIELD_LEN;

   if (pyldLen > 0)
   {
       calcCrc16 = COORD_INTF_calcCkSum16(pyldBuff_p, pyldLen);
       UTIL_htons(buff_p, calcCrc16);
   }
   else
       UTIL_htons(buff_p, 0x0);

   rc = UTIL_writeToUART(serTxBuff, UART_FRAME_HDR_LEN);
   if (rc != 1)
   {
       Serial.printf("\nUTIL_writeToUART() failed !!\n");
       rc = 20;
   }
   
   return rc;
}

 
void setup()
{
  SerialMon.begin(115200);
  
  delay(10);
  
  uart_param_config(COORD_INTF_SERIAL_PORT_NR,
                    &CoordIntf_uartConfigInfo);
                    
  uart_set_pin(COORD_INTF_SERIAL_PORT_NR,
               COORD_INTF_RX_PIN,
               COORD_INTF_TX_PIN,
               UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  uart_driver_install(COORD_INTF_SERIAL_PORT_NR,
                      COORD_INTF_UART_DRIVER_BUFF_SIZE,
                      COORD_INTF_UART_DRIVER_BUFF_SIZE,
                      COORD_INTF_EVT_QUEUE_SIZE,
                      &CoordIntf_uartRxQEvtHndl,
                      0);

  /*
   * QueueHandle_t xQueueCreate(maxQElementCnt, elementSz);
   * 
   * Param: maxQElementCnt
   *        max number of elements the queue can hold at a given time.
   *        
   * Param: elementSz
   *        The size (in bytes) of each element 
   *        
   * Return value:
   *        Success: return an handle for the queue, which is of type QueueHandle_t 
   *        Failure: NULL 
   */
  Q2Hndl = xQueueCreate(QUEUE_Q2_ENTRY_CNT, COORD_INTF_MSG_BUFF_LEN);
  assert(Q2Hndl != NULL);
  
  Q3Hndl = xQueueCreate(QUEUE_Q3_ENTRY_CNT, sizeof(char) * 1024);
  assert(Q3Hndl != NULL);
  
  Q4Hndl = xQueueCreate(QUEUE_Q4_ENTRY_CNT, sizeof(char) * 1024);
  assert(Q4Hndl != NULL);
  
  Q5Hndl = xQueueCreate(QUEUE_Q5_ENTRY_CNT, sizeof(hdrAcked));
  assert(Q5Hndl != NULL);
  
  Q6Hndl = xQueueCreate(QUEUE_Q6_ENTRY_CNT, sizeof(char) * 1024);
  assert(Q6Hndl != NULL);
  
  xTaskCreate(COORD_INTF_uartRxProcTask, "COORD_INTF_UART_RX_PATH_TASK", 20000, NULL, 12, NULL);

  xTaskCreate(COORD_INTF_coordMsgProcTask, "COORD_INTF_COORD_MSG_PROC_TASK", 20000, NULL, 12, NULL);

  xTaskCreate(CloudIf_rxPathTask, "CLOUD_INTF_RX_PATH_TASK", 20000, NULL, 12, NULL);

  GSM_serialObj.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(10);

  MQTT_setUp();

  DBG("Setup Complete :-)");
}

void loop()
{ 
   if (MODEM_restarted == false)
   {
      DBG("Configuring the modem .... ");
      if (!modem.restart())
      {
          DBG("Failed to restart modem ... delaying 10s before retrying !!");
          delay(1 * 1000);
          return;
      }
      else
      {
          MODEM_restarted = true;
      }
   }
  
   if (modem.isNetworkConnected() == false)
   {
       DBG("Registering to the network .... ");
      
       if (!modem.waitForNetwork(600000L))
       {
           delay(1 * 1000);
           GSM_status.gsmNetwrkCntSts = false;
           return;
       }
      
       if (modem.isNetworkConnected())
       {
           DBG("Registered to the network  :-) ");
           GSM_status.gsmNetwrkCntSts = true;
       }
       else
       {
           DBG("Still not registered to the network ... delaying for 10 secs before retrying !!!");
           GSM_status.gsmNetwrkCntSts = false;
           delay(1 * 1000);
           return;
       }
   }
   else
   {
       GSM_status.gsmNetwrkCntSts = true;
   }

   if (modem.isGprsConnected() == false)
   {
       DBG("Connecting to", apn);
       if (!modem.gprsConnect(apn, gprsUser, gprsPass))
       {
           DBG("GPRS connection setup failed ... delaying for 10 secs before retrying !!! ");
           delay(1 * 1000);
           GSM_status.gsmGprsCntSts = false;
           return;
       }
       else
       {
           DBG("GPRS connection setup successful :-)");
           DBG("-----------------------------------------------");
           String ccid = modem.getSimCCID();
           DBG("CCID:", ccid);

           String imei = modem.getIMEI();
           DBG("IMEI:", imei);

           String imsi = modem.getIMSI();
           DBG("IMSI:", imsi);
 
           String cop = modem.getOperator();
           DBG("Operator:", cop);
 
           IPAddress local = modem.localIP();
           DBG("Local IP:", local);
 
           int csq = modem.getSignalQuality();
           DBG("Signal quality:", csq);
           DBG("-----------------------------------------------");
           GSM_status.gsmGprsCntSts = true;
       }
   }
   else
   {
       GSM_status.gsmGprsCntSts = true;
   }

   MQTT_connChecks();
  
   if (GSM_status.gsmNetwrkCntSts && GSM_status.gsmGprsCntSts)
   {
       GSM_status.gsmCntSts = true;
   }

   COORD_IF_checkCoordResp();

   MQTT_loop();
  
   return;
}
