/*
 *********************************************************************
 * File: gsm_intf_main.ino
 * 
 * Author: rkris@wisense.in / siva@wisense.in
 * 
 * Date: March/2021
 * 
 * Copyright (C) WiSense Technologies Pvt Ltd
 * All rights reserved.
 * *******************************************************************
 */


//#define BACKHAUL_TYPE_CELLULAR
#define BACKHAUL_TYPE_WIFI

#if defined(BACKHAUL_TYPE_CELLULAR) && defined(BACKHAUL_TYPE_WIFI)
#error Specify only one type of backhaul !!
#endif

#if !defined(BACKHAUL_TYPE_CELLULAR) && !defined(BACKHAUL_TYPE_WIFI)
#error Backhaul type not specified !!
#endif

#include "driver/uart.h"
#include "coord_intf.h"
#include "dis.h"
#include "gw.h"
#include "pltfrm.h"
#include "wsuart.h"
#include "node_attr.h"

// #define WSN_COORD_SIM_ENA

#define TINY_GSM_MODEM_SIM868

/*
   There are three serial ports on the ESP32 known as U0UXD, U1UXD and U2UXD all
   work at 3.3V TTL Level. There are three hardware supported serial interfaces
   on the ESP32 known as UART0, UART1 and UART2. Like all peripherals, the pins
   for the UARTs can be logically mapped to any of the available pins on the ESP32.
   However, the UARTs can als o have direct access which marginally improves
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

#define LED_PIN 13

const char* get_attr = "get_attr";

#define COORD_INTF_SERIAL_PORT_NR  UART_NUM_1
#define COORD_INTF_RX_PIN  18
#define COORD_INTF_TX_PIN  19

#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifdef BACKHAUL_TYPE_CELLULAR

// -----------------------------------------------------------------


#define GSM_serialObj Serial2
#define GSM_RX_PIN  16
#define GSM_TX_PIN  17

#define TINY_GSM_DEBUG  SerialMon
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 57600

#include <TinyGsmClient.h>
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

TinyGsm CLOUD_IF_tinyGSMObj(GSM_serialObj);

/*
   Create one or more TinyGSM client instances
   For a single connection, use "TinyGsmClient client(modem);".
*/
TinyGsmClient CLOUD_IF_tinyGSMClient0(CLOUD_IF_tinyGSMObj, 0);
TinyGsmClient CLOUD_IF_tinyGSMClient1(CLOUD_IF_tinyGSMObj, 1);
TinyGsmClient CLOUD_IF_tinyGSMClient2(CLOUD_IF_tinyGSMObj, 2);

PubSubClient CLOUD_IF_reqFromCloudMQTTClient(CLOUD_IF_tinyGSMClient0);
PubSubClient CLOUD_IF_respToCloudMQTTClient(CLOUD_IF_tinyGSMClient1);
PubSubClient CLOUD_IF_nodesMQTTClient(CLOUD_IF_tinyGSMClient2);
// -----------------------------------------------------------------

#elif defined(BACKHAUL_TYPE_WIFI)

// -----------------------------------------------------------------
#include <WiFi.h>
const char WIFI_ssid[] = "WiSe-2G"; // CHS_219 McCauley";
const char WIFI_pswd[] = "Ccube4all"; // "HumanSc1";

WiFiClient CLOUD_IF_wifiClient1;
WiFiClient CLOUD_IF_wifiClient2;
WiFiClient CLOUD_IF_wifiClient3;

// For requests from wisense dashboard in the cloud
PubSubClient CLOUD_IF_reqFromCloudMQTTClient(CLOUD_IF_wifiClient1);  

// For response from GW to prior request from wisense dashboard in the cloud
PubSubClient CLOUD_IF_respToCloudMQTTClient(CLOUD_IF_wifiClient2);

// For sending node data to wisense dashboard in the cloud
PubSubClient CLOUD_IF_nodesMQTTClient(CLOUD_IF_wifiClient3);
// -----------------------------------------------------------------

#else
#error Not Supported !!
#endif

/*
#ifdef BACKHAUL_TYPE_CELLULAR

#elif defined(BACKHAUL_TYPE_WIFI)


#else
#error Not Supported !!
#endif
*/

#define Q3_MSG_Q_BUFF_LEN  320
#define Q4_MSG_Q_ENTRY_SZ  320

typedef struct
{
  uint32_t pubCnt;
  uint32_t pubFlrCnt;
  uint32_t pubSuccessCnt;
  uint32_t pubFlrConnUpCnt;
  uint32_t pubFlrConnDownCnt;
  uint32_t sessNotFndCnt;
  uint32_t pubOkCnt;
} MQTT_stats_t;

typedef struct
{
  unsigned char snsrId;
  float snsrVal;
} COORD_IF_snsrDataEntry_t;

#define COORD_IF_MAX_SENSOR_DATA_CNT_PER_MSG  8

COORD_IF_snsrDataEntry_t  COORD_IF_snsrDataList[COORD_IF_MAX_SENSOR_DATA_CNT_PER_MSG];

MQTT_stats_t MQTT_stats = {0, 0, 0, 0, 0};

int ledStatus = LOW;
int hdrAcked = 0, 
    hdrAckSts = 0;

#define QUEUE_Q2_ENTRY_CNT  8
#define QUEUE_Q3_ENTRY_CNT  8
#define QUEUE_Q4_ENTRY_CNT  8
#define QUEUE_Q5_ENTRY_CNT  1
#define QUEUE_Q6_ENTRY_CNT  1
#define QUEUE_Q7_ENTRY_CNT  2

QueueHandle_t CoordIntf_uartRxQEvtHndl;
QueueHandle_t Q2Hndl;
QueueHandle_t Q3Hndl;
QueueHandle_t Q4Hndl;
QueueHandle_t Q5Hndl;
QueueHandle_t Q6Hndl;
QueueHandle_t Q7Hndl;

uint32_t COORD_IF_jsonMsgSeqNr = 0;

char COORD_IF_snsrDataJSONMsg[256];
    
DynamicJsonDocument COORD_IF_snsrDataJSONDoc(512);
JsonObject COORD_IF_snsrDataJsonObj = COORD_IF_snsrDataJSONDoc.to<JsonObject>();

DynamicJsonDocument CLOUD_IF_wsnMsgJSONDoc(1024);
JsonObject CLOUD_IF_wsnMsgJSONObj = CLOUD_IF_wsnMsgJSONDoc.to<JsonObject>();


#define SER_BUFF_LEN 256
unsigned char COORD_IF_serTxBuff[SER_BUFF_LEN];
unsigned char serRxBuff[SER_BUFF_LEN];

boolean MODEM_restarted = false;

#define COORD_INTF_UART_DRIVER_BUFF_SIZE (1024 * 2)
#define COORD_INTF_EVT_QUEUE_SIZE  2

#define COORD_INTF_MSG_BUFF_LEN  256
#define COORD_INTF_RAW_BUFF_LEN  256

uint8_t COORD_INTF_taskRxRawBuff[COORD_INTF_RAW_BUFF_LEN];
uint8_t COORD_INTF_taskRxMsgBuff[COORD_INTF_MSG_BUFF_LEN];

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


boolean COORD_INTF_checkForHdr(uint8_t *hdr_p)
{
  boolean rc = false;
  const uint16_t calcCkSum = COORD_IF_calcCkSum16(hdr_p,
                                                    COORD_INTF_FRAME_HDR_LEN \
                                                    - COORD_INTF_FRAME_HDR_CRC_FIELD_LEN * 2);
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


static char NODE_devName[64];
static char NODE_macId[64];

int COORD_IF_q3EnqOkCnt = 0, COORD_IF_q3EnqFlrCnt = 0;

void COORD_IF_buildFwdSensorData(const char *nodeEA_p,
                                 const int rssi, 
                                 const int lqi,
                                 const int snsrDataCnt)
{
  int totMsgLen = 0, jsonMsgLen = 0, idx;
  char *jsonMsgPtr_p = NULL;
  char *msg_p = NULL;

  sprintf(NODE_devName, "WSN-%02x%02x%02x%02x", 
          nodeEA_p[4], nodeEA_p[5], nodeEA_p[6], nodeEA_p[7]);
  sprintf(NODE_macId, "%02x:%02x:%02x:%02x", 
          nodeEA_p[4], nodeEA_p[5], nodeEA_p[6], nodeEA_p[7]);
  
  Serial.printf("GW_bFSD> Entry %s \n", NODE_macId);
  COORD_IF_jsonMsgSeqNr ++;
  
  COORD_IF_snsrDataJsonObj["Name"] = NODE_devName;
  COORD_IF_snsrDataJsonObj["MacId"] = NODE_macId;
  COORD_IF_snsrDataJsonObj["RSSI"] = rssi;
  COORD_IF_snsrDataJsonObj["LQI"] = lqi;

  for (idx=0; idx<snsrDataCnt; idx++)
  {
     char snsrIdStr[6];
     char snsrOpStr[16];

     sprintf(snsrIdStr, "0x%x", COORD_IF_snsrDataList[idx].snsrId);
     sprintf(snsrOpStr, "%.2f", COORD_IF_snsrDataList[idx].snsrVal);
     COORD_IF_snsrDataJsonObj[snsrIdStr] = snsrOpStr;
  }
  
  COORD_IF_snsrDataJsonObj["Sequence_number"] = COORD_IF_jsonMsgSeqNr;
  
  serializeJson(COORD_IF_snsrDataJSONDoc, COORD_IF_snsrDataJSONMsg);

  Serial.printf("\n\nGW_bFSD> C JSON M: ");
  Serial.printf(COORD_IF_snsrDataJSONMsg);
  Serial.printf("\n\n");

  // Send a message with extended MAC address + serialized JSON message + null character

  jsonMsgLen = strlen(COORD_IF_snsrDataJSONMsg) + 1;  // We will keep the terminating NULL character
  totMsgLen = DIS_LPWMN_MAC_EXT_ADDR_LEN + jsonMsgLen;
  msg_p = (char*)malloc(totMsgLen);
  if (msg_p == NULL)
  {
      Serial.printf("GW_bFSD> malloc(%d) flr !!\n", totMsgLen);
      return;
  }

  memcpy(msg_p, nodeEA_p, DIS_LPWMN_MAC_EXT_ADDR_LEN);
  jsonMsgPtr_p = msg_p + DIS_LPWMN_MAC_EXT_ADDR_LEN;  
  memcpy(jsonMsgPtr_p, COORD_IF_snsrDataJSONMsg, jsonMsgLen);
  Serial.printf("GW_bFSD> jsonMsgLen <%d>\n", jsonMsgLen);
  Serial.printf("GW_bFSD> jsonMsgPtr_p:");
  Serial.printf(jsonMsgPtr_p);
  Serial.printf("\n\n");
  
  if (xQueueSend(Q3Hndl, (void*)msg_p, (TickType_t)5) == pdPASS)
  {
      COORD_IF_q3EnqOkCnt ++;
      Serial.printf("GW_bFSD> JM #%d -> Q3\n", COORD_IF_q3EnqOkCnt);
  }
  else
  {
      COORD_IF_q3EnqFlrCnt ++;
      Serial.printf("GW_bFSD> JM -> Q3 flr #%d\n", COORD_IF_q3EnqFlrCnt);
  }

  free(msg_p);

  return;
}


void COORD_IF_procNodeMsg(uint8_t *buff_p, int msgPyldLen)
{
  int srcShortAddr, off = 0, snsrDataDropCnt = 0,
      snsrOutputTLVCnt = 0;
  unsigned int disMsgType;
  uint8_t *extAddr_p;
  int node_rssi = -127, node_lqi = 0;
  
  COORD_IF_nodeMsgCnt ++;

  snsrOutputTLVCnt = 0;

  memset(COORD_IF_snsrDataList, 0, sizeof(COORD_IF_snsrDataList));
  
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

                         if (snsrOutputTLVCnt < COORD_IF_MAX_SENSOR_DATA_CNT_PER_MSG)
                         {
                             COORD_IF_snsrDataList[snsrOutputTLVCnt].snsrId = snsrId;
                             COORD_IF_snsrDataList[snsrOutputTLVCnt].snsrVal = valF;
                             Serial.printf("C_I_PNM> SD#%d  ID[%d] V[%.2f]\n", 
                                           snsrOutputTLVCnt,
                                           snsrId, valF);   
                             snsrOutputTLVCnt ++;
                         }
                         else
                         {
                             Serial.printf("C_I_PNM> D SD[%d] !! \n", snsrDataDropCnt);    
                             snsrDataDropCnt ++;                         
                         }
                     }
                 }
             }
         }
     }
  }
  

  COORD_IF_buildFwdSensorData((const char *)extAddr_p, node_rssi, node_lqi, snsrOutputTLVCnt);

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


char CoordAttrRcvdData[100];

void COORD_INTF_procRcvdFrame(void)
{
   static uint32_t C_I_qToQ2DoneCnt = 0, C_I_qToQ2FlrCnt = 0;
   uint16_t msgType = UTIL_ntohs(COORD_INTF_taskRxMsgBuff);

   Serial.printf("C_I_pRF> T:%d PL:%d\n", (int)msgType, COORD_INTF_rcvdFramePyldLen);
 
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
   const uint16_t calcCkSum = COORD_IF_calcCkSum16(COORD_INTF_taskRxMsgBuff + COORD_INTF_FRAME_HDR_LEN,
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


int COORD_IF_buildSendHdrToCoord(const int msgType, 
                                 const uint8_t *pyldBuff_p, 
                                 const int pyldLen)
{
   uint8_t *buff_p = COORD_IF_serTxBuff;
   uint16_t calcCrc16;
   static uint8_t seqNr = 0x0;
   int rc;

   Serial.printf("C_I_bSH2C> mT:%d  L:%d\n", 
                 msgType, pyldLen);

   UTIL_htons(buff_p, msgType);
   buff_p += UART_FRAME_HDR_MSG_TYPE_FIELD_LEN;

   *buff_p = 0x0;
   buff_p += UART_FRAME_HDR_FLAGS_FIELD_LEN;

   *buff_p = seqNr ++;
   buff_p += UART_FRAME_HDR_SEQ_NR_FIELD_LEN;

   UTIL_htons(buff_p, pyldLen);
   buff_p += UART_FRAME_HDR_PYLD_LEN_FIELD_LEN;

   calcCrc16 = COORD_IF_calcCkSum16(COORD_IF_serTxBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
   UTIL_htons(buff_p, calcCrc16);
   buff_p += UART_FRAME_HDR_HDR_CRC_FIELD_LEN;

   if (pyldLen > 0)
   {
       calcCrc16 = COORD_IF_calcCkSum16(pyldBuff_p, pyldLen);
       UTIL_htons(buff_p, calcCrc16);
   }
   else
       UTIL_htons(buff_p, 0x0);

   rc = UTIL_writeToUART(COORD_IF_serTxBuff, UART_FRAME_HDR_LEN);
   if (rc != 1)
   {
       Serial.printf("\nUTIL_writeToUART() failed !!\n");
       rc = 20;
   }
   
   return rc;
}

const uint8_t WSN_COORD_SIM_nodeAExtAddr[ ] = {0xfc, 0xc2, 0x3d, 0xff, 0xfe, 0x0d, 0x4a, 0x46};
char *WSN_COORD_SIM_nodeAMqttConnUser = "p0lsPBWqHD2AWhmKUlEu";

#ifdef WSN_COORD_SIM_ENA
const uint8_t WSN_COORD_SIM_nodeAExtAddr[ ] = {0xfc, 0xc2, 0x3d, 0xff, 0xfe, 0x0d, 0x4a, 0x46};
const uint8_t WSN_COORD_SIM_nodeBExtAddr[ ] = {0xfc, 0xc2, 0x3d, 0xff, 0xfe, 0x0d, 0x5f, 0x7c};
char *WSN_COORD_SIM_nodeAMqttConnUser = "p0lsPBWqHD2AWhmKUlEu";
char *WSN_COORD_SIM_nodeBMqttConnUser = "st8T3pTKQ2LUBWBhCrzk";

uint8_t SIM_msgFromA[] = 
{
  0x00, 0x06, 0x00, 0x04, 0x00, 0x2d, 0xff, 0xc8,
  0x8d, 0x74, 0x08, 0xf8, 0xfc, 0xc2, 0x3d, 0xff,
  0xfe, 0x0d, 0x4a, 0x46, 0xd2, 0x01, 0x03, 0x10,
  0x1e, 0x11, 0x07, 0x14, 0x01, 0x78, 0x02, 0x02,
  0x0b, 0xb3, 0x11, 0x0a, 0x14, 0x01, 0x79, 0x04,
  0x01, 0x08, 0x02, 0x02, 0x0c, 0x57, 0x11, 0x07,
  0x14, 0x01, 0x08, 0x02, 0x02, 0x0c, 0x92
};

uint8_t SIM_msgFromB[] = 
{
  0x00, 0x06, 0x00, 0x08, 0x00, 0x2d, 0xff, 0xc4,
  0x72, 0x5e, 0x08, 0xf9, 0xfc, 0xc2, 0x3d, 0xff,
  0xfe, 0x0d, 0x5f, 0x7c, 0xd6, 0x03, 0x03, 0x10,
  0x1e, 0x11, 0x07, 0x14, 0x01, 0x78, 0x02, 0x02,
  0x0b, 0xea, 0x11, 0x0a, 0x14, 0x01, 0x79, 0x04,
  0x01, 0x08, 0x02, 0x02, 0x0c, 0xd3, 0x11, 0x07,
  0x14, 0x01, 0x08, 0x02, 0x02, 0x01, 0x49
};

static void WSN_COORD_simTask(void *params_p)
{
  Serial.printf("Simtask Macid: ");
  for(int indx = 0; indx < 8; indx ++)
  {
      Serial.printf("%s | %d  | %x||\n", WSN_COORD_SIM_nodeBExtAddr[indx], WSN_COORD_SIM_nodeBExtAddr[indx], WSN_COORD_SIM_nodeBExtAddr[indx]);
  }
  Serial.printf("\n");
   int simMsgCnt = 0;
   
   while (1)
   {
      uint8_t *msg_p;

     
      delay(20000);

      if ((simMsgCnt & 0x1) == 0)
      {
          
          msg_p = SIM_msgFromA;
          COORD_INTF_rcvdFramePyldLen = sizeof(SIM_msgFromA);
          uint16_t val = UTIL_ntohs(msg_p + COORD_INTF_rcvdFramePyldLen - 2);
          val += 10;
          if (val >= 7000)
              val = 3218;
          UTIL_htons(msg_p + COORD_INTF_rcvdFramePyldLen - 2, val);
          Serial.printf("[%d] Msg from Node A \n", simMsgCnt);
      }
      else
      {
          msg_p = SIM_msgFromB;
          COORD_INTF_rcvdFramePyldLen = sizeof(SIM_msgFromB);
          uint16_t val = UTIL_ntohs(msg_p + COORD_INTF_rcvdFramePyldLen - 2);
          val += 10;
          if (val >= 3000)
              val = 149;
          UTIL_htons(msg_p + COORD_INTF_rcvdFramePyldLen - 2, val);
          Serial.printf("[%d] Msg from Node B \n", simMsgCnt);
      }
      
      simMsgCnt ++;
       
      memcpy(COORD_INTF_taskRxMsgBuff, msg_p, COORD_INTF_rcvdFramePyldLen);
       
      COORD_INTF_procRcvdFrame();
   }
  
}
#endif
 
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
  
  Q3Hndl = xQueueCreate(QUEUE_Q3_ENTRY_CNT, Q3_MSG_Q_BUFF_LEN);
  assert(Q3Hndl != NULL);
  
  Q4Hndl = xQueueCreate(QUEUE_Q4_ENTRY_CNT, Q4_MSG_Q_ENTRY_SZ);
  assert(Q4Hndl != NULL);
  
  Q5Hndl = xQueueCreate(QUEUE_Q5_ENTRY_CNT, sizeof(hdrAcked));
  assert(Q5Hndl != NULL);
  
  Q6Hndl = xQueueCreate(QUEUE_Q6_ENTRY_CNT, sizeof(char) * 1024);
  assert(Q6Hndl != NULL);

  Q7Hndl = xQueueCreate(QUEUE_Q7_ENTRY_CNT, sizeof(char)* 1024);
  assert(Q7Hndl != NULL);
  
  xTaskCreate(COORD_INTF_uartRxProcTask, "COORD_INTF_UART_RX_PATH_TASK", 20000, NULL, 12, NULL);

  xTaskCreate(COORD_INTF_coordMsgProcTask, "COORD_INTF_COORD_MSG_PROC_TASK", 20000, NULL, 12, NULL);

  xTaskCreate(Cloud_IF_rxPathTask, "CLOUD_INTF_RX_PATH_TASK", 20000, NULL, 12, NULL);

#ifdef BACKHAUL_TYPE_CELLULAR
  GSM_serialObj.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(10);
#elif defined(BACKHAUL_TYPE_WIFI)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(WIFI_ssid, WIFI_pswd);
#else
#error Not Supported !!
#endif

  MQTT_init();

  MQTT_allocNewSession(WSN_COORD_SIM_nodeAExtAddr,
                       WSN_COORD_SIM_nodeAMqttConnUser,
                       NULL);
  

#ifdef WSN_COORD_SIM_ENA
  MQTT_allocNewSession(WSN_COORD_SIM_nodeAExtAddr,
                       WSN_COORD_SIM_nodeAMqttConnUser,
                       NULL);

  MQTT_allocNewSession(WSN_COORD_SIM_nodeBExtAddr,
                       WSN_COORD_SIM_nodeBMqttConnUser,
                       NULL);

  xTaskCreate(WSN_COORD_simTask, "COORD_SIM_TASK", 20000, NULL, 12, NULL);
#endif

  Serial.printf("main> S C \n");

  return;
}


#ifdef BACKHAUL_TYPE_CELLULAR
void loop()
{ 
   if (MODEM_restarted == false)
   {
      DBG("Configuring the modem .... ");
      if (!CLOUD_IF_tinyGSMObj.restart())
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
  
   if (CLOUD_IF_tinyGSMObj.isNetworkConnected() == false)
   {
       DBG("Registering to the network .... ");
      
       if (!CLOUD_IF_tinyGSMObj.waitForNetwork(600000L))
       {
           delay(1 * 1000);
           return;
       }
      
       if (CLOUD_IF_tinyGSMObj.isNetworkConnected())
       {
           DBG("Registered to the network  :-) ");
       }
       else
       {
           DBG("Still not registered to the network ... delaying for 10 secs before retrying !!!");
           delay(1 * 1000);
           return;
       }
   }
   else
   {
      
   }

   if (CLOUD_IF_tinyGSMObj.isGprsConnected() == false)
   {
       DBG("Connecting to", apn);
       if (!CLOUD_IF_tinyGSMObj.gprsConnect(apn, gprsUser, gprsPass))
       {
           DBG("GPRS connection setup failed ... delaying for 1 secs before retrying !!! ");
           delay(1 * 1000);
           return;
       }
       else
       {
           DBG("GPRS connection setup successful :-)");
           DBG("-----------------------------------------------");
           String ccid = CLOUD_IF_tinyGSMObj.getSimCCID();
           DBG("CCID:", ccid);

           String imei = CLOUD_IF_tinyGSMObj.getIMEI();
           DBG("IMEI:", imei);

           String imsi = CLOUD_IF_tinyGSMObj.getIMSI();
           DBG("IMSI:", imsi);
 
           String cop = CLOUD_IF_tinyGSMObj.getOperator();
           DBG("Operator:", cop);
 
           IPAddress local = CLOUD_IF_tinyGSMObj.localIP();
           DBG("Local IP:", local);
 
           int csq = CLOUD_IF_tinyGSMObj.getSignalQuality();
           DBG("Signal quality:", csq);
           DBG("-----------------------------------------------");
       }
   }
   else
   {
       CLOUD_IF_reqFromCloudMQTTConn();
   }

   CLOUD_IF_sendPendMsgToCloud();

   MQTT_loop();
  
   return;
}
#elif defined(BACKHAUL_TYPE_WIFI)

/*
 * WiFi Status codes
 * 
 * > (3) WL_CONNECTED: assigned when connected to a WiFi network;
 * > (255) WL_NO_SHIELD: assigned when no WiFi shield is present;
 * > (?) WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called 
 *                       and remains active until the number of attempts expires (resulting 
 *                       in WL_CONNECT_FAILED) or a connection is established (resulting in 
 *                       WL_CONNECTED);
 * > (1) WL_NO_SSID_AVAIL: assigned when no SSID are available;
 * > (2) WL_SCAN_COMPLETED: assigned when the scan networks is completed;
 * > (4) WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
 * > (5) WL_CONNECTION_LOST: assigned when the connection is lost;
 * > (6) WL_DISCONNECTED: assigned when disconnected from a network;
 */

void loop()
{ 
   static int prevWiFiSts = 0, currWiFiSts;
   static unsigned int __wifiBeginCnt = 0;

   currWiFiSts = WiFi.status();
   if (currWiFiSts != WL_CONNECTED)
   {
       if ((currWiFiSts != prevWiFiSts)
           && ((currWiFiSts == WL_CONNECTION_LOST)
               || (currWiFiSts == WL_DISCONNECTED)
               || (currWiFiSts == WL_NO_SSID_AVAIL)
               || (currWiFiSts == WL_CONNECT_FAILED)
              )
           )
       {
           Serial.printf("L WiFi NC CS:%d/PS:%d !! \n", 
                         currWiFiSts, prevWiFiSts);
           WiFi.disconnect(); 
           delay(100);
           WiFi.begin(WIFI_ssid, WIFI_pswd);  
           delay(1000); 
           __wifiBeginCnt ++;       
       }
   }
   else
   {
       if (prevWiFiSts != WL_CONNECTED)
       {
           Serial.printf("L WiFi C CS:%d/PS:%d \n", 
                         currWiFiSts, prevWiFiSts);
           Serial.printf("WiFi IP: "); 
           Serial.println(WiFi.localIP());
       }

       CLOUD_IF_reqFromCloudMQTTConn();
   }
   
   prevWiFiSts = currWiFiSts;

   CLOUD_IF_sendPendMsgToCloud();

   MQTT_loop();

   CLOUD_IF_newEntryAllocation();

  
   return;
}
#else
#error Not Supported !!
#endif
