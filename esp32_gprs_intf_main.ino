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
#define GSM_serialObj Serial2
#define GSM_RX_PIN  16
#define GSM_TX_PIN  17

#define COORD_INTF_SERIAL_PORT_NR  UART_NUM_1
#define COORD_INTF_RX_PIN  18 
#define COORD_INTF_TX_PIN  19

#define TINY_GSM_DEBUG  SerialMon
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 57600

#define EVENT_FMT_TYPE_MERGED_DATA_JSON

const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* MQTT_brokerURL = "broker.hivemq.com";
const char* MQTT_brokerWsURL = "dashboard.wisense.in";
const char* MQTT_topicLed = "GsmClientTest/led";
const char* MQTT_topicInit = "GsmClientTest/init";
const char* MQTT_topicLedStatus = "GsmClientTest/ledStatus";
const char* get_attr = "get_attr";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

TinyGsm modem(GSM_serialObj);

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

/*
   Create one or more TinyGSM client instances
   For a single connection, use "TinyGsmClient client(modem);".
*/
TinyGsmClient TinyGSM_client_1(modem, 0);
TinyGsmClient TinyGSM_client_2(modem, 1);
TinyGsmClient TinyGSM_client_coord(modem, 2);
TinyGsmClient TinyGSM_client_coordResp(modem,3);

/*
   The PubSubClient library provides a client for doing simple
   publish/subscribe messaging with a server that supports MQTT.
*/
PubSubClient MQTT_client_01(TinyGSM_client_1);
PubSubClient MQTT_client_02(TinyGSM_client_2);
PubSubClient MQTT_client_coord(TinyGSM_client_coord);
PubSubClient MQTT_client_coordResp(TinyGSM_client_coordResp);

bool GSM_CNT_STS = false,
     GSM_NETWRK_CNT_STS = false,
     GSM_GPRS_CNT_STS = false,
     GSM_CLOUD_CNT_STS = false;

bool MQTT_CLIENT_01_CNT_STS = false,
     MQTT_CLIENT_02_CNT_STS = false,
     MQTT_CLIENT_COORD_CNT_STS = false,
     MQTT_CLIENT_COORD_RESP_CNT_STS = false;

#define LED_PIN 13

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;

static QueueHandle_t CoordIntf_evtQHndl;
static QueueHandle_t Queue2;
static QueueHandle_t Queue3;
static QueueHandle_t Queue4;
static QueueHandle_t Queue5;
static QueueHandle_t Queue6;

int hdrAcked = 0, hdrAckSts = 0;

void MQTT_callback_1(char* topic, byte* payload, unsigned int len)
{
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
  if (String(topic) == "")
  {
  }
}
void MQTT_callback_2(char* topic, byte* payload, unsigned int len)
{
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  if (String(topic) == "")
  {
  }
}
int mqttMsgPyldLen = 0;
void MQTT_callback_coord(char* topic, byte* payload, unsigned int len)
{
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
  byte* mqttMsgSub;

  mqttMsgSub = (byte*)malloc(sizeof(byte) * len);
  if (mqttMsgSub == NULL)
    return;
  memcpy(mqttMsgSub, payload, len);
  Serial.printf("\nThe mqttMsgSub:");
  Serial.write(mqttMsgSub, len);
  Serial.printf("\n");
  mqttMsgPyldLen = len;
  if (String(topic) == get_attr)
  {
    if (xQueueSend(Queue4, (void*)mqttMsgSub, (TickType_t)10) == pdPASS)
    {
      Serial.printf("\n MQTT msg passed into Queue4...\n");
    }
    else
    {
      Serial.printf("\nNo queue space in Queue4...\n");
    }
  }
  free(mqttMsgSub);
  mqttMsgSub = NULL;
}

void MQTT_callback_coordResp(char* topic, byte* payload, unsigned int len)
{
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
  if (String(topic) == "")
  {
  }
}

boolean MODEM_restarted = false;

#define COORD_INTF_UART_DRIVER_BUFF_SIZE (1024 * 2)
#define COORD_INTF_EVT_QUEUE_SIZE  2

#define COORD_INTF_TASK_RX_MSG_BUFF_LEN  256
#define COORD_INTF_TASK_RX_RAW_BUFF_LEN  256

uint8_t COORD_INTF_taskRxRawBuff[COORD_INTF_TASK_RX_RAW_BUFF_LEN + 16];
uint8_t COORD_INTF_taskRxMsgBuff[COORD_INTF_TASK_RX_MSG_BUFF_LEN + 16];

struct COORD_DATA_RCVD_PCKT
{
  int pyldLenCnt_que = 0;
  uint8_t COORD_INTF_taskRxRawBuff_que[COORD_INTF_TASK_RX_RAW_BUFF_LEN + 16];
} COORD_DATA_RCVD_PCKT_01;

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

uint16_t UTIL_ntohs(const uint8_t *buff_p)
{
  uint16_t  u16Val = *buff_p;
  u16Val = (u16Val << 8) | buff_p[1];
  return u16Val;
}

uint32_t UTIL_ntohl(uint8_t *buff_p)
{
  uint32_t u32Val = *buff_p;
  u32Val <<= 8;
  u32Val |= buff_p[1];
  u32Val <<= 8;
  u32Val |= buff_p[2];
  u32Val <<= 8;
  u32Val |= buff_p[3];
  return u32Val;
}
uint16_t COORD_INTF_calcCkSum16(uint8_t *buff_p, uint8_t len)
{
  uint32_t ckSum = 0;

  while (len > 1)
  {
    uint16_t tmp = *buff_p;
    tmp = (tmp << 8) | (*(buff_p + 1));
    ckSum = ckSum + tmp;
    buff_p += 2;
    len -= 2;
  }

  if (len > 0)
    ckSum += (*buff_p);

  while (ckSum >> 16)
  {
    ckSum = (ckSum & 0xffff) + (ckSum >> 16);
  }

  return (~ckSum);
}

uint8_t TLV_get(uint8_t *buff_p, uint8_t len, uint8_t type,
                uint8_t *pyldLen_p, uint8_t **pyldBuff_pp)
{
  int buffLen = len;
  uint8_t rc = 0;

  if (buffLen < DIS_TLV_HDR_SZ)
    return 0;

  while (buffLen >= DIS_TLV_HDR_SZ)
  {
    uint8_t tlvPyldLen = *(buff_p + DIS_TLV_TYPE_FIELD_LEN);

    if (*buff_p == type)
    {
      *pyldLen_p = tlvPyldLen;
      *pyldBuff_pp = (buff_p + DIS_TLV_HDR_SZ);
      rc = 1;
      break;
    }
    else
    {
      buff_p += (DIS_TLV_HDR_SZ + tlvPyldLen);
      buffLen -= (DIS_TLV_HDR_SZ + tlvPyldLen);
    }
  }
  return rc;
}

float __latestVcc;
int __latestVccSet = 0;

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

char GW_jsonMsg[1024];
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

void GW_appendToJsonBuff()
{
  int jsonMsgLen;
  char* jsonMsgPtr_p;
  Serial.printf("\nPacket Received... \n");
  seq_nr++;
  GW_snsrDataBuff[0] = '\0';
  GW_snsrDataBuff_0[0] = '\0';
  GW_snsrDataBuff_1[0] = '\0';
  GW_snsrDataBuff_2[0] = '\0';
  GW_snsrDataBuff_3[0] = '\0';

  sprintf(GW_snsrDataBuff_0, "%.2f", snsrDataBuff[0]);
  sprintf(GW_snsrDataBuff_1, "%.2f", snsrDataBuff[1]);
  sprintf(GW_snsrDataBuff_2, "%.2f", snsrDataBuff[2]);
  sprintf(GW_snsrDataBuff_3, "%.2f", snsrDataBuff[3]);

  sprintf(snsrIdStore3, "0x%x", snsrIdStore[3]);
  sprintf(snsrIdStore2, "0x%x", snsrIdStore[2]);
  sprintf(snsrIdStore1, "0x%x", snsrIdStore[1]);
  sprintf(snsrIdStore0, "0x%x", snsrIdStore[0]);
  //root.clear();
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
  if (xQueueSend(Queue3, (void*)jsonMsgPtr_p, (TickType_t)5) == pdPASS)
  {
    Serial.printf("\nData Enqueued in Q3- success\n");
  }
  else
  {
    Serial.printf("\n No Queue space in Queue3...\n");
  }

  free(jsonMsgPtr_p);
  jsonMsgPtr_p = NULL;

  for(int i=0;i<4;i++)
  {
    snsrDataBuff[i] = '\0';
    snsrIdStore[i] = '\0';
  } 
  
  GW_snsrDataBuff[0] = '\0';
  GW_snsrDataBuff_0[0] = '\0';
  GW_snsrDataBuff_1[0] = '\0';
  GW_snsrDataBuff_2[0] = '\0';
  GW_snsrDataBuff_3[0] = '\0';
  merge_count = 0;
}

unsigned short GW_ntohs(unsigned char *buff_p)
{
  short u16Val = *buff_p;
  u16Val = (u16Val << 8) | buff_p[1];
  return u16Val;
}

void CoordIntf_procCoordMsg(void *params_p)
{
  uint8_t rec_data_q2[272];
  struct COORD_DATA_RCVD_PCKT COORD_DATA_RCVD_PCKT_02;
  while (1)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    DBG("Task 2 running...");
    if (xQueueReceive(Queue2, (void *)&COORD_DATA_RCVD_PCKT_02, (portTickType)portMAX_DELAY))
    {
      DBG("Queue2 received...");
      COORD_IF_procNodeMsg(COORD_DATA_RCVD_PCKT_02.COORD_INTF_taskRxRawBuff_que, COORD_DATA_RCVD_PCKT_02.pyldLenCnt_que);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
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
  sprintf(Node_macIdBuff, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", buff_p[off], buff_p[off + 1], buff_p[off + 2], buff_p[off + 3], buff_p[off + 4], buff_p[off + 5], buff_p[off + 6], buff_p[off + 7]);
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
                snsrOp = (int)(*buff3_p);
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
                snsrOp = (int)UTIL_ntohl(buff3_p);
                break;

              default:
                break;
            }
            switch (snsrId)
            {
              case PLTFRM_DUMMY_DEV_ID:
                // Serial.printf("+[Sequence Nr]    ");
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
                // Serial.printf("+[Node_Voltage] ");
                unit_p = "Volts";
                break;

              case PLTFRM_LM75B_1_DEV_ID:
                // Serial.printf("+[Temp_LM75B]   ");
                unit_p = "Deg C";
                break;

              case PLTFRM_TEPT5700_1_DEV_ID:
                // Serial.printf("+[Light_TEPT5700]   ");
                unit_p = "uA";
                scaleFactor = DIS_DATA_SCALE_NONE;
                break;
              case PLTFRM_DUAL_FS_LVL_MON_1_DEV_ID:
                // Serial.printf("+[Dual_Float_Switch_Tank_Mon]  ");
                unit_p = "";
                break;

              case PLTFRM_ON_CHIP_TEMP_SENSOR_DEV_ID:
                // Serial.printf("+[Temp_MSP430]  ");
                unit_p = "deg C";
                break;

              case PLTFRM_BATT_1_DEV_ID:
                // Serial.printf("+[Batt Voltage] ");
                scaleFactor = DIS_DATA_SCALE_MILLI;
                unit_p = "Volts";
                break;

              case PLTFRM_TMP75C_1_DEV_ID:
                unit_p = "Deg C";
                scaleFactor = DIS_DATA_SCALE_CENTI;
                break;

              case PLTFRM_HIH8121_1_RH_DEV_ID:
                //printf("+[RH_HIH8121]   ");
                unit_p = "%";
                scaleFactor = DIS_DATA_SCALE_CENTI;
                break;

              case PLTFRM_HIH8121_1_TEMP_DEV_ID:
                //printf("+[Temp_HIH8121] ");
                unit_p = "Deg C";
                scaleFactor = DIS_DATA_SCALE_CENTI;
                break;

              case PLTFRM_RTD_1000_1_DEV_ID:
                // printf("+[Temp_RTD_1000] ");
                unit_p = "deg C";
                scaleFactor = DIS_DATA_SCALE_NONE;
                break;
              default:
                // Serial.printf("[Unknown]");
                break;
            }
            
            if (snsrId == PLTFRM_GEN_VOLT_MON_DEV_ID
                || snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID)
              scaleFactor = DIS_DATA_SCALE_MILLI;

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
                  && snsrId != PLTFRM_MP3V5004GP_1_DEV_ID
                  // && snsrId != PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID
                 )
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

void COORD_INTF_procRcvdFrame(void)
{
  uint16_t msgType = UTIL_ntohs(COORD_INTF_taskRxMsgBuff);

  Serial.printf("C_I_pRF> T:%d PL:%d\n", (int)msgType, COORD_INTF_rcvdFramePyldLen);

  switch (msgType)
  {
    case LPWMN_GW_MSG_TYPE_RELAY_FROM_NODE:
      {
        COORD_DATA_RCVD_PCKT_01.pyldLenCnt_que = COORD_INTF_rcvdFramePyldLen;
        int i_temp;
        for (int i = 0; i < COORD_INTF_rcvdFramePyldLen; i++)
        {
          i_temp = i + COORD_INTF_FRAME_HDR_LEN;
          COORD_DATA_RCVD_PCKT_01.COORD_INTF_taskRxRawBuff_que[i] = COORD_INTF_taskRxMsgBuff[i_temp];
        }
        if (xQueueSend(Queue2, (void*)&COORD_DATA_RCVD_PCKT_01, (TickType_t)5) == pdPASS)
        {
          Serial.printf("\nData Enqueued in Q2- success\n");
        }
        else
        {
          Serial.printf("\n No Queue space in Queue2...\n");
        }
      }
      break;

      case LPWMN_GW_MSG_TYPE_START_COORD:
           {
              if (COORD_INTF_rcvdFramePyldLen == 0x1)
              {
                  if (COORD_INTF_taskRxMsgBuff[UART_FRAME_HDR_LEN] == 0x1)
                      Serial.printf("done \n");
                  else
                      Serial.printf("request denied !!\n");
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
                  if(xQueueSend(Queue5, (void*)&hdrAcked, (TickType_t)0) == pdPASS)
                  {
                    Serial.printf("\nHDR Acked response passed to Queue5..\n");
                  }
                  else
                  {
                    Serial.printf("\nNo queue space in Queue5...\n");
                  }              
            }
            else
              {
                  Serial.printf("<%s> hdr flag <0x%02x> hdrAcked <0> !! \n", __FUNCTION__, hdrFlags);
                  hdrAcked = 0;
              }
           }
           break;
      
    case LPWMN_GW_MSG_TYPE_GET_RADIO_BAUD_RATE:
      {
          int br=0;
          Serial.printf("\nRADIO_BAUD_RATE\n");
          if(COORD_INTF_rcvdFramePyldLen == LPWMN_GW_MSG_RADIO_BAUD_RATE_FIELD_LEN)
          { 
             br = UTIL_ntohl(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);
             Serial.printf("%d baud \n", br);
          }
          char brStr[10];
          sprintf(brStr,"%d",br);
          if(xQueueSend(Queue6, (void*)&brStr, (TickType_t)0) == pdPASS)
          {
            Serial.printf("\nCoord Responce(grbr) passed into Queue6..\n");
          }
          else
          {
            Serial.printf("\nNo queue space in Queue6...\n");
          }         
         
      }
    break;
    
    case LPWMN_GW_MSG_TYPE_GET_NWK_CHANNEL:
      {
        unsigned int chann=0; 
        Serial.printf("\nRADIO Channel Ack: \n");
        if (COORD_INTF_rcvdFramePyldLen == LPWMN_GW_MSG_RADIO_CHANN_FIELD_LEN)
        {
          chann = UTIL_ntohs(COORD_INTF_taskRxMsgBuff + UART_FRAME_HDR_LEN);
          Serial.printf("Radio Channel - %d\n", chann);
        }
        char channStr[10];
        sprintf(channStr,"%d",chann);
        if(xQueueSend(Queue6, (void*)&channStr, (TickType_t)0) == pdPASS)
        {
          Serial.printf("\nCoord Responce(grch) passed into Queue6...\n");
        }
          else
          {
            Serial.printf("\nNo queue space in Queue6...\n");
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

int COORD_IF_procOneMsg(uint8_t *rawBuff_p, const int rxByteCnt)
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
      memcpy(COORD_INTF_taskRxMsgBuff + COORD_INTF_rxMsgOff,
             rawBuff_p, toCopy);
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

void COORD_IF_procRxBytes(int rxByteCnt)
{
  int totConsumedCnt = 0;
  Serial.printf("PRB rBC:%d\n", rxByteCnt);
  while (rxByteCnt > 0)
  {
    Serial.printf("PRB L1 rBC:%d tCC:%d \n", rxByteCnt, totConsumedCnt);
    int procCnt = COORD_IF_procOneMsg(COORD_INTF_taskRxRawBuff + totConsumedCnt, rxByteCnt);
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

static void CoordIntf_uartEvtTask(void *params_p)
{
  uart_event_t event;

  while (1)
  {
    if (xQueueReceive(CoordIntf_evtQHndl, (void * )&event, (portTickType)portMAX_DELAY))            
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

              toReadCnt = rcvdByteCnt > COORD_INTF_TASK_RX_RAW_BUFF_LEN ? \
                          rcvdByteCnt : COORD_INTF_TASK_RX_RAW_BUFF_LEN;
              readCnt = uart_read_bytes(COORD_INTF_SERIAL_PORT_NR,
                                        COORD_INTF_taskRxRawBuff,
                                        toReadCnt, 100);
              COORD_IF_procRxBytes(readCnt);

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

#define SER_BUFF_LEN 256
unsigned char serTxBuff[SER_BUFF_LEN];
unsigned char serRxBuff[SER_BUFF_LEN];

int writePort(unsigned char *buff_p, unsigned int cnt)
{
  int rc;
  Serial.printf("\nCnt: <%d> \n",cnt);
  Serial.printf("Buff data: ");
  for(int i=0;i<cnt;i++)
  {
    Serial.printf("0x%02x |",buff_p[i]);
  }
  rc = uart_write_bytes(COORD_INTF_SERIAL_PORT_NR, (const char*) buff_p, cnt );
  Serial.printf("\nUART RC: <%d> !! \n",rc);
  if(rc != cnt)
  {
    return -1;
  }
  return 1;
}

void GW_htons(unsigned char *buff_p, unsigned short val)
{
  buff_p[0] = (val >> 8) & 0xff;
  buff_p[1] = (val) & 0xff;
}

void GW_htonl(unsigned char *buff_p, unsigned int val)
{
   buff_p[0] = (val >> 24) & 0xff;
   buff_p[1] = (val >> 16) & 0xff;
   buff_p[2] = (val >> 8) & 0xff;
   buff_p[3] = (val) & 0xff;
}

int GW_buildSendHdr(int msgType, unsigned char *pyldBuff_p, int pyldLen)
{
  unsigned char *buff_p = serTxBuff;
  unsigned short calcCrc16;
  static unsigned char seqNr = 0x0;
  int rc;

  Serial.printf("\nmsgType <0x%x>", msgType);

  GW_htons(buff_p, msgType);
  buff_p += UART_FRAME_HDR_MSG_TYPE_FIELD_LEN;

  *buff_p = 0x0;
  buff_p += UART_FRAME_HDR_FLAGS_FIELD_LEN;

 *buff_p = seqNr ++;
  buff_p += UART_FRAME_HDR_SEQ_NR_FIELD_LEN;

  GW_htons(buff_p, pyldLen);
  buff_p += UART_FRAME_HDR_PYLD_LEN_FIELD_LEN;

  calcCrc16 = COORD_INTF_calcCkSum16(serTxBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
  GW_htons(buff_p, calcCrc16); 
  buff_p += UART_FRAME_HDR_HDR_CRC_FIELD_LEN;

  if (pyldLen > 0)
  {
    calcCrc16 = COORD_INTF_calcCkSum16(pyldBuff_p, pyldLen);
    GW_htons(buff_p, calcCrc16); 
  }
  else
    GW_htons(buff_p, 0x0);

  rc = writePort(serTxBuff, UART_FRAME_HDR_LEN);
    if (rc != 1)
    {
       Serial.printf("\nwritePort() failed !!\n");
       rc = 20;
   }
   return rc;
}

int GW_startCoordReq(void)
{
   int rc;
   rc = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_START_COORD, NULL, 0x0);
   if (rc != 1)
   {
       Serial.printf("failed !! \n");
       return 5;
   }
   return rc;
}

int GW_getRadioBaudRate(void)
{
   int rc = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_GET_RADIO_BAUD_RATE, NULL, 0x0);
   if (rc != 1)
       return 5;
       
   return rc;
}

int GW_getRadioChannReqHndlr( )
{
   int rc = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_GET_NWK_CHANNEL, NULL, 0x0);
   if (rc != 1)
       return 5;
   else
      Serial.printf("\nRequest Sent...");    
   return rc;
}

int GW_setNodeAttrVal(unsigned int shortAddr,
                      unsigned int attrId,
                      int attrVal,
                      const char *strAttrVal_p)
{
   int rc = 1, pyldLen, off = 0, attrValLen = 0;
   unsigned char *pyld_p;

   pyldLen = LPWMN_MAC_SHORT_ADDR_LEN
             + DIS_MSG_TYPE_SZ
             + DIS_TLV_HDR_SZ
             + DIS_ATTR_ID_TLV_SZ
             + DIS_TLV_HDR_SZ;

   switch (attrId)
   {
      case APP_LOAD_CELL_RATED_OUTPUT_LOAD_IN_KGS_ATTR_ID:
      case APP_LOAD_CELL_NOMINAL_LOAD_IN_KGS_ATTR_ID:
      case APP_LOAD_CELL_ZERO_BALANCE_FIGURE_ATTR_ID:
      case APP_LOAD_CELL_SENSITIVITY_IN_UV_PER_V_ATTR_ID:
           attrValLen = 2;
           break;

      case APP_LOAD_CELL_SERIAL_NR_ATTR_ID:
           attrValLen = 8;
           break;

      case AD7797_SYSTEM_ZERO_SCALE_CAL_REQ_ATTR_ID:
           attrValLen = 1;
           break;

      case APP_SENSOR_RPT_ENA_BIT_MSK_ATTR_ID:
           attrValLen = 2;
           break;

      case PLTFRM_TEST_WD_RESET_ATTR_ID:
           attrValLen = 1;
           break;

      case RADIO_AFC_FUNCIONALITY_ATTR_ID:
           attrValLen = 1;
           break;

      case 5000:
           attrValLen = 1;
           break;

      case NM_INTER_SCAN_INTERVAL_SECS_ATTR_ID:
           attrValLen = 2;
           break;

      case PHY_TX_POWER_ATTR_ID:
           attrValLen = 2;
           break;

      case NM_MAX_SCAN_FLR_CNT_TO_RESET_ATTR_ID:
           attrValLen = 1;
           break;

      case CC1120_AGC_CS_THR_ATTR_ID:
           attrValLen = 1;
           break;

      case RADIO_FREQ_OFFSET_ATTR_ID:
           // -18000 HZ to +18000 HZ
           attrValLen = 2;
           break;

      case RADIO_RF_TX_TEST_CHANN_ID_ATTR_ID:
           attrValLen = 1;
           break;

      case RADIO_RF_TX_TEST_PA_CFG_ATTR_ID:
           // 0 to 255
           attrValLen = 1;
           break;

      case RADIO_START_CW_UNMOD_TX_TEST_ATTR_ID:
           // 1 to 65535
           attrValLen = 2;
           break;

      case RADIO_START_CW_MOD_TX_TEST_ATTR_ID:
           // 1 to 65535
           attrValLen = 2;
           break;

      case APP_SNSR_DATA_REPORT_MODE_ATTR_ID:
           attrValLen = 1;
           break;

      case WSMS100_SQ_WAVE_FREQUENCY_CFG_ATTR_ID:
           attrValLen = 4;
           break;

      case RADIO_TX_TO_RX_TURN_AROUND_DELAY_ATTR_ID:
           attrValLen = 1;
           break;

      case APP_MAX_SNSR_DATA_REPORT_INTERVAL_SECS_ATTR_ID:
           attrValLen = 4;
           break;

      case APP_SNSR_DATA_REPORT_DELTA_THRESHOLD_ATTR_ID:
           attrValLen = 4;
           break;

      case DEV_PULSE_CNTR_RESET_CNTR:
           attrValLen = 4;
           break;

      case APP_MAX_SENSOR_DATA_TX_INTERVAL_ATTR_ID:
           attrValLen = 4;
           break;

      case APP_PIR_SENSOR_ENA_DIS_CTRL_ATTR_ID:
           attrValLen = 1;
           break;

      case APP_VEH_DET_MFS_HPF_DETECTION_ALPHA:
           attrValLen = 1;
           break;

      case APP_VEH_DET_MFS_HPF_DETECTION_THRESHOLD:
           attrValLen = 2;
           break;

      case APP_VEH_DET_MFS_HPF_SETTLING_THRESHOLD:
           attrValLen = 2;
           break;

      case MAC_ACK_TMO_DELTA_ATTR_ID:
           attrValLen = 2;
           break;

      case MAC_TX_ON_CCA_ENA_FLAG_ATTR_ID:
           attrValLen = 1;
           break;

      case MAC_LPWMN_ID_ATTR_ID:
           attrValLen = 2;
           break;

      case MESH_TRIGGER_PATH_DISC_ATTR_ID:
           attrValLen = 2;
           break;

      case FFD_LAST_APP_TX_TO_RBT_INTERVAL_MULTIPLE_ATTR_ID:
           attrValLen = 1;
           break;

      case FFD_LAST_APP_TX_TO_RBT_INTERVAL_SECS_ATTR_ID:
           attrValLen = 4;
           break;

      case APP_MAG3110_X_AXIS_BASE_VAL_MFS_ATTR_ID:
      case APP_MAG3110_Y_AXIS_BASE_VAL_MFS_ATTR_ID:
      case APP_MAG3110_Z_AXIS_BASE_VAL_MFS_ATTR_ID:
           attrValLen = 2;
           break;

      case PHY_RAW_BAUD_RATE_ATTR_ID:
           attrValLen = 4;
           break;

      default:
           rc = 0;
           Serial.printf("attribute id <%u> not supported !! \n", attrId);
           break;
   }

   if (rc == 0)
       return rc;

   pyldLen += attrValLen;

   pyld_p = (unsigned char *)malloc(pyldLen);
   if (pyld_p == NULL)
   {
       printf("malloc(%d) failed !! \n", pyldLen);
       return 1;
   }

   GW_htons(pyld_p, shortAddr);
   off = LPWMN_MAC_SHORT_ADDR_LEN;
   pyld_p[off ++] = DIS_MSG_TYPE_SET_ATTR_VAL;
   pyld_p[off ++] = DIS_TLV_TYPE_ATTR_INFO;
   pyld_p[off ++] = DIS_ATTR_ID_TLV_SZ + DIS_TLV_HDR_SZ + attrValLen;
   pyld_p[off ++] = DIS_TLV_TYPE_ATTR_ID;
   pyld_p[off ++] = DIS_ATTR_ID_FIELD_SZ;
   GW_htons(pyld_p + off, attrId);
   off += DIS_ATTR_ID_FIELD_SZ;
   pyld_p[off ++] = DIS_TLV_TYPE_ATTR_VAL;
   pyld_p[off ++] = attrValLen;
   switch (attrValLen)
   {
      case 1:
          pyld_p[off] = attrVal;
          break;

      case 2:
          GW_htons(pyld_p + off, attrVal);
          break;

      case 4:
          GW_htonl(pyld_p + off, attrVal);
          break;

      default:
          if (attrId == APP_LOAD_CELL_SERIAL_NR_ATTR_ID)
          {
              memset(pyld_p + off, 0x20, 8);
              memcpy(pyld_p + off, strAttrVal_p, strlen(strAttrVal_p));
          }
          break;
   }
   off += attrValLen;

   rc = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_RELAY_TO_NODE,
                        pyld_p, pyldLen);
   if (rc != 1)
       return rc;
   if(rc == 1)
    Serial.printf("Gw_buildSendHdr function successfully written waiting for coord Response ACK !! \n");

   Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);   
   if (xQueueReceive(Queue5, (void *)&hdrAckSts, (portTickType)10))  
    {
      if(hdrAckSts == 1)
      {
        Serial.printf("Hdr acked...<%s> !! \n",__FUNCTION__);
        for(int i_a=0;i_a<pyldLen;i_a++)
        {
          Serial.printf("%x ",pyld_p[i_a]);
        }
        rc = writePort(pyld_p, pyldLen);
        if (rc != 1)
        {
          Serial.printf("<%s> writePort(%d) failed !! \n",
                                                  __FUNCTION__, pyldLen);
          return 3;
        }
        else
        {
          Serial.printf("Payload Request Send <%s> !! \n",__FUNCTION__);
        }   
      }
      if(hdrAckSts == 0)
      {
        Serial.printf("Header not acked <%s> !! \n",__FUNCTION__);
      }
        hdrAckSts = 0;
   }
 
   
   return rc;
   /* 
   rc = GW_readSerIntf(UART_MSG_TYPE_ACK, 0);
   if (rc != 0)
       return rc;

   if (hdrAcked == 0x0)
   {
       printf("Header not acked !! \n");
       return 2;
   }

   if (verbose)
       printf("Header acked \n");

   if (verbose)
   {
       int idx;

       printf("\n -------------------------- \n");

       for (idx=0; idx<pyldLen; idx++)
            printf(" 0x%02x ", pyld_p[idx]);

       printf("\n -------------------------- \n");
   }

   // Send payload
   rc = writePort(pyld_p, pyldLen);
   if (rc != 1)
   {
       printf("<%s> writePort(%d) failed !! \n",
              __FUNCTION__, pyldLen);
       return 3;
   }

   printf("Request sent...... \n");
   */

   // printf("Request sent. Waiting for response ..... \n");
   // rc = GW_readSerIntf(LPWMN_GW_MSG_TYPE_SET_COORD_ATTR_VAL, 0);


}

int GW_setCoordAttrVal(unsigned int shortAddr,
                       unsigned int attrId,
                       int attrVal)
{
   int rc = 1, pyldLen, off = 0;
   unsigned char *pyld_p;

   Serial.printf("\n attr-id<%u> / attr-val<%u> \n", attrId, attrVal);

   pyldLen = DIS_ATTR_ID_FIELD_SZ;

   switch (attrId)
   {
      case PLTFRM_TEST_WD_RESET_ATTR_ID:
           pyldLen += 1;
           break;

      case CC1120_AGC_CS_THR_ATTR_ID:
           pyldLen += 1;
           break;

      case RADIO_AFC_FUNCIONALITY_ATTR_ID:
           pyldLen += 1;
           break;

      case RADIO_FREQ_OFFSET_ATTR_ID:
           // -18000 HZ to +18000 HZ
           pyldLen += 2;
           break;

      case PHY_RAW_BAUD_RATE_ATTR_ID:
           pyldLen += 4;
           break;

      case RADIO_RF_TX_TEST_CHANN_ID_ATTR_ID:
           pyldLen += 1;
           break;

      case RADIO_RF_TX_TEST_PA_CFG_ATTR_ID:
           // 0 to 255
           pyldLen += 1;
           break;

      case RADIO_START_CW_UNMOD_TX_TEST_ATTR_ID:
           // 1 to 65535
           pyldLen += 2;
           break;

      case RADIO_START_CW_MOD_TX_TEST_ATTR_ID:
           // 1 to 65535
           pyldLen += 2;
           break;

      case MAC_NWK_TYPE_ATTR_ID:
           pyldLen += 1;
           break;

      case RADIO_TX_TO_RX_TURN_AROUND_DELAY_ATTR_ID:
           pyldLen += 1;
           break;

      case MAC_ACK_TMO_DELTA_ATTR_ID:
           pyldLen += 2;
           break;

      case MAC_LPWMN_ID_ATTR_ID:
           pyldLen += 2;
           break;

      case PHY_RF_CHANN_ID_ATTR_ID:
           pyldLen += 1;
           break;

      case PHY_TX_POWER_ATTR_ID:
           pyldLen += 2;
           break;

      case MESH_TRIGGER_PATH_DISC_ATTR_ID:
           pyldLen += 2;
           break;

      case PLTFRM_SET_GPIO_OUTPUT_ATTR_ID:
      case PLTFRM_CLEAR_GPIO_OUTPUT_ATTR_ID:
           pyldLen += 2;
           break;

      default:
           rc = 0;
           Serial.printf("attribute id <%u> not supported !! \n", attrId);
           break;
   }

   if (rc == 0)
       return rc;

   pyld_p = (unsigned char *)malloc(pyldLen);
   if (pyld_p == NULL)
   {
       Serial.printf("malloc(%d) failed !! \n", pyldLen);
       return 1;
   }
   GW_htons(pyld_p, attrId);


   switch (attrId)
   {
      case MAC_ACK_TMO_DELTA_ATTR_ID:
      case MAC_LPWMN_ID_ATTR_ID:
      case MESH_TRIGGER_PATH_DISC_ATTR_ID:
           // -18000 HZ to +18000 HZ
      case RADIO_FREQ_OFFSET_ATTR_ID:
      case RADIO_START_CW_UNMOD_TX_TEST_ATTR_ID:
      case RADIO_START_CW_MOD_TX_TEST_ATTR_ID:
      case RADIO_RF_TX_TEST_PA_CFG_ATTR_ID:
      case PLTFRM_SET_GPIO_OUTPUT_ATTR_ID:
      case PLTFRM_CLEAR_GPIO_OUTPUT_ATTR_ID:
      case PHY_TX_POWER_ATTR_ID:
           GW_htons(pyld_p + DIS_ATTR_ID_FIELD_SZ, attrVal);
           break;

      case PHY_RAW_BAUD_RATE_ATTR_ID:
           GW_htonl(pyld_p + DIS_ATTR_ID_FIELD_SZ, attrVal);
           break;

      case PHY_RF_CHANN_ID_ATTR_ID:
      case RADIO_TX_TO_RX_TURN_AROUND_DELAY_ATTR_ID:
      case MAC_NWK_TYPE_ATTR_ID:
      case RADIO_RF_TX_TEST_CHANN_ID_ATTR_ID:
      case PLTFRM_TEST_WD_RESET_ATTR_ID:
      case CC1120_AGC_CS_THR_ATTR_ID:
      case RADIO_AFC_FUNCIONALITY_ATTR_ID:
           *(pyld_p + DIS_ATTR_ID_FIELD_SZ) = attrVal;
           break;

      default:
           break;
   }
   rc = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_SET_COORD_ATTR_VAL,
                        pyld_p, pyldLen);
   if (rc != 1)
       return rc;
   if(rc == 1)
    Serial.printf("Gw_buildSendHdr function successfully written waiting for coord Response ACK !! \n");

   Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);  
   if (xQueueReceive(Queue5, (void *)&hdrAckSts, (portTickType)10))  
    {
      if(hdrAckSts == 1)
      {
        Serial.printf("Hdr acked <%s> !! \n", __FUNCTION__);
        for(int i_a=0;i_a<pyldLen;i_a++)
        {
          Serial.printf("%x ",pyld_p[i_a]);
        }
        rc = writePort(pyld_p, pyldLen);
        if (rc != 1)
        {
          Serial.printf("<%s> writePort(%d) failed !! \n",
                                                  __FUNCTION__, pyldLen);
          return 3;
        }
        else
        {
          Serial.printf("Payload Request Send <%s> !! \n", __FUNCTION__);
        }   
      }
      if(hdrAckSts == 0)
      {
        Serial.printf("Header not acked !! \n");
      }
        hdrAckSts = 0;
   }
  
     return rc;
}

int GW_rebootCoordReq(void)
{
   int rc;

   rc = GW_buildSendHdr(LPWMN_GW_MSG_TYPE_REBOOT_COORD, NULL, 0x0);
   if (rc != 1)
   {
       Serial.printf("failed !! \n");
       rc = 5;
   }
   else
       Serial.printf("Request sent ... ");

   return rc;
}

void CoordIntf_txEvtTask(void *params_p)
{
  StaticJsonDocument<512> jsonDoc_coord;
  byte rcvdMqttMsg[512];
  char rcvdCoordAttrCpy[50];
  DBG("\n CoordIntf_txEvtTask Running...\n");
  while (1)
  {
     vTaskDelay(100 / portTICK_PERIOD_MS);
    if (xQueueReceive(Queue4, (void *)&rcvdMqttMsg, (portTickType)10))
    {
      int i = 0, i_tmp = 0;
      char rcvdMqttMsgCpy[512];
      for (i = 0; i < mqttMsgPyldLen; i++)
      {
        rcvdMqttMsgCpy[i] = (char)rcvdMqttMsg[i];
      }
      rcvdMqttMsg[i] = '\0';
      rcvdMqttMsgCpy[i] = '\0';
      Serial.printf("\nReceived MqttMsg in task-4: <%s>, len <%d>", rcvdMqttMsg, mqttMsgPyldLen);
      DeserializationError error = deserializeJson(jsonDoc_coord, rcvdMqttMsgCpy);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      const char* rcvdCoordAttr = jsonDoc_coord["test"];
      Serial.printf("\nCoord Attribute: ");
      Serial.printf(rcvdCoordAttr);
      Serial.printf("\n");
      rcvdMqttMsg[0] = '\0';

                i=0;
                int len = strlen(rcvdCoordAttr);
                int cnt=0, cnxtSpcCntStr[10];
                char attrName[10];
                unsigned int shortAddr, 
                             attrId;
                int attrVal;
                char shortAddrCpy[10], attrIdCpy[10], attrValCpy[10];
                
                for(i=0; i<strlen(rcvdCoordAttr); i++)
                {
                    if(rcvdCoordAttr[i] == ' ')
                    {
                        Serial.printf("found space indx <%d> \n",i+1);
                        cnxtSpcCntStr[cnt] = i+1;
                        ++cnt;
                    }
                }
                printf("Total Entered Cnxt <%d>\n",cnt); 
                if(cnt > 0)
                {
                    Serial.printf("cnxtSpcCntStr value fields:");
                    for(int i=0;i<cnt;i++)
                    {
                        printf("%d ",cnxtSpcCntStr[i]);
                    }
                    printf("\n");
                }
                if(cnt == 0)
                {
                    Serial.printf("Entered attributes Cntx length - 1 !! \n");
                    for(i=0; i<strlen(rcvdCoordAttr); i++)
                    {
                        attrName[i] = rcvdCoordAttr[i];
                    }
                    attrName[i] = '\0';
                }
                if(cnt == 1)
                {
                    Serial.printf("Entered attributes Cnxt length - 2 !! \n");
                    for(i=0; i<cnxtSpcCntStr[0]-1; i++)
                    {
                        attrName[i] = rcvdCoordAttr[i];
                    }
                    attrName[i] = '\0';                
                    int i_temp = 0;
                    for(i=cnxtSpcCntStr[0]; i<len; i++)
                    {
                        
                        shortAddrCpy[i_temp] = rcvdCoordAttr[i];
                        i_temp++;
                    }
                    shortAddrCpy[i_temp] = '\0';
                }
                if(cnt == 2)
                {
                    Serial.printf("Entered attributes Cnxt length - 3 !! \n");
                    for(i=0; i<cnxtSpcCntStr[0]-1; i++)
                    {
                        attrName[i] = rcvdCoordAttr[i];
                    }
                    attrName[i] = '\0';
                    
                    int i_temp = 0;
                    for(i=cnxtSpcCntStr[0]; i<cnxtSpcCntStr[1]; i++)
                    {
                        shortAddrCpy[i_temp] = rcvdCoordAttr[i];
                        i_temp++;
                    }
                    shortAddrCpy[i_temp] = '\0';
                    
                    i_temp = 0;
                    for(i=cnxtSpcCntStr[1]; i<len; i++)
                    {
                        attrIdCpy[i_temp] = rcvdCoordAttr[i];
                        i_temp++;
                    }
                    attrIdCpy[i_temp] = '\0';
                }
                if(cnt == 3)
                {
                    printf("Entered attributes Cnxt length - 4 !! \n");
                    for(i=0; i<cnxtSpcCntStr[0]-1; i++)
                    {
                        attrName[i] = rcvdCoordAttr[i];
                    }
                    attrName[i] = '\0';

                    printf("");
                    
                    int i_temp = 0;
                    for(i=cnxtSpcCntStr[0]; i<cnxtSpcCntStr[1]; i++)
                    {
                        shortAddrCpy[i_temp] = rcvdCoordAttr[i];
                        i_temp++;
                    }
                    shortAddrCpy[i_temp] = '\0';
                    
                    i_temp = 0;
                    for(i=cnxtSpcCntStr[1]; i<cnxtSpcCntStr[2]; i++)
                    {
                        attrIdCpy[i_temp] = rcvdCoordAttr[i];
                        i_temp++;
                    }
                    attrIdCpy[i_temp] = '\0';
                    
                    i_temp = 0;
                    for(i=cnxtSpcCntStr[2]; i<len; i++)
                    {
                        attrValCpy[i_temp] = rcvdCoordAttr[i];
                        i_temp++;
                    }
                    attrValCpy[i_temp] = '\0';
                }
                Serial.printf("attrName <%s> !! \n",attrName);
                shortAddr = atoi(shortAddrCpy);
                Serial.printf("shortAddr <%d> !! \n", shortAddr);
                attrId = atoi(attrIdCpy);
                Serial.printf("attrId <%d> !! \n", attrId);
                attrVal = atoi(attrValCpy);
                Serial.printf("attrVal <%d> !! \n", attrVal);
      if(strcmp(attrName, "sav") == 0)
      {
        if( shortAddr < 1 || shortAddr > LPWMN_MAX_UNICAST_SHORT_ADDR)
        {
          Serial.printf("Enter a valid short address (>=1 && <= %u) !! \n", LPWMN_MAX_UNICAST_SHORT_ADDR);
        }
        else
        {
          if(attrId < 1 || attrId > 65535)
          {
            Serial.printf("Enter a valid attribute Id (>=1 & < =65535) !! \n");
          }
          else
          {
            char *strAttrVal_p = NULL;
            if(attrVal == NULL)
            {
              Serial.printf("Please enter a valid attribute value !! \n");
            }
            else
            {
              if(shortAddr == LPWMN_COORD_SHORT_ADDR)
              {
                int rc = GW_setCoordAttrVal(shortAddr, attrId, attrVal);
                if(rc != 1)
                {
                  Serial.printf("failed: %d \n",rc);
                }
                else
                {
                  Serial.printf("Request Send, Coord BaudRate Changed <%d> !! \n",rc);
                }
              }
              else
              {
                int rc = GW_setNodeAttrVal(shortAddr, attrId, attrVal, strAttrVal_p);
                if(rc != 1)
                {
                  Serial.printf("failed to [setNodeAttrVal] <%d> \n",rc);  
                }
                else
                {
                  Serial.printf("Request Send [SetNodeAttrVal] !! \n");
                }

              }
            }
            
          }
        }
      }
      shortAddr = attrId = attrVal = 0;
      shortAddrCpy[0] = attrIdCpy[0] = attrValCpy[0] = '\0';
      
      if (strcmp(rcvdCoordAttr, "sc") == 0)
      {
        Serial.printf("\nStart the coordinator !!\n");
        int rc = GW_startCoordReq( );     
        if(rc == 5)
          Serial.printf("\nGW_buildSendHdr Failed <%s> !!",__FUNCTION__);  
      }
      
      if (strcmp(rcvdCoordAttr, "rstc") == 0)
      {
        Serial.printf("Reset the Coordinator !!\n");
        int rc = GW_rebootCoordReq( );
        if(rc == 5)
          Serial.printf("\nGW_buildSendHdr Failed <%s>",__FUNCTION__);
      }

      if (strcmp(rcvdCoordAttr, "grch") == 0)
      {
        Serial.printf("Get Radio Channel !!\n");
        int rc = GW_getRadioChannReqHndlr( );
        if(rc == 5)
          Serial.printf("\nGW_builfSendHdr Failed <%s>\n",__FUNCTION__);
      }
      
      if (strcmp(rcvdCoordAttr, "grbr") == 0)
      {
        Serial.printf("Get Radio BaudRate !!\n");
        int rc = GW_getRadioBaudRate( );     
        if(rc == 5)
          Serial.printf("\nGW_buildSendHdr Failed <%s> !!",__FUNCTION__);  
      }      
    }
  }
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
                      &CoordIntf_evtQHndl, 
                      0);
  Queue2 = xQueueCreate(2, sizeof(COORD_DATA_RCVD_PCKT_01));
  Queue3 = xQueueCreate(2, sizeof(char) * 1024);
  Queue4 = xQueueCreate(2, sizeof(char) * 1024);
  Queue5 = xQueueCreate(2, sizeof(hdrAcked));
  Queue6 = xQueueCreate(2, sizeof(char) * 1024); 
  if (Queue2 == NULL)
  {
    Serial.printf("\nQueue2 not created...\n");
  }
  if (Queue3 == NULL)
  {
    Serial.printf("\nQueue2 not created...\n");
  }
  if (Queue4 == NULL)
  {
    Serial.printf("\nQueue4 not created...\n");
  }
  if (Queue5 == NULL)
  {
    Serial.printf("\nQueue5 not created...\n");
  }
  if(Queue6 == NULL)
  {
    Serial.printf("\nQueue6 not created..\n");
  }
  xTaskCreate(CoordIntf_uartEvtTask, "COORD_INTF_UART_ISR_ROUTINE", 20000, NULL, 12, NULL);

  xTaskCreate(CoordIntf_procCoordMsg, "COORD_INTF_PROC_COORD_MSG", 20000, NULL, 12, NULL);

  xTaskCreate(CoordIntf_txEvtTask, "COORD_INTF_TX_EVENT_TASK", 20000, NULL, 12, NULL);

  GSM_serialObj.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(10);

  MQTT_client_01.setServer(MQTT_brokerWsURL, 1883);
  MQTT_client_01.setCallback(MQTT_callback_1);
  MQTT_client_02.setServer(MQTT_brokerWsURL, 1883);
  MQTT_client_02.setCallback(MQTT_callback_2);
  MQTT_client_coord.setServer(MQTT_brokerURL, 1883);
  MQTT_client_coord.setCallback(MQTT_callback_coord);
  MQTT_client_coordResp.setServer(MQTT_brokerWsURL, 1883);
  MQTT_client_coordResp.setCallback(MQTT_callback_coordResp);

  DBG("Setup Complete :-)");
}

void loop()
{
  char rcvdNodePyld[1024];
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
      GSM_NETWRK_CNT_STS = false;
      return;
    }
    if (modem.isNetworkConnected())
    {
      DBG("Registered to the network  :-) ");
      GSM_NETWRK_CNT_STS = true;
    }
    else
    {
      DBG("Still not registered to the network ... delaying for 10 secs before retrying !!!");
      GSM_NETWRK_CNT_STS = false;
      delay(1 * 1000);
      return;
    }
  }
  else
  {
    GSM_NETWRK_CNT_STS == true;
  }
  
  if (modem.isGprsConnected() == false)
  {
    DBG("Connecting to", apn);

    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
      DBG("GPRS connection setup failed ... delaying for 10 secs before retrying !!! ");
      delay(1 * 1000);
      GSM_GPRS_CNT_STS = false;
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
      GSM_GPRS_CNT_STS = true;
    }
  }
  else
  {
    GSM_GPRS_CNT_STS = true;
  }
   if (!MQTT_client_coord.connected())
  {
    DBG("Establishing MQTT connection to ", MQTT_brokerURL);
    boolean status = MQTT_client_coord.connect("MQTT_client_coord", "NULL", "NULL");     
    char* message = "{\"test\":\"success\"}";
    int length = strlen(message);
    boolean retained = true;
    MQTT_client_coord.publish("get_attr", (byte*)message, length, retained);
    delay(1000);
    if (status == false)
    {
      DBG("Failed to setup MQTT_client_01 connection !!! ");
      delay(1 * 1000);
      MQTT_CLIENT_COORD_CNT_STS = false;
    }
    else
    {
      MQTT_CLIENT_COORD_CNT_STS = true;
      DBG("MQTT connection established for MQTT_client_coord :-) ");
      boolean sub_sts = MQTT_client_coord.subscribe(get_attr);
      if (sub_sts == false)
      {
        Serial.printf("\nsending the subscribe failed\n");
      }
      if (sub_sts == true)
      {
        Serial.printf("\nSubscription succes\n");
      }
    }
  }
  if (MQTT_client_coord.connected())
  {
    MQTT_CLIENT_COORD_CNT_STS = true;
  }
  if (!MQTT_client_01.connected())
  {
    DBG("Establishing MQTT_client_01 connection to ", MQTT_brokerWsURL);
    boolean status = MQTT_client_01.connect("GsmClient_01", "p0lsPBWqHD2AWhmKUlEu", "NULL");       
    if (status == false)
    {
      DBG("Failed to setup MQTT_client_01 connection !!! ");
      delay(1 * 1000);
      MQTT_CLIENT_01_CNT_STS = false;
    }
    else
    {
      MQTT_CLIENT_01_CNT_STS = true;
      DBG("MQTT_client_01 connection established :-) ");
    }
  }
  if (MQTT_client_01.connected())
  {
    MQTT_CLIENT_01_CNT_STS = true;
  }
  if (!MQTT_client_02.connected())
  {
    DBG("Establishing MQTT_client_02 connection to ", MQTT_brokerWsURL);
    boolean status = MQTT_client_02.connect("GsmClient_02", "st8T3pTKQ2LUBWBhCrzk", "NULL");     
    if (status == false)
    {
      DBG("Failed to setup MQTT_client_02 connection !!! ");
      delay(1 * 1000);
      MQTT_CLIENT_02_CNT_STS = false;
    }
    else
    {
      MQTT_CLIENT_02_CNT_STS = true;
      DBG("MQTT_client_02 connection established :-) ");
    }
  }
  if (MQTT_client_02.connected())
  {
    MQTT_CLIENT_02_CNT_STS = true;
  }

  if (GSM_NETWRK_CNT_STS && GSM_GPRS_CNT_STS)
  {
    GSM_CNT_STS = true;
  }
  
  char rcvdCoordResp[256],GW_coordMsg[256];
  if (xQueueReceive(Queue6, (void *)&rcvdCoordResp, (portTickType)0))
  {
            Serial.printf("\nReceived coord responce: <%s>\n",rcvdCoordResp);   
            root_coordResp["put_attr"] = rcvdCoordResp;
            serializeJson(jsonDoc_coordResp, GW_coordMsg);
            Serial.printf("\nGW_coordMsg:");
            Serial.printf(GW_coordMsg);
            if (!MQTT_client_coordResp.connected())
            {
                MQTT_CLIENT_COORD_RESP_CNT_STS == false;
                boolean status = MQTT_client_coordResp.connect("GsmClient_coordResp", "BE8RpfrsvVNaumB2o32V", "NULL");       
                if (status == false)
                {
                  Serial.printf("\nMQTT_client_coordResp connect failed...");
                  MQTT_CLIENT_COORD_RESP_CNT_STS = false;  
                }
                else
                {
                  Serial.printf("\nMQTT_client_coordResp connect success...");
                  MQTT_CLIENT_COORD_RESP_CNT_STS = true;
                }
            }
            if(MQTT_client_coordResp.connected())
            {
              MQTT_CLIENT_COORD_RESP_CNT_STS = true;
            }
            if (MQTT_CLIENT_COORD_RESP_CNT_STS == true && GSM_CNT_STS == true)  
            {
                  boolean rc = MQTT_client_coordResp.publish("v1/devices/me/telemetry", GW_coordMsg);
                  if(rc == true)
                  {
                    Serial.printf("\nCoord Response was publish successfully...");
                  }
                  else
                  {
                    if(!MQTT_client_coordResp.connected())
                    {
                      Serial.printf("\nMQTT_client_coordResp connection down !!");
                    }
                    else
                    {
                      Serial.printf("\nMQTT_client_coordResp connection was up and publish failed !!");
                    }
                  }    
            }
            else
            {
              Serial.printf("\nDropping CoordResp Msg !!\n"); 
            }
            MQTT_client_coordResp.disconnect();
  }
   
  if (xQueueReceive(Queue3, &rcvdNodePyld, (portTickType)0))
  {
    Serial.printf("\nReceived Complete payload: ");
    Serial.printf("%s", rcvdNodePyld);
    Serial.printf("\n");
    char rcvdNodeName[13];
    int rcvdNodeNameCnt = 0, rcvdNodePyldLen;
    rcvdNodePyldLen = strlen(rcvdNodePyld);
    for (int i = 9; i < 21; i++)
    {
      Serial.printf("%c", rcvdNodePyld[i]);
      rcvdNodeName[rcvdNodeNameCnt] = rcvdNodePyld[i];
      rcvdNodeNameCnt++;
    }
    rcvdNodeName[rcvdNodeNameCnt] = '\0';
    if (strcmp(rcvdNodeName, "WSN-fe0d4a46") == 0)
    {
      Serial.printf("\nReceived Node Name:[node-1] <%s>", rcvdNodeName);
      if (MQTT_CLIENT_01_CNT_STS == true && GSM_CNT_STS == true)  
      {
        boolean rc = MQTT_client_01.publish("v1/devices/me/telemetry", rcvdNodePyld);
        MQTT_stats.pubCnt ++;
        if (rc == false)
        {
          MQTT_stats.pubFlrCnt ++;
          DBG("MQTT publish failed !!!, cnt: ", MQTT_stats.pubFlrCnt);
          if (MQTT_client_01.connected())
          {
            DBG("MQTT publish failed but MQTT conn is still up");
            MQTT_stats.pubFlrConnUpCnt ++;
          }
          else
          {
            DBG("MQTT publish failed and MQTT conn is down");
            MQTT_stats.pubFlrConnDownCnt ++;
          }
        }
        else
        {
          MQTT_stats.pubSuccessCnt ++;
          DBG("MQTT publish successful, cnt: ", MQTT_stats.pubSuccessCnt);
          Serial.printf("\n**********************MSG Published to Cloud - Success****************\n");
        }
        Serial.printf("\n\n\n");
      }
      else
      {
        Serial.printf("\nDropping Payload !!\n");
      }
    }

    if (strcmp(rcvdNodeName, "WSN-fe0d5f7c") == 0)
    {
      Serial.printf("\nReceived Node Name[node-2]: <%s>", rcvdNodeName);
      if (MQTT_CLIENT_02_CNT_STS == true && GSM_CNT_STS == true)  
      {
        boolean rc = MQTT_client_02.publish("v1/devices/me/telemetry", rcvdNodePyld);
        MQTT_stats.pubCnt ++;
        if (rc == false)
        {
          MQTT_stats.pubFlrCnt ++;
          DBG("MQTT publish failed !!!, cnt: ", MQTT_stats.pubFlrCnt);
          if (MQTT_client_02.connected())
          {
            DBG("MQTT publish failed but MQTT conn is still up");
            MQTT_stats.pubFlrConnUpCnt ++;
          }
          else
          {
            DBG("MQTT publish failed and MQTT conn is down");
            MQTT_stats.pubFlrConnDownCnt ++;
            MQTT_CLIENT_01_CNT_STS = false;
          }
        }
        else
        {
          MQTT_stats.pubSuccessCnt ++;
          DBG("MQTT publish successful, cnt: ", MQTT_stats.pubSuccessCnt);
          Serial.printf("\n**********************MSG Published to Cloud - Success****************\n");
        }
        Serial.printf("\n\n\n");
      }
      else
      {
        Serial.printf("\nDropping Payload !!\n");
      }
    }
    rcvdNodePyld[0] = '\0';
  }
  MQTT_client_01.loop();
  MQTT_client_02.loop();
  MQTT_client_coord.loop();
  return;
}
