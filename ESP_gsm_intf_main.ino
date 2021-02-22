#include "driver/uart.h"

#include "coord_intf.h"
#include "dis.h"
#include "gw.h"
#include "pltfrm.h"

#define TINY_GSM_MODEM_SIM868

/*
 * There are three serial ports on the ESP32 known as U0UXD, U1UXD and U2UXD all 
 * work at 3.3V TTL Level. There are three hardware supported serial interfaces 
 * on the ESP32 known as UART0, UART1 and UART2. Like all peripherals, the pins 
 * for the UARTs can be logically mapped to any of the available pins on the ESP32. 
 * However, the UARTs can also have direct access which marginally improves 
 * performance. The pin mapping table for this hardware assistance is as follows.
 *
 *  ----------------------------------------
 *  UART    RX Pin   TX Pin   CTS     RTS
 *  ----------------------------------------
 *  UART0   GPIO3    GPIO1    N/A     N/A
 *  UART1   GPIO9    GPIO10   GPIO6   GPIO11
 *  UART2   GPIO16   GPIO17   GPIO8   GPIO7
 *  ----------------------------------------
 */

// Set serial for debug console (to the Serial Monitor, default speed 115200)
// UART0
#define SerialMon Serial

// UART1
// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define GSM_serialObj  Serial1
#define GSM_RX_PIN  16 
#define GSM_TX_PIN  17

// UART2 
#define COORD_INTF_SERIAL_PORT_NR  UART_NUM_2
#define COORD_INTF_RX_PIN  18
#define COORD_INTF_TX_PIN  19



// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG  SerialMon

// Range to attempt to autobaud
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 57600

/*
   Tests enabled
*/
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_WIFI false
#define TINY_GSM_TEST_TCP true
//#define TINY_GSM_TEST_SSL true
#define TINY_GSM_TEST_CALL true
//#define TINY_GSM_TEST_SMS true
//#define TINY_GSM_TEST_USSD true
#define TINY_GSM_TEST_BATTERY true
#define TINY_GSM_TEST_TEMPERATURE true
#define TINY_GSM_TEST_GSM_LOCATION true
#define TINY_GSM_TEST_TIME true
#define TINY_GSM_TEST_GPS false
//#define TINY_GSM_TEST_GPS true
// powerdown modem after tests
#define TINY_GSM_POWERDOWN true
#define GSM_MQTT_TCP true

// set GSM PIN, if any
#define GSM_PIN ""

// Set phone numbers, if you want to test SMS and Calls
#define SMS_TARGET  "+919123562470"
#define CALL_TARGET "+919123562470"

// Your GPRS credentials, if any
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "YourSSID";
const char wifiPass[] = "YourWiFiPass";

// Server details to test TCP/SSL
const char server[] = "thingspeak.com";
const char resource[] = "/TinyGSM/logo.txt";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


char GW_jsonMsg[1024];

#ifdef GSM_MQTT_TCP         
#include <PubSubClient.h>
const char* MQTT_brokerURL = "broker.hivemq.com";         // MQTT broker
//const char* MQTT_brokerURL = "dashboard.wisense.in";
const char* MQTT_topicLed = "GsmClientTest/led";
const char* MQTT_topicInit = "GsmClientTest/init";
const char* MQTT_topicLedStatus = "GsmClientTest/ledStatus";
#endif


#if TINY_GSM_TEST_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_TEST_GPRS
#undef TINY_GSM_TEST_WIFI
#define TINY_GSM_TEST_GPRS false
#define TINY_GSM_TEST_WIFI true
#endif

#if TINY_GSM_TEST_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif


TinyGsm modem(GSM_serialObj);


// mqtt configuration

#ifdef GSM_MQTT_TCP

typedef struct
{
  uint32_t pubCnt;
  uint32_t pubFlrCnt;
  uint32_t pubSuccessCnt;
  uint32_t pubFlrConnUpCnt;
  uint32_t pubFlrConnDownCnt;
} MQTT_stats_t;



uint32_t GW_txToCloudPendCnt = 10;

MQTT_stats_t MQTT_stats = {0, 0, 0, 0, 0};

/*
 * Create one or more TinyGSM client instances
 * For a single connection, use "TinyGsmClient client(modem);".
 */
TinyGsmClient TinyGSM_client(modem);

/*
 * The PubSubClient library provides a client for doing simple 
 * publish/subscribe messaging with a server that supports MQTT.
 */
PubSubClient MQTT_client(TinyGSM_client);
bool GSM_CNT_STS = false,
     GSM_NETWRK_CNT_STS = false,
     GSM_GPRS_CNT_STS = false,
     GSM_CLOUD_CNT_STS = false;
/* GSM_CNT_STS = Network & GPRS Connect STS, GSM_NETWRK_CNT_STS = Network Connect STS, GSM_GPRS_CNT_STS = GPRS Connect STS, GSM_CLOUD_CNT_STS = MQTT/Cloud Connect sts  */     
#define LED_PIN 13

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;

/*call back function */
void MQTT_callback(char* topic, byte* payload, unsigned int len) 
{
   SerialMon.print("Message arrived [");
   SerialMon.print(topic);
   SerialMon.print("]: ");
   SerialMon.write(payload, len);
   SerialMon.println();

   // Only proceed if incoming message's topic matches
   if (String(topic) == MQTT_topicLed) 
   {
       ledStatus = !ledStatus;
       digitalWrite(LED_PIN, ledStatus);
       MQTT_client.publish(MQTT_topicLedStatus, ledStatus ? "1" : "0");
   }
}

boolean MODEM_restarted = false;

#endif

#define EVENT_FMT_TYPE_MERGED_DATA_JSON                     //used to send complete node data in one json

#define COORD_INTF_UART_DRIVER_BUFF_SIZE (1024 * 2)
#define COORD_INTF_EVT_QUEUE_SIZE  2
//queues
static QueueHandle_t CoordIntf_evtQHndl;
static QueueHandle_t Queue2;
static QueueHandle_t Queue3;


#define COORD_INTF_TASK_RX_MSG_BUFF_LEN  256
#define COORD_INTF_TASK_RX_RAW_BUFF_LEN  256

uint8_t COORD_INTF_taskRxRawBuff[COORD_INTF_TASK_RX_RAW_BUFF_LEN + 16];
uint8_t COORD_INTF_taskRxMsgBuff[COORD_INTF_TASK_RX_MSG_BUFF_LEN + 16];

struct uart_data_send
    {
        int readCnt_q=0;
        uint8_t COORD_INTF_taskRxRawBuff_q[COORD_INTF_TASK_RX_RAW_BUFF_LEN + 16];
    } uart_data_send_str;


//configure UART's
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

    // Get the tlv type
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


boolean COORD_INTF_checkForHdr(uint8_t *hdr_p)
{
   boolean rc = false;
   const uint16_t calcCkSum = COORD_INTF_calcCkSum16(hdr_p,
                                                     COORD_INTF_FRAME_HDR_LEN - COORD_INTF_FRAME_HDR_CRC_FIELD_LEN*2);
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

/* cloud functions  */
//1
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
 int node_rssi,
     node_lqi, 
     node_process_sts = 0,
     GW_DataBufLen=0,
     GW_relaySnsrDataToCloud = 1;
 char Node_macIdBuff[64],
      GW_devName[1024],
      GW_macBuff[1024];
uint32_t seq_nr=0;
int merge_store[10],merge_count=0;                        //to store each buffer length (for each packet's)
int snsrIdStore[10];     
char merge_event[100];
char GW_snsrDataBuff[512] = {'\0'};

char snsrIdStore0[16],
     snsrIdStore1[16],
     snsrIdStore2[16],
     snsrIdStore3[16],
     snsrIdStore4[16],
     snsrIdStore5[16],
     snsrIdStore6[16],
     snsrIdStore7[16],
     snsrIdStore8[16];
char GW_snsrDataBuff_0[150]={'\0'},
     GW_snsrDataBuff_1[150]={'\0'},
     GW_snsrDataBuff_2[150]={'\0'},
     GW_snsrDataBuff_3[150]={'\0'},
     GW_snsrDataBuff_4[150]={'\0'},
     GW_snsrDataBuff_5[150]={'\0'},
     GW_snsrDataBuff_6[150]={'\0'},
     GW_snsrDataBuff_7[150]={'\0'},
     GW_snsrDataBuff_8[150]={'\0'};
char GW_get_time[150]={'\0'};
float snsrDataBuff[10]={0};

/*  for every packet the node data valF respect to the unit_p will differ, remaining the macid= extAddr_p, snsrId will remain same if the data receive from one sensor node 
    storing snsId's at specific rate: id[0] = node_voltage, id[1]=msp temp, id[2]=humdity, id[3]=temp ....n
*/

void GW_appendToEventBufferJson(unsigned char *extAddr_p,
                            int snsrId,
                            char *unit_p,
                            float valF)
{
     Serial.printf("\nEntering GW_appendToEventBufferJson -1 \n");
    int strLen = strlen(GW_snsrDataBuff);
    if(merge_count==0)
    {
      Serial.printf("\nmerge_count-0 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
        merge_store[merge_count] = strLen;
        snsrIdStore[merge_count] = snsrId;
        snsrDataBuff[merge_count]= valF;
    }
    if(merge_count==1)
    {
      Serial.printf("\nmerge_count-1 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
        merge_store[merge_count] = strLen;
        snsrIdStore[merge_count] = snsrId,
        snsrDataBuff[merge_count]= valF;
    }
    if(merge_count==2)
    {
      Serial.printf("\nmerge_count-2 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
        merge_store[merge_count] = strLen;   
        snsrIdStore[merge_count] = snsrId;
        snsrDataBuff[merge_count]= valF;
    }
    if(merge_count==3)
    {
      Serial.printf("\nmerge_count-3 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
        merge_store[merge_count] = strLen;
        snsrIdStore[merge_count] = snsrId;
        snsrDataBuff[merge_count]= valF;
   }
   /* test; add in addition of using multiple sensor respect to data count n = sensor data count
   if(merge_count==n)
   {
       merge_store[merge_count] = strLen;
       snsrIdStore[merge_count] = snsrId;
       snsrDataBuff[merge_count] = valF;
   }
   */
   if(merge_count==4)
   {
       Serial.printf("\nmerge_count-4 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
       merge_store[merge_count] = strLen;
       snsrIdStore[merge_count] = snsrId;
       snsrDataBuff[merge_count] = valF;
   }
    if(merge_count==5)
   {
       Serial.printf("\nmerge_count-5 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
       merge_store[merge_count] = strLen;
       snsrIdStore[merge_count] = snsrId;
       snsrDataBuff[merge_count] = valF;
   }
   if(merge_count==6)
   {
       Serial.printf("\nmerge_count-6 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
       merge_store[merge_count] = strLen;
       snsrIdStore[merge_count] = snsrId;
       snsrDataBuff[merge_count] = valF;
   }
   if(merge_count==7)
   {
       Serial.printf("\nmerge_count-7 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
       merge_store[merge_count] = strLen;
       snsrIdStore[merge_count] = snsrId;
       snsrDataBuff[merge_count] = valF;
   }
   if(merge_count==8)
   {
       Serial.printf("\nmerge_count-8 snsrId <0x%x> and valF <%.2f>\n",snsrId,valF);
       merge_store[merge_count] = strLen;
       snsrIdStore[merge_count] = snsrId;
       snsrDataBuff[merge_count] = valF;
   }
      if(strLen == 0)
   { 
        strLen = sprintf(GW_snsrDataBuff, "ID_%02x%02x%02x%02x",                                //GW_snsrDataBuff will store the merged string here Device macId 
                        extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
        merge_store[merge_count] = strLen;  //Node macId length storing in the 0 location of merge_store
        snsrIdStore[merge_count] = snsrId;
        /*
            sprintf(GW_eventBuff, "Total Buffer Length: ");
            sprintf(GW_TotalBuffCount,"Count = %d",GW_DataBufLen);
            Particle.publish(GW_eventBuff, GW_TotalBuffCount);
            
            sprintf(GW_eventBuff, "Merge_store_Check: ");
            sprintf(merge_event,"0=%d,1=%d,2=%d,3=%d,4=%d",merge_store[0],merge_store[1],merge_store[2],merge_store[3],merge_store[4]);
            Particle.publish(GW_eventBuff,merge_event);
            GW_snsrDataBuffLen= GW_DataBufLen;
            GW_DataBufLen=0;
            merge_count=0;
        */
   } 
   if(strLen == 0)
   { 
        strLen = sprintf(GW_snsrDataBuff, "ID_%02x%02x%02x%02x",                                //GW_snsrDataBuff will store the merged string here Device macId 
                        extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
        merge_store[merge_count] = strLen;  //Node macId length storing in the 0 location of merge_store
        snsrIdStore[merge_count] = snsrId;
   }
    merge_count++;

    sprintf(GW_devName, "WSN-%02x%02x%02x%02x", extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
    sprintf(GW_macBuff,"%02x:%02x:%02x:%02x", extAddr_p[4],extAddr_p[5],extAddr_p[6],extAddr_p[7]);
    Serial.printf("\nGW_devName <%s> GW_macBuff <%s>\n",GW_devName,GW_macBuff);
    GW_DataBufLen = GW_DataBufLen + strLen; 
    sprintf(GW_snsrDataBuff + strLen,  "__S%d_V%.3f_U%s", snsrId, valF, unit_p);                         //complete merged string it will store (2_t)
}
/*
 ********************************************************************
 *
 *
 *
 ********************************************************************
 */
void GW_appendIntToEventBufferJson(unsigned char *extAddr_p,
                               int snsrId,
                               char *unit_p,
                               int valI)
{
   int strLen = strlen(GW_snsrDataBuff);
   
   if (strLen == 0)
   { 
       strLen = sprintf(GW_snsrDataBuff, "ID_%02x%02x%02x%02x",            
                        extAddr_p[4], extAddr_p[5], extAddr_p[6], extAddr_p[7]);
   }
   sprintf(GW_snsrDataBuff + strLen,  "__S%d_V%d_U%s", snsrId, valI, unit_p);
}

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */

void GW_sendMergedDataEvtToCloudJson(void)
{
  Serial.printf("\n Node Data receives Completely, Proceeding JSON \n");
  //add the node_process_sts statements inside the function and call in main or after processing of data completes[processing of one complete node data]
}

#endif



float __latestVcc;
int __latestVccSet = 0;

unsigned short GW_ntohs(unsigned char *buff_p)
{
   short u16Val = *buff_p;
   u16Val = (u16Val << 8) | buff_p[1];  
   return u16Val;
}


 const double RTD_A = (float)3.9083e-3;
 const double RTD_B = (float)-5.775e-7;
 double RTD_1000_processADCVal(unsigned int adcVal)
 {
   double RTDnominal = 1000, refResistor = 4300;
  double Z1, Z2, Z3, Z4, Rt, temp;

   // printf("\n adcVal : %u \n", adcVal);

   Rt = adcVal;
   Rt /= 32768;
   Rt *= refResistor;

   // printf("\n Rt:%f, RTD_A:%f, RTD_B:%f \n", Rt, RTD_A, RTD_B);

   Z1 = -RTD_A;
   Z2 = RTD_A * RTD_A - (4 * RTD_B);
   Z3 = (4 * RTD_B) / RTDnominal;
   Z4 = 2 * RTD_B;

   temp = Z2 + (Z3 * Rt);
   temp = (sqrt(temp) + Z1) / Z4;

   // printf("\n temp: %f \n", temp);

   if (temp >= 0) return temp;

   // ugh.
   Rt /= RTDnominal;
   Rt *= 100;      // normalize to 100 ohm

   float rpoly = Rt;

   temp = -242.02;
   temp += 2.2228 * rpoly;
   rpoly *= Rt;  // square
   temp += 2.5859e-3 * rpoly;
   rpoly *= Rt;  // ^3
   temp -= 4.8260e-6 * rpoly;
   rpoly *= Rt;  // ^4
   temp -= 2.8183e-8 * rpoly;
   rpoly *= Rt;  // ^5
   temp += 1.5243e-10 * rpoly;

   return temp;
 }
// queue task test


void CoordIntf_procCoordMsg(void *params_p)
{
 // struct uart_data_send *uart_data_rcvd_frmQ1; 
  uint8_t rec_data_q2[272];
  while (1)
  {
    DBG("Task 2 running...");
     if (xQueueReceive(Queue2, (void *)&rec_data_q2, (portTickType)portMAX_DELAY)) 
     {
      DBG("Queue2 received...");
     // DBG("\nReadcnt received <%d> \n",uart_data_rcvd_frmQ1->readCnt_q);
     // DBG("\nReceived bytes...\n");
     // for(int i=0;i<uart_data_rcvd_frmQ1->readCnt_q;i++)
     // {
     //   DBG(uart_data_rcvd_frmQ1->COORD_INTF_taskRxRawBuff_q[i]);
     // }

      for(int i=0;i<55;i++)
      {
        DBG("|",rec_data_q2[i]);
      }
      
      //xQueueSend(Queue3,(void*)&rec_data_q2,(TickType_t)5);
     }
    vTaskDelay(10000/portTICK_PERIOD_MS); 
  }   
}



// queue task test

 
 
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
   Serial.printf("\nProcesing complete msg... done\n");
       
   srcShortAddr = buff_p[off];
   srcShortAddr = (srcShortAddr << 8) | buff_p[off + 1];
   
   off += DIS_LPWMN_MAC_SHORT_ADDR_LEN;
 
#if 1  
   Serial.printf("C_I_pNM> [%u] Received msg from node <%05u / %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x>\n", 
                COORD_IF_nodeMsgCnt,
                srcShortAddr, 
                (unsigned int)buff_p[off],
                (unsigned int)buff_p[off+1],
                (unsigned int)buff_p[off+2],
                (unsigned int)buff_p[off+3],
                (unsigned int)buff_p[off+4],
                (unsigned int)buff_p[off+5],
                (unsigned int)buff_p[off+6],
                (unsigned int)buff_p[off+7]);   
                sprintf(Node_macIdBuff,"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", buff_p[off],buff_p[off+1],buff_p[off+2],buff_p[off+3],buff_p[off+4],buff_p[off+5],buff_p[off+6],buff_p[off+7]); 
#endif

   extAddr_p = buff_p + off;
   buff_p += DIS_LPWMN_MAC_EXT_ADDR_LEN;
   
   // LPWMN_updateNodeList(srcShortAddr, extAddr_p, 1);

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
      /*
        if (disMsgType  == DIS_MSG_TYPE_ATUAT_TAG_BCN_INFO)
        {
           GW_processTagBcn(extAddr_p, buff_p, msgLen);
            return;
        }  
      */
  
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
                  merge_count=0;
                  break;
              } 
              else
              {
                  uint8_t tlvLen3, *buff3_p;
                  int snsrId, scaleFactor = DIS_DATA_SCALE_CENTI;

                  // Make buff1_p point to the end of the current DIS_TLV_TYPE_SENSOR_OUTPUT TLV
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
                          // if (verbose)
                          //     Serial.printf("Found Scale factor <%d> \r\n", scaleFactor); 
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
       
                      switch(tlvLen3)
                      {
                         case 1:
                              snsrOp = (int)(*buff3_p);
                              //Serial.printf("SensorOp 1 <%x> <%d>",snsrOp,snsrOp);
                              break;
                    
                         case 2:
                              {
                                snsrOp16 = UTIL_ntohs(buff3_p);
                                snsrOp = snsrOp16;
                                Serial.printf("SensorOp-2 <%x> <%d>",snsrOp,snsrOp);                          
                              }
                              break;
                              
                         case 3:
                              {
                                uint8_t lBuff[4];
                                lBuff[0] = 0;
                                memcpy(lBuff + 1, buff3_p, 3);
                                snsrOp = (int)UTIL_ntohl(lBuff);
                                printf("C_I_pNM>  <1b> snsrOp : %d\n", snsrOp);
                                //Serial.printf("SensorOp 3 <%x> <%d>",snsrOp,snsrOp);
                              }
                              break;
                              
                         case 4:
                              snsrOp = (int)UTIL_ntohl(buff3_p);
                              break;
                              //Serial.printf("SensorOp 4 <%x> <%d>",snsrOp,snsrOp);

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
                         case PLTFRM_DO_SNSR_1_DEV_ID:      
                         case PLTFRM_ADS1015_1_DEV_ID:
                              // TODO - Remove PLTFRM_ADS1015_1_DEV_ID
                              unit_p = "uVolts";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;
                              
                         case PLTFRM_PH_SNSR_1_DEV_ID:
                              unit_p = "mVolts";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
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

                         case PLTFRM_GEN_VOLT_MON_DEV_ID:
                              // Serial.printf("+[Ext Voltage]   ");
                              unit_p = "Volts";
                              break;

                         case PLTFRM_GEN_CURRENT_MON_DEV_ID:
                              // Serial.printf("+[Ext Current]   ");
                              unit_p = "mA";
                              break;
                         
                         case PLTFRM_AD7797_1_DEV_ID:
                              // Serial.printf("+[Load Cell]   ");
                              unit_p = "KG";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;
 
                         case PLTFRM_MP3V5050GP_1_DEV_ID:
                              // Serial.printf("+[P_MP3V5050GP]   ");
                              unit_p = "kPa";
                              break;

                         case PLTFRM_MP3V5010_1_DEV_ID:
                              // Serial.printf("+[P_MP3V5010]   ");
                              unit_p = "%";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;
 
                         case PLTFRM_MPXV5010G_1_DEV_ID:
                              // Serial.printf("+[P_MPXV5010G]   ");
                              unit_p = "mm of water";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_MP3V5004GP_1_DEV_ID:
                              // Serial.printf("+[P_MP3V5004GP]   ");
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "mv"; // "kPa";
                              break;

                         case PLTFRM_MPL115A2_1_DEV_ID:
                              // Serial.printf("+[P_MPL115A2]   ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "Pa";
                              break;

                         case PLTFRM_MS5637_1_DEV_ID:
                              // Serial.printf("+[P_MS5637]     ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "mbar";
                              break;

                         case PLTFRM_LLS_1_DEV_ID:
                              // Serial.printf("+[LLS_POT]   ");
                              scaleFactor = DIS_DATA_SCALE_DECI;
                              unit_p = "Ohms";
                              break;

                         case PLTFRM_LM75B_1_DEV_ID:
                              // Serial.printf("+[Temp_LM75B]   ");
                              unit_p = "Deg C";
                              break;

                         case PLTFRM_P43_US_1_DEV_ID:
                              // Serial.printf("+[P43_US]  ");
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "V";
                              break;

                         case PLTFRM_HPM_1_PM2PT5_DEV_ID:
                              // Serial.printf("+[HPM PM 2.5]   ");
                              unit_p = "";
                              break;

                         case PLTFRM_HPM_1_PM10_DEV_ID:
                              // Serial.printf("+[HPM PM 10]   ");
                              unit_p = "";
                              break;

                         case PLTFRM_AS339_1_DIVER_PRESSURE_DEV_ID: 
                              // Serial.printf("+[Diver Pressure x 10]  ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "cmH2O";
                              break;

                         case PLTFRM_AS339_1_DIVER_TEMPERATURE_DEV_ID: 
                              // Serial.printf("+[Diver Temperature]  ");
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              unit_p = "Deg C";
                              break;

                         case PLTFRM_AS339_1_DIVER_MOD_PRESSURE_DEV_ID: 
                              // Serial.printf("+[Diver-MOD Pressure x 10]  ");
                              scaleFactor = DIS_DATA_SCALE_DECI;
                              unit_p = "cmH2O";
                              break;

                         case PLTFRM_AS339_1_DIVER_MOD_TEMPERATURE_DEV_ID: 
                              // Serial.printf("+[Diver-MOD Temperature]  ");
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              unit_p = "Deg C";
                              break;

                         case PLTFRM_TEPT5700_1_DEV_ID:
                              // Serial.printf("+[Light_TEPT5700]   ");
                              unit_p = "uA";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;

                         case PLTFRM_CC2D33S_1_RH_DEV_ID:
                              // Serial.printf("+[RH_CC2D33S]   ");
                              unit_p = "%";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_CC2D33S_1_TEMP_DEV_ID:
                              // Serial.printf("+[Temp_CC2D33S] ");
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_DUAL_FS_LVL_MON_1_DEV_ID:
                              // Serial.printf("+[Dual_Float_Switch_Tank_Mon]  ");
                              unit_p = "";
                              break;

                         case PLTFRM_MAG3110_1_DEV_ID:
                              // Serial.printf("+[MFS_MAG3110]  ");
                              unit_p = "uT";
                              break;

                         case PLTFRM_WS_VEH_DET_1_DEV_ID:
                              // Serial.printf("+[WISENSE_VEH_MD]     ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "";
                              break;

                         case PLTFRM_MDS_1_DEV_ID:
                              // Serial.printf("+[Mains Monitor]  ");
                              unit_p = "";
                              break;
                         
                         case PLTFRM_VIBRATION_SNSR_1_DEV_ID:
                              // Serial.printf("+[Vibration Monitor]  ");
                              unit_p = "";
                              break;

                         case PLTFRM_EKMC160111X_1_DEV_ID:
                              // Serial.printf("+[Motion Sensed Count]     ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "";
                              break;
                          
                         case PLTFRM_PULSE_CNTR_2_DEV_ID:
                              // Serial.printf("+[Pulse Count (Rain Gauge)]  ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "";
                              break;

                         case PLTFRM_PULSE_CNTR_1_DEV_ID:
                              // Serial.printf("+[Pulse Count (Water Meter)]  ");
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              unit_p = "";
                              break;

                         case PLTFRM_SHT10_1_TEMP_DEV_ID:
                              // Serial.printf("+[Temp (SHT10)] ");
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_SHT15_1_TEMP_DEV_ID:
                              // Serial.printf("+[Temp (SHT15)] ");
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_SHT10_1_RH_DEV_ID:
                              // Serial.printf("+[RH (SHT10)]   ");
                              unit_p = "%";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_SHT15_1_RH_DEV_ID:
                              // Serial.printf("+[RH (SHT15)]   ");
                              unit_p = "%";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;

                         case PLTFRM_SOL_PSU_1_VSOL_DEV_ID: 
                              // Serial.printf("+[SOL-PSU PANEL_V");
                              unit_p = "Volts";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_SOL_PSU_1_VBATT_DEV_ID: 
                              // Serial.printf("+[SOL-PSU BATT_V");
                              unit_p = "Volts";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_SOL_PSU_1_VSYS_DEV_ID: 
                              // Serial.printf("+[SOL-PSU SYS_V");
                              unit_p = "Volts";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_SOL_PSU_1_ISOL_DEV_ID: 
                              // Serial.printf("+[SOL-PSU PANEL_I");
                              unit_p = "mA";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_SOL_PSU_1_IBATT_DEV_ID: 
                              // Serial.printf("+[SOL-PSU BATT_I");
                              unit_p = "mA";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_INA219_1_BV_DEV_ID:
                              // Serial.printf("+[INA219_BUS_V]  ");
                              unit_p = "milli-volts";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;
                         
                         case PLTFRM_INA219_3_BV_DEV_ID:
                              // Serial.printf("+[INA219_BUS_V]  ");
                              unit_p = "milli-volts";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;
                         
                         case PLTFRM_ACS712_1_CURRENT_DEV_ID:
                              // Serial.printf("+[ACS712_1_S_V]  ");
                              unit_p = "milli-volts";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;

                         case PLTFRM_ACS712_2_CURRENT_DEV_ID:
                              // Serial.printf("+[ACS712_2_S_V]  ");
                              unit_p = "milli-volts";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;

                         case PLTFRM_ACS712_3_CURRENT_DEV_ID:
                              // Serial.printf("+[ACS712_3_S_V]     ");
                              unit_p = "milli-volts";
                              scaleFactor = DIS_DATA_SCALE_NONE;
                              break;

                         case PLTFRM_INA219_1_SV_DEV_ID:
                              // Serial.printf("+[INA219_CURRENT]   ");
                              unit_p = "milli-amps";
                              break;

                         case PLTFRM_INA219_3_SV_DEV_ID:
                              // Serial.printf("+[INA219_CURRENT]   ");
                              unit_p = "milli-amps";
                              break;

                         case PLTFRM_NTCALUG02A_1_DEV_ID:
                              // Serial.printf("+[NTC_THERM (Vishay NTCALUG02A)]   ");
                              unit_p = "deg C";
                              break;
                              
                         case PLTFRM_APS_1_DEV_ID:
                              unit_p = "milli-psi";
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              break;

                         case PLTFRM_NXFT15XH103_1_DEV_ID:
                         case PLTFRM_NXFT15XH103_2_DEV_ID:
                              // Serial.printf("+[NTC_THERM (Murata NXFT15XH103)]   ");
                              unit_p = "deg C";
                              break;

                         case PLTFRM_DNAX300R103L040_1_DEV_ID:
                              // Serial.printf("+[NTC_THERM (Deem DNAX300R103L040)]   ");
                              unit_p = "deg C";
                              break;
                         
                         case PLTFRM_NTCALUG02A_2_DEV_ID:
                              // Serial.printf("+[NTC_THERM (Vishay NTCALUG02A)]   ");
                              unit_p = "deg C";
                              break;
                               
                         case PLTFRM_BAR_CODE_SCANNER_1_DEV_ID:
                              // Serial.printf("+[Bar Code]     ");
                              unit_p = "";
                              break;

                         case PLTFRM_MAX_SONAR_1_DEV_ID:
                              // Serial.printf("+[Dist_MaxSonar_1 ]  ");
                              unit_p = "Inches";
                              break;

                         case PLTFRM_MAX_SONAR_2_DEV_ID:
                              // Serial.printf("+[Dist_MaxSonar_2 ]  ");
                              unit_p = "Inches";
                              break;
                         
                         case PLTFRM_MAX_SONAR_3_DEV_ID:
                              // Serial.printf("+[Dist_MaxSonar_3 ]  ");
                              unit_p = "Inches";
                              break;
                         
                         case PLTFRM_CHIRP_PWLA_1_DEV_ID:
                              // Serial.printf("+[Moisture_CHIRP] ");
                              unit_p = "na";
                              break;

                         case PLTFRM_WSMS100_1_DEV_ID:
                              // Serial.printf("+[Moisture_WSMS100] ");
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              unit_p = "%";
                              break;

                         case PLTFRM_ON_CHIP_TEMP_SENSOR_DEV_ID:
                              // Serial.printf("+[Temp_MSP430]  ");
                              unit_p = "deg C";
                              break;

                         case PLTFRM_TSL45315_1_DEV_ID:
                              // Serial.printf("+[Light_TSL45315] ");
                              unit_p = "Lux";
                              break;

                         case PLTFRM_BATT_1_DEV_ID:
                              // Serial.printf("+[Batt Voltage] ");
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "Volts";
                              break;

                         case PLTFRM_SETRA_3100_1_DEV_ID:
                              // Serial.printf("+[SETRA 3100 Op Voltage] ");
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "Volts";
                              break;

                         case PLTFRM_DEV_TYPE_SOLAR_PWR_SRC_VSENSE:
                              // Serial.printf("+[Panel Voltage] ");
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "Volts";
                              break;

                         case PLTFRM_EXT_VOLTAGE_MON_DEV_ID:
                              // Serial.printf("+[Ext Voltage] ");
                              scaleFactor = DIS_DATA_SCALE_MILLI;
                              unit_p = "Volts";
                              break;

                         case PLTFRM_HE055T01_1_DEV_ID:
                              // Serial.printf("+[Current_HE055T01] ");
                              unit_p = "mA";
                              break;

                         case PLTFRM_WPDS_DEV_ID:
                              // Serial.printf("+[WPDS ALARM]  ");
                              break;
                        
                         case PLTFRM_TMP75C_1_DEV_ID:
                              unit_p = "Deg C";
                              scaleFactor = DIS_DATA_SCALE_CENTI;
                              break;
                              
                         case PLTFRM_HIH8121_1_RH_DEV_ID:                                       //added
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
                      }//snsrId-1
                      
                      
                      if (snsrId == PLTFRM_GEN_VOLT_MON_DEV_ID
                          || snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID)
                          scaleFactor = DIS_DATA_SCALE_MILLI;
                      
                      if (snsrId == PLTFRM_GEN_CURRENT_MON_DEV_ID)
                          scaleFactor = DIS_DATA_SCALE_NONE;

                      if (snsrId == PLTFRM_LM75B_1_DEV_ID
                          || snsrId == PLTFRM_ADS1015_1_DEV_ID
                          || snsrId == PLTFRM_DO_SNSR_1_DEV_ID
                          || snsrId == PLTFRM_P43_US_1_DEV_ID
                          || snsrId == PLTFRM_HPM_1_PM2PT5_DEV_ID
                          || snsrId == PLTFRM_HPM_1_PM10_DEV_ID
                          || snsrId == PLTFRM_AS339_1_DIVER_PRESSURE_DEV_ID
                          || snsrId == PLTFRM_AS339_1_DIVER_MOD_PRESSURE_DEV_ID
                          || snsrId == PLTFRM_AS339_1_DIVER_TEMPERATURE_DEV_ID
                          || snsrId == PLTFRM_AS339_1_DIVER_MOD_TEMPERATURE_DEV_ID
                          || snsrId == PLTFRM_BATT_1_DEV_ID
                          || snsrId == PLTFRM_SETRA_3100_1_DEV_ID
                          || snsrId == PLTFRM_AD7797_1_DEV_ID
                          || snsrId == PLTFRM_GEN_CURRENT_MON_DEV_ID
                          || snsrId == PLTFRM_GEN_VOLT_MON_DEV_ID
                          || snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID
                          || snsrId == PLTFRM_ON_CHIP_TEMP_SENSOR_DEV_ID
                          || snsrId == PLTFRM_MP3V5050GP_1_DEV_ID
                          || snsrId == PLTFRM_MP3V5010_1_DEV_ID
                          || snsrId == PLTFRM_MPXV5010G_1_DEV_ID
                          || snsrId == PLTFRM_MP3V5004GP_1_DEV_ID
                          || snsrId == PLTFRM_FC_28_1_DEV_ID
                          || snsrId == PLTFRM_ACS712_1_CURRENT_DEV_ID
                          || snsrId == PLTFRM_ACS712_2_CURRENT_DEV_ID
                          || snsrId == PLTFRM_LLS_1_DEV_ID
                          || snsrId == PLTFRM_ACS712_3_CURRENT_DEV_ID
                          || snsrId == PLTFRM_SHT10_1_RH_DEV_ID
                          || snsrId == PLTFRM_SHT10_1_TEMP_DEV_ID
                          || snsrId == PLTFRM_SHT15_1_TEMP_DEV_ID
                          || snsrId == PLTFRM_SHT15_1_RH_DEV_ID
                          || snsrId == PLTFRM_WSMS100_1_DEV_ID
                          || snsrId == PLTFRM_EXT_VOLTAGE_MON_DEV_ID
                          || snsrId == PLTFRM_DEV_TYPE_SOLAR_PWR_SRC_VSENSE
                          || snsrId == PLTFRM_CC2D33S_1_RH_DEV_ID
                          || snsrId == PLTFRM_CC2D33S_1_TEMP_DEV_ID
                          || snsrId == PLTFRM_TEPT5700_1_DEV_ID
                          || snsrId == PLTFRM_SOL_PSU_1_VSOL_DEV_ID
                          || snsrId == PLTFRM_SOL_PSU_1_VBATT_DEV_ID
                          || snsrId == PLTFRM_SOL_PSU_1_VSYS_DEV_ID
                          || snsrId == PLTFRM_MAX_SONAR_1_DEV_ID
                          || snsrId == PLTFRM_MAX_SONAR_2_DEV_ID
                          || snsrId == PLTFRM_MAX_SONAR_3_DEV_ID
                          || snsrId == PLTFRM_DS18B20_1_DEV_ID
                          || snsrId == PLTFRM_PH_SNSR_1_DEV_ID
                          || snsrId == PLTFRM_APS_1_DEV_ID
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

                          if (snsrId == PLTFRM_AD7797_1_DEV_ID)
                          { 
                          }
                          
                          if (snsrId == PLTFRM_RTD_1000_1_DEV_ID)
                            {
                                     // if (tlvLen3 == 4)
                                              //{
                                              int val;
                                              double tempVal = RTD_1000_processADCVal(snsrOp);
                                              valF=tempVal;

#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                                              GW_appendIntToEventBufferJson(extAddr_p, snsrId, "", val);
#endif
                                            //printf("<%f %s> \n", tempVal, unit_p);
                                              //}
                          }
                                                              
                                                             
                                                             
                          if (snsrId == PLTFRM_MP3V5004GP_1_DEV_ID)
                          {
                              if (__latestVccSet)
                              {
                                  float currRatio, delta;
                                  float startRatio = 0.527289;

                                  // Vout = Vs*(0.2*p + 0.2) +/- (2.5 % of VFSS)

                                  // p = ((Vout / Vs) - 0.2) * 5
                                  // Inches of water = p * 101.97


                                  currRatio = valF;
                                  currRatio /= __latestVcc;

                                  delta = startRatio;
                                  delta -= currRatio;
                                  delta /= startRatio;
                                  delta *= 100;

                                  // temp /= __latestVcc;
                                  // temp -= 0.2187;   // -0.2
                                  // temp /= .176;  // / 0.2
                                  // temp *= 101.97;  // in mm of water

                                  // printf(" <%f mm of water> \n", temp); 
                  
                                  // Serial.printf("Adc: %f V / Vcc: %f V / ratio: %f / delta: %f percent \r\n", 
                                  //               valF, __latestVcc, currRatio, delta);
                              }
                              else
                              {
                                  // Serial.printf(" <%f %s> \r\n", valF, "milli-volts");
                              }
                          }
                          

                          if (snsrId != PLTFRM_AD7797_1_DEV_ID
                              && snsrId != PLTFRM_MP3V5050GP_1_DEV_ID
                              && snsrId != PLTFRM_MP3V5004GP_1_DEV_ID
                              // && snsrId != PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID
                              )
                          {
                              // Serial.printf(" <%f %s> \r\n", valF, unit_p);
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                              GW_appendToEventBufferJson(extAddr_p, snsrId, unit_p, valF);  
#endif 

                          }
                          
                          if (snsrId == PLTFRM_ON_CHIP_VCC_SENSOR_DEV_ID)
                          {
                              __latestVcc = valF;
                              __latestVccSet = 1;
                          }
                      }
                      else
                      {
                          if (snsrId == PLTFRM_BAR_CODE_SCANNER_1_DEV_ID)
                          {
                              buff3_p[tlvLen3] = '\0';
                              // Serial.printf(" <%s %s> \r\n", buff3_p, unit_p);
                          }
                          else
                          {
                              if (snsrId == PLTFRM_NTCALUG02A_1_DEV_ID
                                  || snsrId == PLTFRM_NTCALUG02A_2_DEV_ID
                                  || snsrId == PLTFRM_NXFT15XH103_1_DEV_ID
                                  || snsrId == PLTFRM_NXFT15XH103_2_DEV_ID
                                  || snsrId == PLTFRM_DNAX300R103L040_1_DEV_ID)
                              {
                                  double _r, _rl;
                                  double t25 = 25 + 273.15;
                                  double r_t = (double)snsrOp; // 6794;
                                  int b25by85;

                                  switch (snsrId)
                                  {
                                     case PLTFRM_NTCALUG02A_1_DEV_ID:
                                     case PLTFRM_NTCALUG02A_2_DEV_ID:
                                          _r = NTC_THERM_NTCALUG02A_R25_VAL;
                                          b25by85 = NTC_THERM_NTCALUG02A_B_25_85_VAL;
                                          break;

                                     case PLTFRM_DNAX300R103L040_1_DEV_ID:
                                          _r =  NTC_THERM_DNAX300R103L040_4_R25_VAL;
                                          b25by85 = NTC_THERM_DNAX300R103L040_4_B_25_85_VAL;
                                          break;

                                     case PLTFRM_NXFT15XH103_1_DEV_ID:
                                     case PLTFRM_NXFT15XH103_2_DEV_ID:
                                          _r = NTC_THERM_NXFT15XH103_R25_VAL;
                                          b25by85 = NTC_THERM_NXFT15XH103_B_25_85_VAL;
                                          break;

                                     default:
                                          break;
                                  }

                                  // loge(R25/RT) = B * (1/T25 - 1/T)
                                  // 1/T = 1/T25 - loge(R25/RT)/B
                                  // T = 1 / (1/T25 - loge(R25/RT)/B)
    
                                  _r /= r_t;
                                  _rl = log(_r);
                                  _rl /= b25by85;
                                  _r = 1;
                                  _r /= t25;
                                  _r -= _rl;
                                  _r = (1 / _r);
                                  
                                  // Serial.printf(" <%.2f %s> \r\n", _r - 273.15, unit_p);
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                                  GW_appendToEventBufferJson(extAddr_p, snsrId, unit_p, _r - 273.15);
#endif

                              }
                              else
                              {
                                  if (snsrId == PLTFRM_INA219_1_SV_DEV_ID
                                      || snsrId == PLTFRM_INA219_3_SV_DEV_ID
                                      || snsrId == PLTFRM_SOL_PSU_1_ISOL_DEV_ID
                                      || snsrId == PLTFRM_SOL_PSU_1_IBATT_DEV_ID)
                                  {
                                      // Reported value is voltage in mv * 100
                                      float opVal = (int)snsrOp;
                                      opVal /= 100;
                                      // I = V/R
                                      if (snsrId == PLTFRM_SOL_PSU_1_ISOL_DEV_ID
                                          || snsrId == PLTFRM_SOL_PSU_1_IBATT_DEV_ID)
                                          opVal *= 10; // shunt resistance of 0.1 ohms
                                      else
                                          opVal *= 1;  // shunt resistance of 1 ohms
                                          
                                      // Serial.printf(" <%f %s> \r\n", opVal, unit_p);
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                                      GW_appendToEventBufferJson(extAddr_p, snsrId, unit_p, opVal); 
#endif
                                  }
                                  else
                                  {
                                      if (snsrId == PLTFRM_MDS_1_DEV_ID)
                                      {
                                          // Serial.printf(" <%s> \r\n", snsrOp ? "On" : "Off");
                                      }
                                      else
                                      {
                                          if (snsrId == PLTFRM_VIBRATION_SNSR_1_DEV_ID)
                                          {     
                                              // Serial.printf(" <%s> \r\n", snsrOp ? "Yes" : "No");
                                          }
                                          else 
                                          { 
                                              if (snsrId == PLTFRM_MAG3110_1_DEV_ID)
                                              {
                                                  if (tlvLen3 == 6)
                                                  {
                                                      short magFldInt = (short)GW_ntohs(buff3_p);
                                                      float magFldIntF = magFldInt;

                                                      magFldIntF *= 0.1;
                                                      // Serial.printf("x: %f uT, ", magFldIntF);

                                                      magFldInt = (short)GW_ntohs(buff3_p + 2);
                                                      magFldIntF = magFldInt;
                                                      magFldIntF *= 0.1;
                                                      // Serial.printf("y: %f uT, ", magFldIntF);

                                                      magFldInt = (short)GW_ntohs(buff3_p + 4);
                                                      magFldIntF = magFldInt;
                                                      magFldIntF *= 0.1;
                                                      // Serial.printf("z: %f uT ::: \n", magFldIntF);

                                                      // short totalDelta = (short)GW_ntohs(buff3_p + 6);
                                                      // float totalDeltaF = totalDelta;
                                                      // totalDeltaF *= 0.1;

                                                      // printf("tot-abs-delta: %f uT \n", totalDeltaF);
                                                  }
                                                  else 
                                                  {
                                                      // Serial.printf("\n tlvLen3 : %d  \r\n",  tlvLen3);
                                                  }
                                              }
                                              else
                                              {
                                                  if (snsrId == PLTFRM_DUAL_FS_LVL_MON_1_DEV_ID)
                                                  {
                                                      if (tlvLen3 == 1)
                                                      {
                                                          unsigned char byte = *buff3_p;
                                                          unsigned char alertType;
                                                          unsigned char upperSw;
                                                          unsigned char lowerSw;
  
                                                          // printf("\n 0x%02x \n", byte);
                                                          
                                                          alertType = byte & 0x3;
                                                          byte >>= 2;
                                                          lowerSw =  byte & 0x3;
                                                          byte >>= 2;
                                                          upperSw =  byte & 0x3;

                                                          // printf("\n 0x%02x \n", byte);

                                                          Serial.printf("Upper-Sw<%s> Lower-Sw<%s> Alert<%s> \r\n",
                                                                 upperSw == 1 ?  "Open" :  "Closed",             
                                                                 lowerSw == 1 ?  "Open" :  "Closed",      
                                                                 alertType ==  0  ? "None" : \
                                                                 alertType ==  1  ? "High **************" : \
                                                                 alertType ==  2  ? "Low **************" : "Invalid **************");
         
                                                      }
                                                      else 
                                                          Serial.printf("\n tlvLen3 : %d  !!\r\n",  tlvLen3);
                                                                      
                                                  }
                                                  else
                                                  {
                                                      if (snsrId == PLTFRM_WPDS_DEV_ID)
                                                      {                                
                                                          if (tlvLen3 >= (1 + 2 + 1 + 3))
                                                          {
                                                              int channId, battV, rssi;

                                                              channId = *(buff3_p ++);
                                                              battV = GW_ntohs(buff3_p);
                                                              buff3_p += 2;
                                                              rssi = (int)(*((char *)(buff3_p ++)));
                                                              // Serial.printf("<Sender (0x%02x:0x%02x:0x%02x) ",
                                                              //               buff3_p[0], buff3_p[1],  buff3_p[2]);
                                                              // Serial.printf("/ RSSI(%d) / BattV(%d mV) / Channel(%d)> \r\n",
                                                              //               rssi, battV, channId + 1);
                                                          }
                                                          else
                                                              Serial.printf("Malformed event !! \r\n");
                                                      }    
                                                      else                              
                                                      {                                
                                                          if (snsrId == PLTFRM_AUTO_ASSY_TMON_1_DEV_ID)
                                                          {
                                                              int idx;
                                                              for (idx=0; idx<2; idx++)
                                                              {
                                                                   // Serial.printf("Socket [%d] <%s> \r\n", 
                                                                   //     idx + 1, (snsrOp & (1 << idx)) ? "Empty" : "Plugged-In");
                                                              }
                                                          }
                                                          else
                                                          {
                                                                if (snsrId == PLTFRM_RTD_1000_1_DEV_ID)
                                                                {
                                                                 // if (tlvLen3 == 4)
                                                                  //{
                                                                     // double tempVal = RTD_1000_processADCVal(snsrOp);          //add if any rtd related modification in EventBuffer
                                                                      //val=tempVal;
#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                                                                //      GW_appendIntToEventBufferJson(extAddr_p, snsrId, "", val);
#endif

                                                                      //printf("<%f %s> \n", tempVal, unit_p);
                                                                  //}
                                                              }
                                                              
                                                             
                                                              if (snsrId == PLTFRM_GPIO_REMOTE_CTRL_DEV_ID)
                                                              {
                                                                  if (tlvLen3 == 3)
                                                                  {
                                                                      int val = ((*(buff3_p + 0)) + 1)*100;
                                                                      val += ((*(buff3_p + 1))*10);
                                                                      val += (*(buff3_p + 2));


#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
                                                                      GW_appendIntToEventBufferJson(extAddr_p, snsrId, "", val); 
#endif

                                                                  }
                                                              }
                                                              else
                                                              {
#if 0                                                              
                                                                  if (strlen(unit_p) > 0)  
                                                                      Serial. printf(" <%d %s> \r\n", snsrOp, unit_p);
                                                                  else
                                                                      Serial.printf(" <%d> \r\n", snsrOp);
#endif
                                                              }
                                                          }
                                                      }    
                                                  }
                                              }
                                          }   
                                      }
                                  }
                              }
                          }
                      }


                  }
              }  // SENSOR_OP TLV found 
          }  // while (1)
       }
   }

#ifdef EVENT_FMT_TYPE_MERGED_DATA_JSON
    node_process_sts = 1;
    //GW_sendMergedDataEvtToCloudJson();
    merge_count=0;
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
              Serial.printf("\nProcess Node msg...\n");
              COORD_IF_procNodeMsg(COORD_INTF_taskRxMsgBuff + COORD_INTF_FRAME_HDR_LEN,
                                    COORD_INTF_rcvdFramePyldLen);
            //  xQueueSend(Queue2,(void*)&COORD_INTF_taskRxMsgBuff,(TickType_t)5);
           
            
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
       // Header not received already
       if (currMsgBytesTotalRcvdCnt >= COORD_INTF_FRAME_HDR_LEN)
       {
           int toCopy = COORD_INTF_FRAME_HDR_LEN - COORD_INTF_rxMsgOff;

           Serial.printf("POM tC:%d \n", toCopy);

           // Verify if valid header received
           memcpy(COORD_INTF_taskRxMsgBuff + COORD_INTF_rxMsgOff,
                  rawBuff_p, toCopy);
           consumedCnt += toCopy;

           COORD_INTF_rxMsgOff += toCopy;

           if (COORD_INTF_checkForHdr(COORD_INTF_taskRxMsgBuff) == true)
           {
               if (COORD_INTF_rcvdFramePyldLen == 0)
               {
                   // Msg with no payload
                   COORD_INTF_procRcvdFrame();

                   // Reset offsets
                   COORD_INTF_rcvdFramePyldLen = 0;
                   COORD_INTF_rxMsgOff = 0;
               }
           }
           else
           {
               // Bad header - drop all bytes received !!    
               consumedCnt = -1;

               // Reset offsets
               COORD_INTF_rcvdFramePyldLen = 0;
               COORD_INTF_rxMsgOff = 0;
           }
       }
       else
       {
           // Full header not received yet
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
           
       // Header has already been received, check if payload fully received
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
                                  
           // Reset offsets
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
         // Handle received event
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
                       // <TODO> Error check readCnt
                       uart_data_send_str.readCnt_q = readCnt;
                       for(int i=0;i<readCnt;i++)
                       {
                        Serial.printf("%x ",COORD_INTF_taskRxRawBuff[i]);
                        uart_data_send_str.COORD_INTF_taskRxRawBuff_q[i] = COORD_INTF_taskRxRawBuff[i];
                       }
                      Serial.printf("\nAfter copying...\n");
                       for(int i=0;i<readCnt;i++)
                       {
                        Serial.printf("%x ",uart_data_send_str.COORD_INTF_taskRxRawBuff_q[i]);
                       }
                        Serial.printf("\n");
                        xQueueSend(Queue2,(void*)&COORD_INTF_taskRxRawBuff,(TickType_t)5);
                      // COORD_IF_procRxBytes(readCnt);
                       
                    } while (1);
                 }
                 break;

            case UART_FRAME_ERR:
                 {
                    // TODO
                 }
                 break;

            default:
                 {
                    // TODO
                 }
                 break;
         }
     }
  }
    
  return;
}


void setup() 
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);  

  // Configure serial port connecting to the coordinator 
  uart_param_config(COORD_INTF_SERIAL_PORT_NR, 
                    &CoordIntf_uartConfigInfo);

  /* 
   * esp_err_t uart_set_pin(uart_port_tuart_num, 
   *                        int tx_io_num, int rx_io_num, 
   *                        int rts_io_num, int cts_io_num);
   *               
   */
  uart_set_pin(COORD_INTF_SERIAL_PORT_NR, 
               COORD_INTF_RX_PIN, 
               COORD_INTF_TX_PIN,
               UART_PIN_NO_CHANGE, 
               UART_PIN_NO_CHANGE);
  
  uart_driver_install(COORD_INTF_SERIAL_PORT_NR, 
                      COORD_INTF_UART_DRIVER_BUFF_SIZE,   // Rx Buffer
                      COORD_INTF_UART_DRIVER_BUFF_SIZE,   // Tx Buffer
                      COORD_INTF_EVT_QUEUE_SIZE,  // event queue size 
                      &CoordIntf_evtQHndl,  // event queue handle
                      0);

  
    Queue2 = xQueueCreate(100,sizeof(uart_data_send_str));
    Queue3 = xQueueCreate(100,sizeof(COORD_INTF_taskRxRawBuff)); 
    if(Queue2 == NULL)
    {
      Serial.printf("\nQueue2 not created...\n"); 
    }
    if(Queue3 == NULL)
    {
      Serial.printf("\nQueue2 not created...\n");
    }
  //Create a task to handler UART event from ISR
  xTaskCreate(CoordIntf_uartEvtTask, "COORD_INTF_UART_ISR_ROUTINE", 4000, NULL, 12, NULL);

  xTaskCreate(CoordIntf_procCoordMsg,"COORD_INTF_PROC_COORD_MSG", 10000, NULL, 12, NULL);
  
    
  // Configure serial port connected to the GSM Modem
  GSM_serialObj.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(10);
  
#ifdef GSM_MQTT_TCP
  MQTT_client.setServer(MQTT_brokerURL, 1883);
  MQTT_client.setCallback(MQTT_callback);
  //MQTT_client.setServer("dashboard.wisense.in", 1883);
  //MQTT_client.setCallback(MQTT_callback);
#endif

  DBG("Setup Complete :-)");
}

boolean mqttConnect(char* user, char* pass)
{
    boolean status;
    status = MQTT_client.connect("GsmClient",user,pass);  
    //client.connect("GsmClient", user , pass);
    int sts = 0;
    while(!MQTT_client.connected() && sts<3)
    {
      sts++;
      status = MQTT_client.connect("GsmClient", user , pass);
    }
    return status;
}


void loop() 
{  
  if (MODEM_restarted == false)
  { 
      /*
       * modem is an object of the TinyGsm class.
       * TinyGsm.init( ) -or- TinyGsm.restart( ) can be used to configure
       * the modem.
       * "restart()" generally takes longer than init but ensures the module 
       * doesn't have lingering connections
       */

      DBG("Configuring the modem .... ");
       
      if (!modem.restart()) 
      {
          DBG("Failed to restart modem ... delaying 10s before retrying !!");

          // rkris: Assuming Modem always uses 115200 or is the modem
          //        capable of baud rate detection ? Verifiy. <TODO>
          // Restart autobaud in case GSM just rebooted
          // TinyGsmAutoBaud(GSM_serialObj, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);

          delay(10*1000); 
          //delay(1*1000);    //commented
          //add all the error checks here.
          return;
      }
      else
      {
          MODEM_restarted = true;
      }
  }
  /*
#if TINY_GSM_TEST_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) 
  {
      modem.simUnlock(GSM_PIN);
      // DBG("SimUnlock processing done...");
  }
#endif
*/

  if (modem.isNetworkConnected() == false)
  {
      DBG("Registering to the network .... ");

      // Wait for network registration to be successful
      if (!modem.waitForNetwork(600000L)) 
      {
          delay(10*1000);
          return;
      }

      if (modem.isNetworkConnected()) 
      {
          DBG("Registered to the network  :-) ");
      }
      else
      {
          DBG("Still not registered to the network ... delaying for 10 secs before retrying !!!");
          delay(10*1000);
          return;
      }
  }
  
#if TINY_GSM_TEST_GPRS
  if (modem.isGprsConnected() == false)
  {
      DBG("Connecting to", apn);
  
      /*
       * If using cellular, establish the GPRS or EPS data connection after your are 
       * successfully registered on the network.
       */
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
      {
          DBG("GPRS connection setup failed ... delaying for 10 secs before retrying !!! ");
          delay(10*1000);
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
      }
  }
#endif



#if GSM_MQTT_TCP
  /*
   * MQTT.connected( )
   * This method allows your code to quickly determine whether an mqttclient 
   * object is currently connected to an MQTT broker:
   */
#if 1
  if (!MQTT_client.connected())
  {
      DBG("Establishing MQTT connection to ", MQTT_brokerURL);
      
      /*
       * boolean connect (clientID, 
       *                  [username, password], 
       *                  [willTopic, willQoS, willRetain, willMessage], 
       *                  [cleanSession]
       *                 )
       * Returns:
       *  > false - connection failed
       *  > true - connection succeeded
       */
      boolean status = MQTT_client.connect("GsmClient","NULL","NULL");         // clientId
      if (status == false) 
      {
          DBG("Failed to setup MQTT connection !!! ");
          delay(10*1000);
          return;
      }
      else
      {
          GSM_CLOUD_CNT_STS = true;
          DBG("MQTT connection established :-) ");  
          MQTT_client.publish(MQTT_topicInit, "GsmClientTest started");    /* creating json */
      }
  }
  if(MQTT_client.connected())
  {
    GSM_CLOUD_CNT_STS = true;
  }
#endif

  //DBG("Pending Tx Cnt: ", GW_txToCloudPendCnt); //commented

  
/*
* Continously Monitoring the connect status of network and GPRS.
* if connection was still up then status will be True, payload are ready to publish to cloud
* if no, drop all the payload and try to connect with networks and provide LED indication.
*
*/

  if(modem.isNetworkConnected())
  {
      GSM_NETWRK_CNT_STS = true;
      //Serial.printf("\nModem Connected to network...\n");
      if(modem.isGprsConnected())
      {
          GSM_GPRS_CNT_STS = true;
          //Serial.printf("\nModem GPRS Connect: Connected Success...\n");
      }
      else
      {
        GSM_GPRS_CNT_STS = false;
        //Serial.printf("\nModem GPRS Connect: Not Connected..\n");
      }
  }
  else
  { 
      GSM_NETWRK_CNT_STS = false;
      //Serial.printf("\nModem not connected to the network....\n");
  }
  if(GSM_NETWRK_CNT_STS && GSM_GPRS_CNT_STS)
  {
    GSM_CNT_STS = true;
    //Serial.printf("\n Network and GPRS connection Successfully...\n");
  }



 //if (GW_txToCloudPendCnt > 0)
 // {

 //once the complete node data received...
 if(node_process_sts)        //once the data received it will be 1/true, once process completes need to make zero/false
{
  node_process_sts=0;
  Serial.printf("\nPacket Received... \n");
 /*
  if(MQTT_client.connected())
  {
    GSM_CLOUD_CNT_STS = true;
    Serial.printf("\nMQTT Connect State: Connected\n");
  }
  else
  {
    GSM_CLOUD_CNT_STS = false;
    Serial.printf("\nMQTT Connect State: Not-Connected\n");
  }
  */  
  if(GSM_CLOUD_CNT_STS == true && GSM_CNT_STS == true)    //checking both network/GPRS & cloud
  {
  //if(GSM_CNT_STS == true)
  //{
         Serial.printf("\nProceeding with JSON\n");
         Serial.printf("\n\n");
         int snsrId_count =4;
         seq_nr++;   
            /*spilting GW_snsrDataBuff into json file
              GW_snsrDataBuff_0=Node_voltage,  GW_snsrDataBuff_1=msp_temp, GW_snsrDataBuff_2=R_Humidity, GW_snsrDataBuff_3=R_TEMP, GW_snsrDataBuff_4= *** 
            */
         char GW_jsonMsg[1024];
#if 1
          // Arduinojson 5
          StaticJsonBuffer<1024> jsonBuffer;
          JsonObject& root = jsonBuffer.createObject();
#else
//      Arduinojson 6
//      DynamicJsonDocument jsonDoc(1024); 
//      JsonObject root = jsonDoc.to<JsonObject>();
#endif
          
            GW_snsrDataBuff[0] = '\0';
            GW_snsrDataBuff_0[0] = '\0';
            GW_snsrDataBuff_1[0] = '\0';
            GW_snsrDataBuff_2[0] = '\0';
            GW_snsrDataBuff_3[0] = '\0';
            GW_snsrDataBuff_4[0] = '\0';
            GW_snsrDataBuff_5[0] = '\0';
            GW_snsrDataBuff_6[0] = '\0';
            GW_snsrDataBuff_7[0] = '\0';
            GW_snsrDataBuff_8[0] = '\0';
           
            sprintf(GW_snsrDataBuff_0, "%.2f",snsrDataBuff[0]);         //changed to 2 decimal points        
            sprintf(GW_snsrDataBuff_1, "%.2f",snsrDataBuff[1]);
            sprintf(GW_snsrDataBuff_2, "%.2f",snsrDataBuff[2]);
            sprintf(GW_snsrDataBuff_3, "%.2f",snsrDataBuff[3]);     
            sprintf(GW_snsrDataBuff_4, "%.2f",snsrDataBuff[4]);
            sprintf(GW_snsrDataBuff_5, "%.2f",snsrDataBuff[5]);            
            sprintf(GW_snsrDataBuff_6, "%.2f",snsrDataBuff[6]);
            sprintf(GW_snsrDataBuff_7, "%.2f",snsrDataBuff[7]);
            sprintf(GW_snsrDataBuff_8, "%.2f",snsrDataBuff[8]);
            for(int i=0;i<snsrId_count;i++)
            {
                if(snsrIdStore[i] == 0x8)
                {
                    snsrIdStore[i] = 0x80;
                }
            }
            sprintf(snsrIdStore8,"0x%x",snsrIdStore[8]);
            sprintf(snsrIdStore7,"0x%x",snsrIdStore[7]);
            sprintf(snsrIdStore6,"0x%x",snsrIdStore[6]);
            sprintf(snsrIdStore5,"0x%x",snsrIdStore[5]);
            sprintf(snsrIdStore4,"0x%x",snsrIdStore[4]);    
            sprintf(snsrIdStore3,"0x%x",snsrIdStore[3]);
            sprintf(snsrIdStore2,"0x%x",snsrIdStore[2]);
            sprintf(snsrIdStore1,"0x%x",snsrIdStore[1]);
            sprintf(snsrIdStore0,"0x%x",snsrIdStore[0]);
            
            //mapping 
            unsigned int sm_lvl,sm_lvl_lct;
            if(snsrIdStore[0] == 0x74 || snsrIdStore[1] == 0x74 || snsrIdStore[2] == 0x74 || snsrIdStore[3] == 0x74 || snsrIdStore[4] == 0x74 || snsrIdStore[5] == 0x74 )
            {
               if(snsrIdStore[0] == 0x74)
                    sm_lvl_lct = 0;
                if(snsrIdStore[1] == 0x74)
                    sm_lvl_lct = 1;  
                if(snsrIdStore[2] == 0x74)
                    sm_lvl_lct = 2;
                if(snsrIdStore[3] == 0x74)
                    sm_lvl_lct = 3;
                if(snsrIdStore[4] == 0x74)
                    sm_lvl_lct = 4;
                if(snsrIdStore[5] == 0x74)
                    sm_lvl_lct = 5; 
                sm_lvl = snsrDataBuff[sm_lvl_lct]; 
                sm_lvl = map(sm_lvl, 0, 3000, 0, 100);    //adc value range from 0 to 3000   
            }  
            root["Name"] = GW_devName;
            root["MacId"] = GW_macBuff;
            root["RSSI"] = node_rssi;
            root["LQI"] = node_lqi;
            root[snsrIdStore0] = GW_snsrDataBuff_0;     
            root[snsrIdStore1] = GW_snsrDataBuff_1;
        if(snsrIdStore[2] != 0x0)                                           //condition; if its zero it wont store in json  
        {
            root[snsrIdStore2] = GW_snsrDataBuff_2;
        }
        if(snsrIdStore[3] != 0x0)
        {
            root[snsrIdStore3] = GW_snsrDataBuff_3;
        }
        if(snsrIdStore[4] != 0x0 && GW_snsrDataBuff_4 != 0)
        {
            root[snsrIdStore4] = GW_snsrDataBuff_4;
        }
        if(snsrIdStore[5] != 0x0 && GW_snsrDataBuff_5 != 0)
        {
            root[snsrIdStore5] = GW_snsrDataBuff_5;
        }
        if(snsrIdStore[6] != 0x0 && GW_snsrDataBuff_6 != 0)
        {
            root[snsrIdStore6] = GW_snsrDataBuff_6;
        }
        if(snsrIdStore[7] != 0x0 && GW_snsrDataBuff_7 != 0)
        {
            root[snsrIdStore7] = GW_snsrDataBuff_7;
        }
        if(snsrIdStore[8] != 0x0 && GW_snsrDataBuff_8 != 0)
        {
            root[snsrIdStore8] = GW_snsrDataBuff_8;
        }
        if(snsrIdStore[0] == 0x74 || snsrIdStore[1] == 0x74 || snsrIdStore[2] == 0x74 || snsrIdStore[3] == 0x74 || snsrIdStore[4] == 0x74 || snsrIdStore[5] == 0x74 )
        {
            root["Moisture Level"] = sm_lvl;
        }
            root["Sequence_number"] = seq_nr;                               // number indicates number of data's pushed to the events, whenever the system restarts, this also start from Zero
            root["SensorData Date&time"] = GW_get_time;
            root.printTo(GW_jsonMsg);           
            
            //sprintf(GW_eventBuff, "SENSOR_MERGED_DATA_EVT");
            //Serial.printf(GW_jsonMsg);
            Serial.printf("\nComplete JSON MSG: ");
            Serial.printf(GW_jsonMsg);
            Serial.printf("\n");

      
#if 1
      // ArduinoJson 5
      root.printTo(GW_jsonMsg);
#else     
      /*
       * With ArduinoJson 6, you call the function serializeJson() and pass 
       * the JsonArray, JsonObject, or the JsonDocument.
       */
      serializeJson(jsonDoc, GW_jsonMsg);
#endif
      /*
       * publish( ) returns
       *   > false - publish failed, either connection lost or message too large
       *   > true - publish succeeded
       *   
       *   publish( ) is obviously a blocking call.
       */
//common USN,PWD start

          //boolean status = mqttConnect("NULL","NULL");
          if(MQTT_client.connected())
          {
            GSM_CLOUD_CNT_STS = true;
            Serial.printf("\nMQTT Connect State: Connected\n");
          }
          else
          {
            GSM_CLOUD_CNT_STS = false;
            Serial.printf("\nMQTT Connect State: Not-Connected\n");
          }
             boolean rc = MQTT_client.publish("v1/devices/me/telemetry",GW_jsonMsg);
             MQTT_stats.pubCnt ++;
            if (rc == false)
            {
                MQTT_stats.pubFlrCnt ++;
                DBG("MQTT publish failed !!!, cnt: ", MQTT_stats.pubFlrCnt);
                if (MQTT_client.connected())
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
              Serial.printf("\n**********************JSON Publish Success****************\n");
             }       
             Serial.printf("\n\n\n");   
             //MQTT_client.disconnect();

// common USN,PWD end       

#if 0
      if(strcmp(GW_macBuff, "fe:0d:4a:46") == 0)
      {
        Serial.printf("\n1st sensor\n");
          boolean status = mqttConnect("p0lsPBWqHD2AWhmKUlEu","NULL");
          if(MQTT_client.connected())
          {
            GSM_CLOUD_CNT_STS = true;
            Serial.printf("\nMQTT Connect State: Connected\n");
          }
          else
          {
            GSM_CLOUD_CNT_STS = false;
            Serial.printf("\nMQTT Connect State: Not-Connected\n");
          }
          /*
          if(status == false)
          {
            Serial.printf("\nMQTT Connect: failed...\n");
          }
          else
          {
            Serial.printf("\nMQTT Connect: Success...\n");  
          }
          */


             boolean rc = MQTT_client.publish("v1/devices/me/telemetry",GW_jsonMsg);
             MQTT_stats.pubCnt ++;
            if (rc == false)
            {
                MQTT_stats.pubFlrCnt ++;
                DBG("MQTT publish failed !!!, cnt: ", MQTT_stats.pubFlrCnt);
                if (MQTT_client.connected())
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
              Serial.printf("\n**********************JSON Publish Success****************\n");
             }       
             Serial.printf("\n\n\n");   
             MQTT_client.disconnect();
        }

        //2nd sensor
         if(strcmp(GW_macBuff, "fe:0d:5f:7c") == 0)
        {
          Serial.printf("\n2nd sensor <%s>\n",GW_macBuff);
          boolean status = mqttConnect("st8T3pTKQ2LUBWBhCrzk","NULL");
          if(MQTT_client.connected())
          {
            GSM_CLOUD_CNT_STS = true;
            Serial.printf("\nMQTT Connect State: Connected\n");
          }
          else
          {
            GSM_CLOUD_CNT_STS = false;
            Serial.printf("\nMQTT Connect State: Not-Connected\n");
          }
          /*
          if(status == false)
          {
            Serial.printf("\nMQTT Connect: failed...\n");
          }
          else
          {
            Serial.printf("\nMQTT Connect: Success...\n");  
          }
          */

             boolean rc = MQTT_client.publish("v1/devices/me/telemetry",GW_jsonMsg);
             MQTT_stats.pubCnt ++;
            if (rc == false)
            {
                MQTT_stats.pubFlrCnt ++;
                DBG("MQTT publish failed !!!, cnt: ", MQTT_stats.pubFlrCnt);
                if (MQTT_client.connected())
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
              Serial.printf("\n**********************JSON Publish Success****************\n");
             }       
             Serial.printf("\n\n\n");   
             MQTT_client.disconnect();
        }
#endif



}
else
{
  Serial.printf("\ndrooping payload \n");
  if(GSM_CNT_STS == false)
  {
    Serial.printf("\nGSM Network/gprs connection not established..\n");
  }
  if(GSM_CLOUD_CNT_STS == false)
  {
    Serial.printf("\nCloud/mqtt connection not established...\n");
  }
}

            int i;
            for(i=0;i<=8;i++)
            {
              snsrIdStore[i] = 0x0;
            }
   
            /*
            snsrIdStore[0] = 0x0;
            snsrIdStore[1] = 0x0;
            snsrIdStore[2] = 0x0;
            snsrIdStore[3] = 0x0;
            snsrIdStore[4] = 0x0;
            snsrIdStore[5] = 0x0;
            snsrIdStore[6] = 0x0;
            snsrIdStore[7] = 0x0;
            snsrIdStore[8] = 0x0;
            */
            /*
            snsrDataBuff[0] = {0};
            snsrDataBuff[1] = {0};
            snsrDataBuff[2] = {0};
            snsrDataBuff[3] = {0};
            snsrDataBuff[4] = {0};
            */
            
          
   GW_snsrDataBuff[0] = '\0';
   GW_snsrDataBuff_0[0] = '\0';
   GW_snsrDataBuff_1[0] = '\0';
   GW_snsrDataBuff_2[0] = '\0';
   GW_snsrDataBuff_3[0] = '\0';
   GW_snsrDataBuff_4[0] = '\0';
            merge_count=0;                                          //clearing merge count to set to start first          
            node_process_sts= 0;  
}
 

#endif

  //Serial.printf("\nLoop running forever....\n");
  //DBG("Delaying for 3 second .... ");
  //delay(1*1000);
  return;
}
