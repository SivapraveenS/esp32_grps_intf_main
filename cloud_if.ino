/*
 *********************************************************************
 * File: cloud_if.ino
 * 
 * Author: rkris@wisense.in / siva@wisense.in
 * 
 * Date: March/2021
 * 
 * Copyright(C) WiSense Technologies Pvt Ltd 
 * All rights reserved.
 * *******************************************************************
 */



/*
   The PubSubClient library provides a client for doing simple
   publish/subscribe messaging with a server that supports MQTT.
*/
struct __CLOUD_IF_mqttSession__ 
{
   uint8_t valid;
   uint8_t extAddr[DIS_LPWMN_MAC_EXT_ADDR_LEN];
   uint32_t pubCnt;
   uint32_t pubFlrCnt;
   uint32_t pubOkCnt;
   uint32_t pubFlrConnDownCnt;
   uint32_t pubFlrConnUpCnt;
   uint32_t connReqFlrCnt;
   uint32_t connReqOkCnt;
   char userName[64];
   char password[64];
} CLOUD_IF_mqttSession_s;

#define CLOUD_IF_MAX_MQTT_SESSIONS_CNT   8

struct __CLOUD_IF_mqttSession__ CLOUD_IF_mqttSessionList[CLOUD_IF_MAX_MQTT_SESSIONS_CNT];


char attrName[10],
     shortAddrCpy[10], 
     attrIdCpy[10], 
     attrValCpy[10];


static char CLOUD_IF_tempRcvdMQTTMsgBuff[Q4_MSG_Q_ENTRY_SZ];
static char CLOUD_IF_wsnMsgSerJsonFmt[320];
static char CLOUD_IF_jsonMsgFromWSN[320]; 

const char* MQTT_brokerURL = "broker.hivemq.com";
const char* MQTT_wsnDashBoardBrokerURL = "dashboard.wisense.in";


struct __CLOUD_IF_mqttSession__ *CLOUD_IF_getMQTTSession(uint8_t *extAddr_p)
{
   int idx = 0;
   struct __CLOUD_IF_mqttSession__ *mqttSession_p = CLOUD_IF_mqttSessionList;   

   for (; idx<CLOUD_IF_MAX_MQTT_SESSIONS_CNT; idx++)
   {
       if (memcmp(extAddr_p, mqttSession_p->extAddr, DIS_LPWMN_MAC_EXT_ADDR_LEN) == 0)
       {
           return mqttSession_p;
       }

       mqttSession_p ++;
   }     

   return NULL;
}


/*
 * API document; https://pubsubclient.knolleary.net/api#connect
 * 
 * ----------------------------------------------------------------------------------------
 * PubSubClient.connect( ) API
 * ----------------------------------------------------------------------------------------
 * boolean connect (clientID, [username, password])
 * Params
 *  > clientID const char[] - the client ID to use when connecting to the server
 *  > username const char[] - the username to use. If NULL, no username or password is used
 *  > password const char[] - the password to use. If NULL, no password is used
 * 
 * Returns
 *  > false - connection failed
 *  > true - connection succeeded
 * ----------------------------------------------------------------------------------------
 * 
 *  
 * ----------------------------------------------------------------------------------------
 * PubSubClient.publish( ) API
 * ----------------------------------------------------------------------------------------
 * boolean publish (topic, payload, [length], [retained])
 * Params
 *  > topic const char[] - the topic to publish to
 *  > payload const char[], byte[] - the message to publish
 *  > length unsigned int (optional) - the length of the payload. Required if payload is a byte[]
 *  > retained boolean (optional) - whether the message should be retained
 *  
 * 
 * Returns
 *  > false - publish failed, either connection lost or message too large
 *  > true - publish succeeded
 * ----------------------------------------------------------------------------------------
 * 
 */

char NODE_macIdCpy[13];
void CLOUD_IF_sendPendMsgToCloud(void)
{
   static unsigned int CLOUD_IF_nodeMQTTConnReqFlrCnt = 0,
                       CLOUD_IF_nodeMQTTConnReqOkCnt = 0,
                       CLOUD_IF_coordRespMsgDQCnt = 0,
                       CLOUD_IF_mqttPubOkCnt = 0,
                       CLOUD_IF_mqttPubFlrCnt = 0;
   /*
    *  Setting xTicksToWait to 0 will cause the function to return immediately 
    *  if the queue is empty
    */

   // Serial.printf("CLI_sPMTC> E \n");
    
   if (xQueueReceive(Q6Hndl, (void *)CLOUD_IF_jsonMsgFromWSN, (portTickType)0))
   { 
       CLOUD_IF_coordRespMsgDQCnt ++;
       Serial.printf("CLI_sPMTC> CR M #%u \n", CLOUD_IF_coordRespMsgDQCnt);
      
       if (!CLOUD_IF_respToCloudMQTTClient.connected())
       {
           static uint32_t CLOUD_IF_respToCloudMQTTConnCnt = 0,
                           CLOUD_IF_respToCloudMQTTConnFlrCnt = 0,
                           CLOUD_IF_respToCloudMQTTConnOkCnt = 0;
           boolean status;

           CLOUD_IF_respToCloudMQTTConnCnt ++;
           
           Serial.printf("CLI_sPMTC> CResp MQC CR To Brkr #%u\n", CLOUD_IF_respToCloudMQTTConnCnt);
           
           status = CLOUD_IF_respToCloudMQTTClient.connect("WSN_GW_respToCloudMQTTClient", 
                                                          "BE8RpfrsvVNaumB2o32V", 
                                                          NULL);
           if (status == false)
           {
               CLOUD_IF_respToCloudMQTTConnFlrCnt ++;   
               Serial.printf("CLI_sPMTC> CRsp MQC CR Fld #%u !!\n", CLOUD_IF_respToCloudMQTTConnFlrCnt);
           }
           else
           { 
               CLOUD_IF_respToCloudMQTTConnOkCnt ++;
               Serial.printf("CLI_sPMTC> CRsp MQC CR OK #%u\n", CLOUD_IF_respToCloudMQTTConnOkCnt);
           }
       }

       if (CLOUD_IF_respToCloudMQTTClient.connected())
       {
           boolean rc;
          
           CLOUD_IF_wsnMsgJSONObj["put_attr"] = CLOUD_IF_jsonMsgFromWSN;      
           serializeJson(CLOUD_IF_wsnMsgJSONDoc, CLOUD_IF_wsnMsgSerJsonFmt);

           Serial.printf("\n\nCLI_sPMTC> MTC: ");
           Serial.printf(CLOUD_IF_wsnMsgSerJsonFmt);
           Serial.printf("\n\n");       
             
           rc = CLOUD_IF_respToCloudMQTTClient.publish("v1/devices/me/telemetry", 
                                                      CLOUD_IF_wsnMsgSerJsonFmt);
           if (rc == true)
           {
               CLOUD_IF_mqttPubOkCnt ++;
               Serial.printf("CLI_sPMTC> CRsp MQTT Pub OK #%u\n", CLOUD_IF_mqttPubOkCnt);
           }
           else
           {
               CLOUD_IF_mqttPubFlrCnt ++;
               Serial.printf("CLI_sPMTC> CRsp MQTT Pub F #%u !!\n", CLOUD_IF_mqttPubFlrCnt);
           }
       }
   }

   if (xQueueReceive(Q3Hndl, (void *)CLOUD_IF_jsonMsgFromWSN, (portTickType)0))
   {
       char *rcvdJsonMsg_p;
       uint8_t *eA_p = (uint8_t *)CLOUD_IF_jsonMsgFromWSN;
       int jsonMsgLen;
       struct __CLOUD_IF_mqttSession__ *mqttSess_p = NULL;
      
       // The received message format is:
       //   8 bytes of extended MAC address
       //   + seralized JSON message
       //   + NULL character
      
       rcvdJsonMsg_p = (char *)(eA_p + DIS_LPWMN_MAC_EXT_ADDR_LEN);
       jsonMsgLen = strlen(rcvdJsonMsg_p) + 1;
       
       Serial.printf("CLI_sPMTC> Rx M L<%d> Q3 %02x:%02x:%02x:%02x\n", 
                     jsonMsgLen, eA_p[4], eA_p[5], eA_p[6], eA_p[7]);
                            
       mqttSess_p = CLOUD_IF_getMQTTSession(eA_p);
       
       if (mqttSess_p == NULL)
       {
           // No MQTT session found corresponding to the MAC address in the received 
           // message !!. Drop this message.

           MQTT_stats.sessNotFndCnt ++;
           Serial.printf("CLI_sPMTC> M S N Not Fnd [%u] !! \n", MQTT_stats.sessNotFndCnt);

           /*
            * msg format Eg: fc:c2:3d:ff:fe:d:5f:7c
            */
            
           sprintf(NODE_macIdCpy, "%x:%x:%x:%x:%x:%x:%x:%x", eA_p[0], eA_p[1], eA_p[2], eA_p[3], eA_p[4], eA_p[5], eA_p[6], eA_p[7]);

           Serial.printf("CLI_sPMTC> Non-Registered Node Entry: <%s> !! \n", NODE_macIdCpy);
           boolean nodeAccPubSts = CLOUD_IF_reqFromCloudMQTTClient.publish("wsn_get_node_access", NODE_macIdCpy);             

           if(nodeAccPubSts == true)
           {
              Serial.printf("CLI_sPMTC> CLOUD_IF_reqFromCloudMQTTClient Publish succcess !! \n");
           }
           else
           {
              Serial.printf("CLI_sPMTC> CLOUD_IF_reqFromCloudMQTTClient publish failed !! \n");
           }
       }
       else
       {
           PubSubClient *mqtt_p = &CLOUD_IF_nodesMQTTClient;

           Serial.printf("CLI_sPMTC> M S N Fnd \n");

           // Connection should always be disconnected
           if (mqtt_p->connected()) 
           {
               Serial.printf("CLI_sPMTC> M S C - D C \n");
               mqtt_p->disconnect();
           }
           
           if (1)
           {
               Serial.printf("CLI_sPMTC> C N MQ U<%s> \n", mqttSess_p->userName);
            
               boolean status = mqtt_p->connect("WiSense_LPWMN_MSG",   // ClientId 
                                                mqttSess_p->userName,  // "BE8RpfrsvVNaumB2o32V", 
                                                NULL);  // No password !!
               if (status == false)
               {
                   CLOUD_IF_nodeMQTTConnReqFlrCnt ++;   
                   mqttSess_p->connReqFlrCnt ++;
                   Serial.printf("CLI_sPMTC> MQ C R Flr #%u !!\n", CLOUD_IF_nodeMQTTConnReqFlrCnt);
               }
               else
               { 
                   CLOUD_IF_nodeMQTTConnReqOkCnt ++;
                   mqttSess_p->connReqOkCnt ++;
                   Serial.printf("CLI_sPMTC> MQ C R OK #%u !!\n", CLOUD_IF_nodeMQTTConnReqOkCnt);
               }
           }
                    
           if (mqtt_p->connected())
           {
               boolean rc;

               // MQTT connection is up, publish the message to the broker
               
               rc = mqtt_p->publish("v1/devices/me/telemetry", rcvdJsonMsg_p);
               mqttSess_p->pubCnt ++;
               MQTT_stats.pubCnt ++;
               
               if (rc == false)
               {
                   MQTT_stats.pubFlrCnt ++;
                   mqttSess_p->pubFlrCnt ++;
                   Serial.printf("MQTT P Flr !! #%u\n", MQTT_stats.pubFlrCnt);
                   if (mqtt_p->connected())
                   {
                       MQTT_stats.pubFlrConnUpCnt ++;
                       mqttSess_p->pubFlrConnUpCnt ++;
                       Serial.printf("MQTT P Flr Conn Up !!, #%u\n", MQTT_stats.pubFlrConnUpCnt);
                   }
                   else
                   {
                       MQTT_stats.pubFlrConnDownCnt ++;
                       mqttSess_p->pubFlrConnDownCnt;
                       Serial.printf("MQTT publish failed and MQTT conn is down");            
                   }
               }
               else
               {
                   MQTT_stats.pubOkCnt ++;
                   mqttSess_p->pubOkCnt ++;
                   Serial.printf("MQTT P OK #%u\n\n\n", MQTT_stats.pubOkCnt);
               }
           }

           // PubSubClient.disconnect() API
           // Disconnects the client.
           mqtt_p->disconnect();
       }
   }
}


void MQTT_respToCloudCallBackFn(char* topic_p, byte* payload_p, unsigned int len)
{
  
}


/*
 * Handle requests from WiSense dashboard in the cloud
 */
void MQTT_reqFromCloudCallBackFn(char* topic_p, byte* payload_p, unsigned int len)
{
   static uint32_t CLOUD_IF_qToQ4FlrCnt = 0,
                   CLOUD_IF_qToQ4DoneCnt = 0;
   byte* mqttMsg_p = (byte *)CLOUD_IF_tempRcvdMQTTMsgBuff;

   // This function is called in the context of the thread which
   // calls this function CLOUD_IF_respToCloudMQTTClient.loop( ) which
   // in our case is the main thread which runs the main loop.
   
   Serial.printf("MTT C CB FN> T:%s L:%u\n", topic_p, len);
   Serial.printf("MTT C CB FN> Pyld: ");
   Serial.write(payload_p, len);
   Serial.println();
   
   memcpy(mqttMsg_p, payload_p, len);

   if(String(topic_p) == "get_attr")
   {
           // Q4 queue entry size is 320
      if (xQueueSend(Q4Hndl, (void*)mqttMsg_p, (TickType_t)0) == pdPASS)
      {
          CLOUD_IF_qToQ4DoneCnt ++;
          Serial.printf("MTT C CB FN> #%u -> Q4 OK \n",
                             CLOUD_IF_qToQ4DoneCnt);       
      }
      else
      {
          CLOUD_IF_qToQ4FlrCnt ++;
          Serial.printf("MTT C CB FN> #%u -> Q4 Flr ||\n", 
                             CLOUD_IF_qToQ4FlrCnt);
      }
  }
  
  if(String(topic_p) == "wsn_node_access_response")
  {
      if (xQueueSend(Q7Hndl, (void*)mqttMsg_p, (TickType_t)0) == pdPASS)
      {
          Serial.printf("MTT C CB FN> WSN_nodeAcsResp Passed into Q7 \n");
      }
      else
      {
          Serial.printf("MTT C CB FN> No Queue space in Q7 !! \n");
      }
  }
   return;
}


void MQTT_loop(void)
{
   /* 
    * PubSubClient.loop() API
    * boolean loop ()
    * Params
    *  Description: This should be called regularly to allow the client to process 
    *               incoming messages and maintain its connection to the server.
    *
    * Returns
    *  > false - the client is no longer connected
    *  > true - the client is still connected\
    */

   // We do not have to worry about CLOUD_IF_nodesMQTTClient since it is disconnected
   // after each message is sent.
  
   CLOUD_IF_respToCloudMQTTClient.loop();

   CLOUD_IF_reqFromCloudMQTTClient.loop();
}


void MQTT_allocNewSession(const uint8_t *extAddr_p,
                          char *userName_p,
                          char *password_p)
{
   int idx = 0;
   struct __CLOUD_IF_mqttSession__ *mqttSession_p = CLOUD_IF_mqttSessionList;   

   for (; idx<CLOUD_IF_MAX_MQTT_SESSIONS_CNT; idx++)
   {
        if (mqttSession_p->valid == 0)
            break;
        mqttSession_p ++;
   }

   Serial.printf("MaNS> idx:%d  \n", idx);

   if (idx < CLOUD_IF_MAX_MQTT_SESSIONS_CNT)
   {
       // Unused entry found              
       memcpy(mqttSession_p->extAddr, extAddr_p, DIS_LPWMN_MAC_EXT_ADDR_LEN);
       
       strncpy(mqttSession_p->userName, userName_p, sizeof(mqttSession_p->userName));
       
       if (password_p != NULL)
           strncpy(mqttSession_p->password, password_p, sizeof(mqttSession_p->password));

       mqttSession_p->valid = 1;
   }
}


void CLOUD_IF_reqFromCloudMQTTConn(void)
{
   static uint32_t CLOUD_IF_reqFromCloudMQTTConnOkCnt = 0,
                   CLOUD_IF_reqFromCloudMQTTConnFlrCnt = 0,
                   CLOUD_IF_getAttrSubReqFlrCnt = 0,
                   CLOUD_IF_getAttrSubReqOkCnt = 0;
                   
   if (!(CLOUD_IF_reqFromCloudMQTTClient.connected()))
   {
       boolean status;
       
       Serial.printf("C_I_cCRMC> C_I_coReqMC N C !! \n");
       
       status = CLOUD_IF_reqFromCloudMQTTClient.connect("WSN_GW_reqFromCloudMQTTClient", "wisense", "Wisense@123");
       if (status == false)
       {
           CLOUD_IF_reqFromCloudMQTTConnFlrCnt ++;   
           Serial.printf("CLI_sPMTC> CRsp MQC CR Fld #%u !!\n", CLOUD_IF_reqFromCloudMQTTConnFlrCnt);
       }
       else
       { 
           boolean subReqSts;
           
           CLOUD_IF_reqFromCloudMQTTConnOkCnt ++;
           Serial.printf("CLI_sPMTC> CRsp MQC CR OK #%u\n", CLOUD_IF_reqFromCloudMQTTConnOkCnt);

           subReqSts = CLOUD_IF_reqFromCloudMQTTClient.subscribe("get_attr");
           if (subReqSts == false)
           {
               CLOUD_IF_getAttrSubReqFlrCnt ++;
               Serial.printf("CLI_sPMTC> sub g_a fl - #%u D-C C_I_cReqMC !!\n", CLOUD_IF_getAttrSubReqFlrCnt);
               CLOUD_IF_reqFromCloudMQTTClient.disconnect();
           }
           else
           {
               CLOUD_IF_getAttrSubReqOkCnt = 0;
               Serial.printf("CLI_sPMTC> sub g_a ok - #%u\n", CLOUD_IF_getAttrSubReqOkCnt);
           }
           
           boolean subReqSts_2;
           subReqSts_2 = CLOUD_IF_reqFromCloudMQTTClient.subscribe("wsn_node_access_response");
           if(subReqSts_2 == false)
           {
              Serial.printf("\nwsn_node_access_response subscribe failed !! \n"); 
           }
           else
           {
              Serial.printf("\nwsn_node_access_reponse subscribe success !! \n");
           }
       }        
   }
}


void MQTT_init(void)
{
   int idx = 0;
   struct __CLOUD_IF_mqttSession__ *mqttSession_p = CLOUD_IF_mqttSessionList;   

   // "broker.hivemq.com" is an external Public broker
   // The "CLOUD_IF_respToCloudMQTTClient" is used to receive commands from the 
   // cloud resident wisense server. These commands can be for the coordinator
   // or for any attached node. There is only one such client.
   
   CLOUD_IF_reqFromCloudMQTTClient.setServer("3.21.126.253", 1883);
   CLOUD_IF_reqFromCloudMQTTClient.setCallback(MQTT_reqFromCloudCallBackFn);
   
   CLOUD_IF_respToCloudMQTTClient.setServer(MQTT_brokerURL, 1883);
   CLOUD_IF_respToCloudMQTTClient.setCallback(MQTT_respToCloudCallBackFn);

   // This MQTT client is used only to send messages (received from the nodes)
   // to the cloud. The broker is in the wisense server and each dashboard (one
   // per node) is a client of this broker.
   CLOUD_IF_nodesMQTTClient.setServer(MQTT_wsnDashBoardBrokerURL, 1883);

   for (; idx<CLOUD_IF_MAX_MQTT_SESSIONS_CNT; idx++)
   {
       memset(mqttSession_p, 0, sizeof(struct __CLOUD_IF_mqttSession__));
   }  

   return;
}
