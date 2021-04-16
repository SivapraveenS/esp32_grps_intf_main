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
struct __GW_nodeCntxt__ 
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
} GW_nodeCntxt_s;

struct __GW_nodesPendCredsCntxt__
{
   uint8_t valid;
   uint8_t respPending;
   uint8_t commAttempts;
   uint8_t extAddr[DIS_LPWMN_MAC_EXT_ADDR_LEN];
   uint32_t tickStamp;
} GW_nodesPendCredsCntxt_s;

struct __GW_blackListedNode__
{
   uint8_t valid;
   uint8_t extAddr[DIS_LPWMN_MAC_EXT_ADDR_LEN];  
} GW_blackListedNode_s;

#define GW_MAX_SUPPORTED_NODE_CNT  8
#define GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT  8
#define GW_MAX_BLACK_LISTED_NODES_LIST_ENTRY_CNT  8

#define CLOUD_IF_MAX_CLOUD_RESP_WAIT_TIME  60000   // 60 secs
#define CLOUD_IF_MAX_CLOUD_COMM_ATTEMPTS  3

struct __GW_nodeCntxt__  GW_nodeCntxtList[GW_MAX_SUPPORTED_NODE_CNT];
struct __GW_nodesPendCredsCntxt__  GW_nodesPendCredsList[GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT];
struct __GW_blackListedNode__  GW_blackListedNodeList[GW_MAX_BLACK_LISTED_NODES_LIST_ENTRY_CNT];

char attrName[10],
     shortAddrCpy[10], 
     attrIdCpy[10], 
     attrValCpy[10];


static char CLOUD_IF_tempRcvdMQTTMsgBuff[Q4_MSG_Q_ENTRY_SZ];

static char CLOUD_IF_wsnMsgSerJsonFmt[320];

static char CLOUD_IF_jsonMsgFromWSN[320]; 

const char* MQTT_brokerURL = "broker.hivemq.com";
const char* MQTT_wsnDashBoardBrokerURL = "dashboard.wisense.in";

StaticJsonDocument<128> CLOUD_IF_rcvdNodeProvJsonDoc;
 

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
               Serial.printf("CLI_sPMTC> CRsp MQTT Pub F #%u DisConn !!\n", CLOUD_IF_mqttPubFlrCnt);
               CLOUD_IF_respToCloudMQTTClient.disconnect();
           }
       }
       else
       {
           // Disconnect from the WiFi
           WiFi.disconnect();
           return;
       }
   }

   if (xQueueReceive(Q3Hndl, (void *)CLOUD_IF_jsonMsgFromWSN, (portTickType)0))
   {
       char *rcvdJsonMsg_p;
       uint8_t *eA_p = (uint8_t *)CLOUD_IF_jsonMsgFromWSN;
       int jsonMsgLen;
       struct __GW_nodeCntxt__ *nodeCntxt_p = NULL;
       
       // The received message format is:
       //   8 bytes of extended MAC address
       //   + seralized JSON message
       //   + NULL character
      
       rcvdJsonMsg_p = (char *)(eA_p + DIS_LPWMN_MAC_EXT_ADDR_LEN);
       jsonMsgLen = strlen(rcvdJsonMsg_p) + 1;
       
       Serial.printf("CLI_sPMTC> Rx M L<%d> Q3 %02x:%02x:%02x:%02x\n", 
                     jsonMsgLen, eA_p[4], eA_p[5], eA_p[6], eA_p[7]);
   
       nodeCntxt_p = GW_getNodeCntxt(eA_p);
       
       if (nodeCntxt_p == NULL)
       {
           // No node context found corresponding to the MAC address in the received 
           // message !!. Drop this message.

           MQTT_stats.nodeCntxtNotFndCnt ++;
           Serial.printf("CLI_sPMTC> M S N Not Fnd [%u] !! \n", MQTT_stats.nodeCntxtNotFndCnt);
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
               Serial.printf("CLI_sPMTC> C N MQ U<%s> \n", nodeCntxt_p->userName);
            
               boolean status = mqtt_p->connect("WiSense_LPWMN_MSG",   // ClientId 
                                                nodeCntxt_p->userName,  // "BE8RpfrsvVNaumB2o32V", 
                                                NULL);  // No password !!
               if (status == false)
               {
                   CLOUD_IF_nodeMQTTConnReqFlrCnt ++;   
                   nodeCntxt_p->connReqFlrCnt ++;
                   Serial.printf("CLI_sPMTC> MQ C R Flr #%u !!\n", CLOUD_IF_nodeMQTTConnReqFlrCnt);
               }
               else
               { 
                   CLOUD_IF_nodeMQTTConnReqOkCnt ++;
                   nodeCntxt_p->connReqOkCnt ++;
                   Serial.printf("CLI_sPMTC> MQ C R OK #%u !!\n", CLOUD_IF_nodeMQTTConnReqOkCnt);
               }
           }
                    
           if (mqtt_p->connected())
           {
               boolean rc;

               // MQTT connection is up, publish the message to the broker
               
               rc = mqtt_p->publish("v1/devices/me/telemetry", rcvdJsonMsg_p);
               nodeCntxt_p->pubCnt ++;
               MQTT_stats.pubCnt ++;
               
               if (rc == false)
               {
                   MQTT_stats.pubFlrCnt ++;
                   nodeCntxt_p->pubFlrCnt ++;
                   Serial.printf("MQTT P Flr !! #%u\n", MQTT_stats.pubFlrCnt);
                   if (mqtt_p->connected())
                   {
                       MQTT_stats.pubFlrConnUpCnt ++;
                       nodeCntxt_p->pubFlrConnUpCnt ++;
                       Serial.printf("MQTT P Flr Conn Up !!, #%u\n", MQTT_stats.pubFlrConnUpCnt);
                   }
                   else
                   {
                       MQTT_stats.pubFlrConnDownCnt ++;
                       nodeCntxt_p->pubFlrConnDownCnt;
                       Serial.printf("MQTT publish failed and MQTT conn is down");            
                   }
               }
               else
               {
                   MQTT_stats.pubOkCnt ++;
                   nodeCntxt_p->pubOkCnt ++;
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


uint32_t __asciihex2uint(uint8_t nibLChar, uint8_t nibHChar)
{
   uint32_t retVal = 0;
   
   if (nibHChar >= '0' && nibHChar <= '9')
       nibHChar = nibHChar - '0';
   else
   {
       if (nibHChar >= 'a' && nibHChar <= 'f')
           nibHChar = nibHChar - 'a' + 10;
       else
           return 0x100;
   }

   if (nibLChar >= '0' && nibLChar <= '9')
       nibLChar = nibLChar - '0';
   else
   {
       if (nibLChar >= 'a' && nibLChar <= 'f')
           nibLChar = nibLChar - 'a' + 10;
       else
           return 0x100;
   }

   retVal = nibHChar*16 + nibLChar;

   return retVal;
}


/*
 * Handle requests from WiSense dashboard in the cloud
 */
void MQTT_processMsgFromCloud(char* topic_p, byte* payload_p, unsigned int len)
{
   static uint32_t CLOUD_IF_qToQ4FlrCnt = 0,
                   CLOUD_IF_qToQ4DoneCnt = 0,
                   CLOUD_IF_nodeProvMsgRxCnt = 0,
                   CLOUD_IF_deSerNodeProvMsgFlrCnt = 0,
                   CLOUD_IF_deSerNodeProvMsgOkCnt = 0;

   byte* rcvdMQTTMsg_p = (byte *)CLOUD_IF_tempRcvdMQTTMsgBuff;

   // This function is called in the context of the thread which
   // calls this function CLOUD_IF_respToCloudMQTTClient.loop( ) which
   // in our case is the main thread which runs the main loop.
   
   Serial.printf("MTT C CB FN> T:%s L:%u\n", topic_p, len);
   Serial.printf("MTT C CB FN> Pyld: ");
   Serial.write(payload_p, len);
   Serial.println();
   
   memcpy(rcvdMQTTMsg_p, payload_p, len);

   if (String(topic_p) == "get_attr")
   {
       // Q4 queue entry size is 320
       if (xQueueSend(Q4Hndl, (void*)rcvdMQTTMsg_p, (TickType_t)0) == pdPASS)
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

       return;
   }

   if (String(topic_p) == "wsn_node_access_response")   // "wsn_provision_nwk_node"
   {
       DeserializationError error;

       /*
        * Msg Format from cloud Eg:
        * {
        *    "macId":"fe:c2:xx:xx:xx:xx:xx:xx",
        *    "username":"st8T3pTKQ2LUBWBhCrzk"
        * }
        */

       CLOUD_IF_nodeProvMsgRxCnt ++;
       Serial.printf("M_rFC_CBFn> Prov M Rx #%u \n", CLOUD_IF_nodeProvMsgRxCnt);
       
       error = deserializeJson(CLOUD_IF_rcvdNodeProvJsonDoc, 
                               rcvdMQTTMsg_p);
       if (error)
       {
           CLOUD_IF_deSerNodeProvMsgFlrCnt ++;
           Serial.printf("M_rFC_CBFn> D-S Json Flr #%u !!\n", CLOUD_IF_deSerNodeProvMsgFlrCnt);
           Serial.println(error.f_str());
       }
       else
       {
           const char *macIdStr_p = CLOUD_IF_rcvdNodeProvJsonDoc["macId"];
           const char *userNameStr_p = CLOUD_IF_rcvdNodeProvJsonDoc["username"];
           uint8_t macIdBin[DIS_LPWMN_MAC_EXT_ADDR_LEN];
           int macIdStrLen = strlen(macIdStr_p);

           CLOUD_IF_deSerNodeProvMsgOkCnt ++;

           Serial.printf("M_rFC_CBFn> #%u  M:%s  N:%s\n",
                         CLOUD_IF_deSerNodeProvMsgOkCnt,
                         macIdStr_p, userNameStr_p);

           if (macIdStrLen == 23)
           {
               int off = 0, idx = 0;
               
               for (; idx<DIS_LPWMN_MAC_EXT_ADDR_LEN; idx++)
               {
                    uint8_t nibH = *((uint8_t *)(macIdStr_p + off)),
                            nibL = *((uint8_t *)(macIdStr_p + off + 1));
                    uint32_t byte = __asciihex2uint(nibL, nibH);

                    if (byte > 0xff)
                        break;

                    macIdBin[idx] = (uint8_t)byte;
                           
                    off += 2;
                    
                    char delim = *(macIdStr_p + off);
                    if (delim != '\0' && delim != ':')
                        break;
                        
                    off ++;
               }

               if (idx == DIS_LPWMN_MAC_EXT_ADDR_LEN)
               {
                   GW_allocNodeCntxt(macIdBin,
                                     (char*)userNameStr_p,
                                     NULL);                 
                   GW_rmNodeFromBlackList(macIdBin);
                   GW_rmNodeFromNodesPendingCredsList(macIdBin);
               }
               else
               {
                   Serial.printf("M_rFC_CBFn> Bad MAC Fmt !! \n");
               }
           }
           else
           {
               Serial.printf("M_rFC_CBFn> Bad MAC Len %d !! \n", macIdStrLen);
           }
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


void GW_rmNodeFromBlackList(const uint8_t *extAddr_p)
{
   struct __GW_blackListedNode__ *entry_p = GW_blackListedNodeList;
   int idx;

   for (idx = 0; idx<GW_MAX_BLACK_LISTED_NODES_LIST_ENTRY_CNT; idx++, entry_p++)
   {
      if (entry_p->valid)
      {
          if (memcmp(entry_p->extAddr, extAddr_p, DIS_LPWMN_MAC_EXT_ADDR_LEN) == 0)
          {        
              // We could use locks to make it safer
              entry_p->valid = 0;
              memset(entry_p->extAddr, 0, DIS_LPWMN_MAC_EXT_ADDR_LEN);
              break;
          }
      }
   }

   return;
}


void GW_addNodeToBlackList(const uint8_t *extAddr_p)
{
   struct __GW_blackListedNode__ *entry_p = GW_blackListedNodeList;
   int idx;
   
   // Ext address not found, add it to list if not blacklisted
   for (idx = 0; idx<GW_MAX_BLACK_LISTED_NODES_LIST_ENTRY_CNT; idx++, entry_p++)
   {
      if (!entry_p->valid)
      {
          memcpy(entry_p->extAddr, extAddr_p, DIS_LPWMN_MAC_EXT_ADDR_LEN);        
          entry_p->valid = 1;
      }
   }

   return;
}


bool GW_checkNodeBlackListed(const uint8_t *extAddr_p)
{
   struct __GW_blackListedNode__ *entry_p = GW_blackListedNodeList;
   int idx;
     
   // Ext address not found, add it to list if not blacklisted
   for (idx = 0; idx<GW_MAX_BLACK_LISTED_NODES_LIST_ENTRY_CNT; idx++)
   {
      if (entry_p->valid)
      {
          if (memcmp(extAddr_p, entry_p->extAddr, DIS_LPWMN_MAC_EXT_ADDR_LEN) == 0)
          {
              return true;         
          }
      }

      entry_p ++;
   }

   return false;
}


static char NODE_macIdStrFmt[64];

bool CLOUD_IF_buildSendNodeCredsReqMsg(void)
{
   struct __GW_nodesPendCredsCntxt__ *entry_p = GW_nodesPendCredsList;
   static uint32_t CLOUD_IF_credReqPubOkCnt = 0,
                   CLOUD_IF_credReqPubFlrCnt = 0;
   bool rc = true;
   const char *a_p = (char *)(entry_p->extAddr);
   
   sprintf(NODE_macIdStrFmt, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", 
           a_p[0], a_p[1], a_p[2], a_p[3],
           a_p[4], a_p[5], a_p[6], a_p[7]);

   Serial.printf("C_I_bSNCRM> MAC (%d) %s\n",
                 strlen(NODE_macIdStrFmt), NODE_macIdStrFmt);
             
   rc = CLOUD_IF_reqFromCloudMQTTClient.publish("wsn_get_node_access", NODE_macIdStrFmt);
   if (rc == true)
   {
       CLOUD_IF_credReqPubOkCnt ++;
       Serial.printf("C_I_bSNCRM> CRED Msg Pub OK #%u\n", CLOUD_IF_credReqPubOkCnt);
       entry_p->commAttempts ++;
       entry_p->tickStamp = xTaskGetTickCount();
       entry_p->respPending = true;
   }
   else
   {
       CLOUD_IF_credReqPubFlrCnt ++;
       Serial.printf("C_I_bSNCRM> CRED Msg Pub Flr #%u !!\n", CLOUD_IF_credReqPubFlrCnt);
       entry_p->respPending = false;
       entry_p->commAttempts = 0;
       entry_p->tickStamp = 0;
   }

   return rc;
}


int CLOUD_IF_procPendCredsListHead(void)
{
   int rc = 255;
   struct __GW_nodesPendCredsCntxt__ *head_p = GW_nodesPendCredsList;
   
   if (head_p->valid)
   {
       if (head_p->respPending)
       {
           uint32_t deltaTicks, currTicks = xTaskGetTickCount();

           deltaTicks = currTicks - head_p->tickStamp;
           deltaTicks *= portTICK_RATE_MS;

           if (deltaTicks > CLOUD_IF_MAX_CLOUD_RESP_WAIT_TIME)
           {
               if (head_p->commAttempts > CLOUD_IF_MAX_CLOUD_COMM_ATTEMPTS)
               {  
                   // Something wrong with the MQTT conn !!
                   head_p->commAttempts = 0;
                   head_p->tickStamp = 0;
                   head_p->respPending = false;
                   rc = 0;  
               }
               else
               {
                   // Try again
                   rc = 1;
               }
           }    
           else
           {
               // timeout not happend yet .. nothing else to do
               rc = 2;
           }
       }
       else
       {
           // Try first attempt
           rc = 1;
       }
   }
   else
   {
       // Get an entry from the list 
       rc = 3;
   }

   configASSERT(rc != 255);
   
   return rc;
}

   
bool CLOUD_IF_procNodesPendingCredsList(void)
{
   int rc;
   bool locked = true, retCode = true;

   // We need a mutex because the nodesPendingCredsList data structure
   // is accessed by two threads. One is the main loop( ) and the other is
   // the COORD_INTF_coordMsgProcTask which handles messages from the 
   // coordinator.
   
   // xSemaphoreTake(CLOUD_IF_nodeListPendingCredsMutexHndl, 
   //               portMAX_DELAY);

   rc = CLOUD_IF_procPendCredsListHead( );
   //Serial.printf("C_I_pNPCL> rc: %d\n", rc);
   switch (rc)
   {
      case 0:
           {
              // No response from cloud !!
              CLOUD_IF_respToCloudMQTTClient.disconnect();
              retCode = false;
           }
           break;
           
      case 1:
           {
              bool retVal;
              // No response but more attempts left.

              // xSemaphoreGive(CLOUD_IF_nodeListPendingCredsMutexHndl);
              // locked = false;

              // This does not need a lock because the zero index (head) entry 
              // is already valid
              retVal = CLOUD_IF_buildSendNodeCredsReqMsg();
              if (!retVal)
              {
                  CLOUD_IF_respToCloudMQTTClient.disconnect();
                  retCode = false;
              }
           }
           break;

      case 2: 
           {
              // Timeout has not occurred yet. Need to wait some more.
           }
           break;
         
      case 3:
           { 
              int idx = 1;
              struct __GW_nodesPendCredsCntxt__ *entry_p = GW_nodesPendCredsList;

              // Index 0 is empty. Get a valid entry from the rest of the list.
              
              for (; idx<GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT; idx++, entry_p++)
              {
                   if (entry_p->valid)
                   {
                       bool retVal;
                       struct __GW_nodesPendCredsCntxt__ *head_p = GW_nodesPendCredsList;

                       Serial.printf("C_I_pNPCL> FND V E at %d\n", idx);
                       
                       // Copy to index 0
                       memcpy(head_p, entry_p, sizeof(__GW_nodesPendCredsCntxt__));

                       // zero out source entry
                       memset(entry_p, 0, sizeof(__GW_nodesPendCredsCntxt__));

                       // xSemaphoreGive(CLOUD_IF_nodeListPendingCredsMutexHndl);
                       // locked = false;

                       head_p->commAttempts = 0;
                       head_p->tickStamp = 0;
                       head_p->respPending = false;
                       
                       retVal = CLOUD_IF_buildSendNodeCredsReqMsg();
                       if (!retVal)
                       {
                           CLOUD_IF_respToCloudMQTTClient.disconnect();
                           retCode = false;
                       }

                       break;
                   }
              }             
           }
           break;
           
      default:
           {
              configASSERT(0);
           }
   }

   // if (locked)
   //    xSemaphoreGive(CLOUD_IF_nodeListPendingCredsMutexHndl);

   return retCode;
}


void GW_rmNodeFromNodesPendingCredsList(const uint8_t *extAddr_p)
{
   int idx = 0, freeIdx = GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT;
   struct __GW_nodesPendCredsCntxt__ *cntxt_p = GW_nodesPendCredsList;

   // Start from index 0 when removing. We go through all the entries
   // so that we take care of situations where the cloud dashboard
   // sends unsolicited node credentials.

   
   for (; idx<GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT; idx++, cntxt_p ++)
   {
       if (cntxt_p->valid)
       {
           if (memcmp(extAddr_p, cntxt_p->extAddr, DIS_LPWMN_MAC_EXT_ADDR_LEN) == 0)
           {
               cntxt_p->valid = 0;
               memset(cntxt_p, 0, sizeof(struct __GW_nodesPendCredsCntxt__));
               break;
           }
       }
   }

   return;  
}


void GW_addNodeToPendingCredsList(const uint8_t *extAddr_p)
{
   int idx = 0, freeIdx = GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT;
   struct __GW_nodesPendCredsCntxt__ *entry_p = GW_nodesPendCredsList;

   // Make sure this mac address is added only once to the list

   // xSemaphoreTake(CLOUD_IF_nodeListPendingCredsMutexHndl, 
   //               portMAX_DELAY);

   for (; idx<GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT; idx++, entry_p ++)
   {
       if (entry_p->valid)
       {
           if (memcmp(extAddr_p, entry_p->extAddr, DIS_LPWMN_MAC_EXT_ADDR_LEN) == 0)
           {
               // MAC address already in the list.
               freeIdx = GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT;
               Serial.printf("C_I_aNPCL> MAC A P at #%d\n", idx);
               break;
           }
       }
       else
       {
           if (freeIdx == GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT)
               freeIdx = idx;
       }
   }
   
   if (freeIdx != GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT)
   {
       entry_p = GW_nodesPendCredsList + freeIdx;
       memcpy(entry_p->extAddr, extAddr_p, DIS_LPWMN_MAC_EXT_ADDR_LEN);
       entry_p->tickStamp = 0;
       entry_p->commAttempts = 0;
       entry_p->respPending = 0;
       entry_p->valid = 1;
       Serial.printf("C_I_aNPCL> New E allocd at #%d\n", freeIdx);
   }

   // xSemaphoreGive(CLOUD_IF_nodeListPendingCredsMutexHndl);

   return;
}


struct __GW_nodeCntxt__ *GW_getNodeCntxt(const uint8_t *extAddr_p)
{
   int idx = 0;
   struct __GW_nodeCntxt__ *nodeCntxt_p = GW_nodeCntxtList;

   for (; idx<GW_MAX_SUPPORTED_NODE_CNT; idx++)
   {
       if (nodeCntxt_p->valid)
       {
           if (memcmp(extAddr_p, nodeCntxt_p->extAddr, DIS_LPWMN_MAC_EXT_ADDR_LEN) == 0)
           {
               Serial.printf("GW_gNC> idx:%d  \n", idx);
               return nodeCntxt_p;
           }
       }

       nodeCntxt_p ++;
   }

   return NULL;
}


void GW_allocNodeCntxt(const uint8_t *extAddr_p,
                       char *userName_p,
                       char *password_p)
{
   int idx = 0;
   struct __GW_nodeCntxt__ *nodeCntxt_p = GW_nodeCntxtList;   

   for (; idx<GW_MAX_SUPPORTED_NODE_CNT; idx++)
   {
        if (nodeCntxt_p->valid == 0)
            break;
        nodeCntxt_p ++;
   }

   Serial.printf("M_aNS> idx:%d  \n", idx);

   if (idx < GW_MAX_SUPPORTED_NODE_CNT)
   {
       // Unused entry found
       
       memcpy(nodeCntxt_p->extAddr, extAddr_p, DIS_LPWMN_MAC_EXT_ADDR_LEN);
       
       strncpy(nodeCntxt_p->userName, userName_p, sizeof(nodeCntxt_p->userName));
       
       if (password_p != NULL)
           strncpy(nodeCntxt_p->password, password_p, sizeof(nodeCntxt_p->password));

       nodeCntxt_p->valid = 1;
   }
   else
   {
       Serial.printf("M_aNS> No free slots !!\n");
   }
}


void CLOUD_IF_reqFromCloudMQTTConn(void)
{
   static uint32_t CLOUD_IF_reqFromCloudMQTTConnOkCnt = 0,
                   CLOUD_IF_reqFromCloudMQTTConnFlrCnt = 0,
                   CLOUD_IF_getAttrSubReqFlrCnt = 0,
                   CLOUD_IF_setAttrSubReqFlrCnt = 0,
                   CLOUD_IF_mqttSubReqOkCnt = 0,
                   CLOUD_IF_provNwkNodeSubReqFlrCnt = 0;
                   
   if (!(CLOUD_IF_reqFromCloudMQTTClient.connected()))
   {
       boolean status;
       
       Serial.printf("C_I_cCRMC> C_I_coReqMC N C !! \n");
       
       status = CLOUD_IF_reqFromCloudMQTTClient.connect("WSN_GW_reqFromCloudMQTTClient", "wisense", "Wisense@123");
       if (status == false)
       {
           CLOUD_IF_reqFromCloudMQTTConnFlrCnt ++;   
           Serial.printf("C_L_rFCMC> CRsp MQC CR Fld #%u !!\n", CLOUD_IF_reqFromCloudMQTTConnFlrCnt);
       }
       else
       { 
           boolean subReqSts;
           
           CLOUD_IF_reqFromCloudMQTTConnOkCnt ++;
           Serial.printf("C_L_rFCMC> CRsp MQC CR OK #%u\n", CLOUD_IF_reqFromCloudMQTTConnOkCnt);

           subReqSts = CLOUD_IF_reqFromCloudMQTTClient.subscribe("get_attr");
           if (subReqSts == true)
           {
               CLOUD_IF_mqttSubReqOkCnt ++;
               subReqSts = CLOUD_IF_reqFromCloudMQTTClient.subscribe("wsn_node_access_response"); // "wsn_provision_nwk_node"
               if (subReqSts == true)
               {
                   CLOUD_IF_mqttSubReqOkCnt  ++;
                   subReqSts = CLOUD_IF_reqFromCloudMQTTClient.subscribe("set_attr");
                   if (subReqSts == false)
                       CLOUD_IF_setAttrSubReqFlrCnt ++;
                   else
                       CLOUD_IF_mqttSubReqOkCnt ++;
               }
               else
                   CLOUD_IF_provNwkNodeSubReqFlrCnt ++;
           }
           else
               CLOUD_IF_getAttrSubReqFlrCnt ++;

           if (subReqSts == false)
           {
               CLOUD_IF_getAttrSubReqFlrCnt ++;
               Serial.printf("C_L_rFCMC> sub req flr - ga#%u /pn %u/ sa%u !!\n", 
                             CLOUD_IF_getAttrSubReqFlrCnt, 
                             CLOUD_IF_provNwkNodeSubReqFlrCnt,
                             CLOUD_IF_getAttrSubReqFlrCnt); 
               CLOUD_IF_reqFromCloudMQTTClient.disconnect();
           }
           else
           {
               Serial.printf("C_L_rFCMC> all subscribe reqs done - #%u\n", CLOUD_IF_mqttSubReqOkCnt);
           }
       }        
   }
}


void MQTT_init(void)
{
   // The "CLOUD_IF_respToCloudMQTTClient" is used to receive commands from the 
   // cloud resident wisense server. These commands can be for the coordinator
   // or for any attached node. There is only one such client.
   
   CLOUD_IF_reqFromCloudMQTTClient.setServer("3.21.126.253", 1883);
   CLOUD_IF_reqFromCloudMQTTClient.setCallback(MQTT_processMsgFromCloud);
   
   CLOUD_IF_respToCloudMQTTClient.setServer(MQTT_wsnDashBoardBrokerURL, 1883);
   CLOUD_IF_respToCloudMQTTClient.setCallback(MQTT_respToCloudCallBackFn);

   // This MQTT client is used only to send messages (received from the nodes)
   // to the cloud. The broker is in the wisense server and each dashboard (one
   // per node) is a client of this broker.
   CLOUD_IF_nodesMQTTClient.setServer(MQTT_wsnDashBoardBrokerURL, 1883);

   {
      int idx = 0;
      struct __GW_nodeCntxt__ *nodeCntxt_p = GW_nodeCntxtList;   
      struct __GW_nodesPendCredsCntxt__ *pendEntry_p = GW_nodesPendCredsList;
      struct __GW_blackListedNode__  *blEntry_p = GW_blackListedNodeList;
      
      for (; idx<GW_MAX_SUPPORTED_NODE_CNT; idx++, nodeCntxt_p++)
           memset(nodeCntxt_p, 0, sizeof(struct __GW_nodeCntxt__));
   
      for (idx=0; idx<GW_MAX_NODES_PENDING_CREDS_LIST_ENTRY_CNT; idx++, pendEntry_p++)
           memset(pendEntry_p, 0, sizeof(struct __GW_nodesPendCredsCntxt__));

      for (idx=0; idx<GW_MAX_BLACK_LISTED_NODES_LIST_ENTRY_CNT; idx++, blEntry_p++)
           memset(blEntry_p, 0, sizeof(struct __GW_blackListedNode__));
   }        

   return;
}
