/*
   The PubSubClient library provides a client for doing simple
   publish/subscribe messaging with a server that supports MQTT.
*/

extern QueueHandle_t CoordIntf_uartRxQEvtHndl;
extern QueueHandle_t Q2Hndl;
extern QueueHandle_t Q3Hndl;
extern QueueHandle_t Q4Hndl;
extern QueueHandle_t Q5Hndl;
extern QueueHandle_t Q6Hndl;

PubSubClient mqttClientCoord(TinyGSM_client_coord);
PubSubClient mqttClientCoordResp(TinyGSM_client_coordResp);
PubSubClient mqttCmnClient(tinyGsmCmnClient);

const char* mqttBrokerUrl = "broker.hivemq.com";
const char* mqttBrokerWsUrl = "dashboard.wisense.in";

typedef struct
{
  bool mqttClientCoordCntSts;
  bool mqttClientCoordRespCntSts;
  bool mqttClientCmnCntSts;
}MQTT_cnt_sts_t;

MQTT_cnt_sts_t MQTT_cnt_sts = {false, false, false};

void MQTT_callbackCoord(char* topic, byte* payload, unsigned int len)
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
      if (xQueueSend(Q4Hndl, (void*)mqttMsgSub, (TickType_t)10) == pdPASS)
      {
        Serial.printf("\n MQTT msg passed into Q4Hndl...\n");
      }
      else
      {
        Serial.printf("\nNo queue space in Q4Hndl...\n");
      }
    }
    free(mqttMsgSub);
    mqttMsgSub = NULL;
}

void MQTT_callbackCoordResp(char* topic, byte* payload, unsigned int len)
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

void MQTT_callbackCmnClient(char* topic, byte* payload, unsigned int len)
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

void MQTT_connChecks(void)
{
  if (!mqttClientCoord.connected())
  {
      DBG("Establishing MQTT connection to ", mqttBrokerUrl);
      boolean status = mqttClientCoord.connect("MQTT_client_coord", "NULL", "NULL");
      char* message = "{\"test\":\"success\"}";
      int length = strlen(message);
      boolean retained = true;
      mqttClientCoord.publish("get_attr", (byte*)message, length, retained);
      delay(1000);
      if (status == false)
      {
          DBG("Failed to setup MQTT_client_coord connection !!! \n");
          delay(1 * 1000);
          MQTT_cnt_sts.mqttClientCoordCntSts = false;
      }
      else
      {
          MQTT_cnt_sts.mqttClientCoordCntSts = true;
          DBG("MQTT connection established for MQTT_client_coord :-) ");
          boolean sub_sts = mqttClientCoord.subscribe(get_attr);
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
  if (mqttClientCoord.connected())
  {
      MQTT_cnt_sts.mqttClientCoordCntSts = true;
  }
}

void COORD_IF_checkCoordResp(void)
{
  if (xQueueReceive(Q6Hndl, (void *)&rcvdCoordResp, (portTickType)0))
  {
      Serial.printf("\nReceived coord responce: <%s>\n", rcvdCoordResp);
      root_coordResp["put_attr"] = rcvdCoordResp;
      serializeJson(jsonDoc_coordResp, GW_coordMsg);
      Serial.printf("\nGW_coordMsg:");
      Serial.printf(GW_coordMsg);
      if (!mqttClientCoordResp.connected())
      {
          MQTT_cnt_sts.mqttClientCoordRespCntSts == false;
          boolean status = mqttClientCoordResp.connect("GsmClient_coordResp", "BE8RpfrsvVNaumB2o32V", "NULL");
          if (status == false)
          {
              Serial.printf("\nMQTT_client_coordResp connect failed...");
              MQTT_cnt_sts.mqttClientCoordRespCntSts = false;
          }
          else
          {
              Serial.printf("\nMQTT_client_coordResp connect success...");
              MQTT_cnt_sts.mqttClientCoordRespCntSts = true;
          }
     }
     if (mqttClientCoordResp.connected())
     {
         MQTT_cnt_sts.mqttClientCoordRespCntSts = true;
     }
     if(MQTT_cnt_sts.mqttClientCoordRespCntSts == true && GSM_status.gsmCntSts == true)
     {
         boolean rc = mqttClientCoordResp.publish("v1/devices/me/telemetry", GW_coordMsg);
         if (rc == true)
         {
             Serial.printf("\nCoord Response was publish successfully...");
         }
         else
         {
             if (!mqttClientCoordResp.connected())
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
     memset(rcvdCoordResp, 0, sizeof(rcvdCoordResp));
     memset(GW_coordMsg, 0, sizeof(rcvdCoordResp));
     mqttClientCoordResp.disconnect();
  }

  if (xQueueReceive(Q3Hndl, &rcvdNodePyld, (portTickType)0))
  {
    Serial.printf("\nReceived Complete payload: ");
    Serial.printf("%s", rcvdNodePyld);
    Serial.printf("\n");
    int rcvdNodeNameCnt = 0, rcvdNodePyldLen;
    rcvdNodePyldLen = strlen(rcvdNodePyld);
    
    memcpy(rcvdNodeName, rcvdNodePyld + 9 , 12);  
    if(GSM_status.gsmCntSts == true)
    {
        if(strcmp(rcvdNodeName, "WSN-fe0d4a46") == 0)
        {
            Serial.printf("Received Node[node-1]: %s \n", rcvdNodeName);
            DBG("Establishing MQTT Client Connection to ", mqttBrokerWsUrl);
            Serial.printf("\n");
            boolean status = mqttCmnClient.connect("GsmClient_01", "p0lsPBWqHD2AWhmKUlEu", "NULL");
            if(status == false)
            {
                Serial.printf("Failed to setup MQTT Client Connetion [node-1] !! \n ");
                delay(1*1000);
                MQTT_cnt_sts.mqttClientCmnCntSts = false;
            }
            else
            {
                MQTT_cnt_sts.mqttClientCmnCntSts = true;
                Serial.printf("MQTT Client [node-1] was established :-) \n");
            }
        }
        if(strcmp(rcvdNodeName, "WSN-fe0d5f7c") == 0)
        {
            Serial.printf("Received Node[node-2]: %s \n", rcvdNodeName);
            DBG("Establishing MQTT Client Connection to ", mqttBrokerWsUrl);
            Serial.printf("\n");
            boolean status = mqttCmnClient.connect("GsmClient_02", "st8T3pTKQ2LUBWBhCrzk", "NULL");
            if(status == false) 
            {
                Serial.printf("Failed to setup MQTT Client Connection [node-2] !! \n");
                delay(1*1000);
                MQTT_cnt_sts.mqttClientCmnCntSts = false;
            }
            else
            {
                MQTT_cnt_sts.mqttClientCmnCntSts = true;
                Serial.printf("MQTT Client [node-2] was established :-) \n");
            }
        }
        if(MQTT_cnt_sts.mqttClientCmnCntSts == true)
        {
            boolean rc = mqttCmnClient.publish("v1/devices/me/telemetry", rcvdNodePyld);
            MQTT_stats.pubCnt ++;
            if (rc == false)
            {
                MQTT_stats.pubFlrCnt ++;
                DBG("MQTT publish failed !!!, cnt: ", MQTT_stats.pubFlrCnt);
                if (mqttCmnClient.connected())
                {
                     DBG("MQTT publish failed but MQTT conn is still up !! \n");
                     MQTT_stats.pubFlrConnUpCnt ++;
                }
                else
                {
                     DBG("MQTT publish failed and MQTT conn is down !! \n");
                     MQTT_stats.pubFlrConnDownCnt ++;
                }
            }
            else
            {
                MQTT_stats.pubSuccessCnt ++;
                DBG("MQTT publish successful, cnt: ", MQTT_stats.pubSuccessCnt);
                Serial.printf("\n**********************MSG Published to Cloud - Success***************\n");
            }
            mqttCmnClient.disconnect();    
            Serial.printf("\n\n\n");
        }
        else
        {
            Serial.printf("MQTT Common Client Connection failed !! \n");
        }
      }
      else
      {
          Serial.printf("\nDropping Payload, Modem Connection Down !!\n");
      }
      MQTT_cnt_sts.mqttClientCmnCntSts = false;
      memset(rcvdNodeName, 0, sizeof(rcvdNodeName));
      memset(rcvdNodePyld, 0, sizeof(rcvdNodePyld)); 
  }
}


void CloudIf_rxPathTask(void *params_p)
{
  StaticJsonDocument<512> jsonDoc_coord;
  DBG("\n CloudIf_rxPathTask Running...\n");
  while (1)
  {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (xQueueReceive(Q4Hndl, (void *)&rcvdMqttMsg, (portTickType)10))
    {
      int i = 0, i_tmp = 0;
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

      i = 0;
      int len = strlen(rcvdCoordAttr);
      int cnt = 0, cnxtSpcCntStr[10];

      memset(attrName, 0, sizeof(attrName));
      memset(attrArgv1, 0, sizeof(attrArgv1));
      memset(attrArgv2, 0, sizeof(attrArgv2));
      memset(attrArgv3, 0, sizeof(attrArgv3));
      memset(attrArgv4, 0, sizeof(attrArgv4));
      memset(attrArgv5, 0, sizeof(attrArgv5));
      
      for (i = 0; i < strlen(rcvdCoordAttr); i++)
      {
        if (rcvdCoordAttr[i] == ' ')
        {
          Serial.printf("found space indx <%d> \n", i + 1);
          cnxtSpcCntStr[cnt] = i + 1;
          ++cnt;
        }
      }
      printf("Total Entered Cnxt <%d>\n", cnt);
      if (cnt > 0)
      {
        Serial.printf("cnxtSpcCntStr value fields:");
        for (int i = 0; i < cnt; i++)
        {
          printf("%d ", cnxtSpcCntStr[i]);
        }
        printf("\n");
      }
      if (cnt == 0)
      {
        printf("Entered attributes Cntx length - 1 !! \n");
        printf("len: %d !! \n", len);
        memcpy(attrArgv1, rcvdCoordAttr, len);
      }
      if (cnt == 1)
      {
        printf("Entered attributes Cnxt length - 2 !! \n");
        memcpy(attrArgv1, rcvdCoordAttr, cnxtSpcCntStr[0]-1);
        memcpy(attrArgv2, rcvdCoordAttr + cnxtSpcCntStr[0], len);

      }
      if (cnt == 2)
      {
        printf("Entered attributes Cnxt length - 3 !! \n");
        memcpy(attrArgv1, rcvdCoordAttr, cnxtSpcCntStr[0] - 1);
        memcpy(attrArgv2, rcvdCoordAttr + cnxtSpcCntStr[0], abs(cnxtSpcCntStr[0] - (cnxtSpcCntStr[1] - 1)));
        memcpy(attrArgv3, rcvdCoordAttr + cnxtSpcCntStr[1], len);
      }
      if (cnt == 3)
      {
        printf("Entered attributes Cnxt length - 4 !! \n");
        memcpy(attrArgv1, rcvdCoordAttr, cnxtSpcCntStr[0] - 1);
        memcpy(attrArgv2, rcvdCoordAttr + cnxtSpcCntStr[0], abs(cnxtSpcCntStr[0] - (cnxtSpcCntStr[1] - 1)));
        memcpy(attrArgv3, rcvdCoordAttr + cnxtSpcCntStr[1], abs(cnxtSpcCntStr[1] - (cnxtSpcCntStr[2] - 1)));
        memcpy(attrArgv4, rcvdCoordAttr + cnxtSpcCntStr[2], len);
      }
      
      if(cnt == 4)
      {
        printf("Entered attributes Cnxt length - 5 !! \n");
        memcpy(attrArgv1, rcvdCoordAttr, cnxtSpcCntStr[0] - 1);
        memcpy(attrArgv2, rcvdCoordAttr + cnxtSpcCntStr[0], abs(cnxtSpcCntStr[0] - (cnxtSpcCntStr[1] - 1)));
        memcpy(attrArgv3, rcvdCoordAttr + cnxtSpcCntStr[1], abs(cnxtSpcCntStr[1] - (cnxtSpcCntStr[2] - 1)));
        memcpy(attrArgv4, rcvdCoordAttr + cnxtSpcCntStr[2], abs(cnxtSpcCntStr[2] - (cnxtSpcCntStr[3] - 1)));
        memcpy(attrArgv5, rcvdCoordAttr + cnxtSpcCntStr[3], len); 
      }

      memcpy(attrName, attrArgv1, strlen(attrArgv1));
      
      if (strcmp(attrName, "sav") == 0)
      {
        unsigned int shortAddr, attrId;
        int attrVal;
        
        Serial.printf("attrName <%s> !! \n", attrName);
        shortAddr = atoi(attrArgv2);
        Serial.printf("shortAddr <%d> !! \n", shortAddr);
        attrId = atoi(attrArgv3);
        Serial.printf("attrId <%d> !! \n", attrId);
        attrVal = atoi(attrArgv4);
        Serial.printf("attrVal <%d> !! \n", attrVal);
        
        if ( shortAddr < 1 || shortAddr > LPWMN_MAX_UNICAST_SHORT_ADDR)
        {
          Serial.printf("Enter a valid short address (>=1 && <= %u) !! \n", LPWMN_MAX_UNICAST_SHORT_ADDR);
        }
        else
        {
          if (attrId < 1 || attrId > 65535)
          {
            Serial.printf("Enter a valid attribute Id (>=1 & < =65535) !! \n");
          }
          else
          {
            char *strAttrVal_p = NULL;
            if (attrVal == NULL)
            {
              Serial.printf("Please enter a valid attribute value !! \n");
            }
            else
            {
              if (shortAddr == LPWMN_COORD_SHORT_ADDR)
              {
                int rc = GW_setCoordAttrVal(shortAddr, attrId, attrVal);
                if (rc != 1)
                {
                  Serial.printf("failed: %d \n", rc);
                }
                else
                {
                  Serial.printf("Request Send, Coord BaudRate Changed <%d> !! \n", rc);
                }
              }
              else
              {
                int rc = GW_setNodeAttrVal(shortAddr, attrId, attrVal, strAttrVal_p);
                if (rc != 1)
                {
                  Serial.printf("failed to [setNodeAttrVal] <%d> \n", rc);
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

      if (strcmp(attrName, "gav") == 0)
      {
        unsigned int shortAddr, attrId;

        Serial.printf("attrName <%s> !! \n", attrName);
        shortAddr = atoi(attrArgv2);
        Serial.printf("shortAddr <%d> !! \n", shortAddr);
        attrId = atoi(attrArgv3);
        Serial.printf("attrId <%d> !! \n", attrId);
        
        if ( shortAddr < 1 || shortAddr > LPWMN_MAX_UNICAST_SHORT_ADDR)
        {
          Serial.printf("Enter a valid short address (>=1 && <= %u) !! \n", LPWMN_MAX_UNICAST_SHORT_ADDR);
        }
        else
        {
          if (attrId < 1 || attrId > 65535)
          {
            Serial.printf("Enter a valid attribute Id (>=1 & < =65535) !! \n");
          }
          else
          {
            if (shortAddr == LPWMN_COORD_SHORT_ADDR)
            {
              int rc = GW_getCoordAttrVal(attrId);
              if(rc != 1)
                  Serial.printf("failed [GW_getCoordAttrVal] rc<%d> !! \n", rc);
              else
                  Serial.printf("Request Sent [GW_getCoordAttrVal] !! \n");   
            }
            else
            {
              int rc = GW_getNodeAttrVal(shortAddr, attrId);
              if(rc != 1)
                  Serial.printf("failed [GW_getNodeAttrVal] rc<%d> !! \n", rc);
              else
                  Serial.printf("Request Sent [GW_getNodeAttrVal] !! \n"); 
            }
          }
          if (attrId == 0)
          {
            Serial.printf("Enter a valid attribute Id (>=1) !! \n");
          }
        }
      }
      
      if(strcmp(attrName, "cfg-dpi") == 0)
      {
        int rc = 1;
        
        unsigned int shortAddr, pi;        
        Serial.printf("attrName <%s> !! \n", attrName);
        shortAddr = atoi(attrArgv2);
        Serial.printf("shortAddr <%d> !! \n", shortAddr);
        pi = atoi(attrArgv3);
        Serial.printf("pi <%d> !! \n", pi);

        if(shortAddr == 0)
        {
            Serial.printf("please enter sensor node's short address (> 0x1) !! \n");
            rc = 8;
        }     
        if (shortAddr < 2)
        {
            Serial.printf("please enter a valid short address (> 1 & <= %u) !! \n", LPWMN_MAX_UNICAST_SHORT_ADDR);
            rc = 8;
        }
        else
        {
            if(pi == 0 || pi > 65535)
            {
                Serial.printf("please enter sensor interval (1 to 65535 secs) !! \n"); 
                rc = 8;
            }
            else
            {
                rc = GW_cfgNodeDataPushInterval(shortAddr, pi);
                if(rc != 1)
                    Serial.printf("failed [GW_cfgNodeDataPushInterval] rc<%d> !! \n", rc);
                else
                    Serial.printf("Request Sent [GW_cfgNodeDataPushInterval] !! \n"); 
            }
         }
      }
      
      if (strcmp(rcvdCoordAttr, "rstc") == 0)
      {
        Serial.printf("Reset the Coordinator !!\n");
        int rc = GW_rebootCoordReq( );
        if (rc == 5)
          Serial.printf("\nGW_rebootCoordReq() Failed <%s>", __FUNCTION__);
      }

      if (strcmp(rcvdCoordAttr, "grch") == 0)
      {
        Serial.printf("Get Radio Channel !!\n");
        int rc = GW_getRadioChannReqHndlr( );
        if (rc == 5)
          Serial.printf("\nGW_getRadioChannReqHndlr() Failed <%s>\n", __FUNCTION__);
      }

      if (strcmp(rcvdCoordAttr, "grbr") == 0)
      {
        Serial.printf("Get Radio BaudRate !!\n");
        int rc = GW_getRadioBaudRate( );
        if (rc == 5)
          Serial.printf("\nGW_getRadioBaudRate() Failed <%s> !!", __FUNCTION__);
      }
    }
    memset(rcvdMqttMsg, 0, sizeof(rcvdMqttMsg));
    memset(rcvdCoordAttrCpy, 0, sizeof(rcvdCoordAttrCpy));
    memset(rcvdMqttMsgCpy, 0, sizeof(rcvdMqttMsgCpy));
  }
}


void MQTT_loop(void)
{
  mqttClientCoord.loop();
  mqttCmnClient.loop();
  mqttClientCoordResp.loop();
}


void MQTT_setUp(void)
{
  mqttClientCoord.setServer(mqttBrokerUrl, 1883);
  mqttClientCoord.setCallback(MQTT_callbackCoord);
  
  mqttClientCoordResp.setServer(mqttBrokerWsUrl, 1883);
  mqttClientCoordResp.setCallback(MQTT_callbackCoordResp);
  
  mqttCmnClient.setServer(mqttBrokerWsUrl, 1883);
  mqttCmnClient.setCallback(MQTT_callbackCmnClient);
}
