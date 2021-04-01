
StaticJsonDocument<1024> CLOUD_IF_rcvdMqttNodeCred;

static char CLOUD_IF_nodeAcsResp[1024]; 

unsigned char macIdCpy[8];

void CLOUD_IF_newEntryAllocation()
{
  
     static uint32_t CLOUD_IF_cloudMqttNodeCredRxCnt = 0,
                   CLOUD_IF_deSerNodeCredJSONMsgFlrCnt = 0,
                   CLOUD_IF_deSerNodeCredJSONMsgOkCnt = 0;

    if(xQueueReceive(Q7Hndl, (void *)CLOUD_IF_nodeAcsResp, (portTickType)0))
    { 
        Serial.printf("C_I_nEA> Msg in Q7Hndl / wsn_node_access_response: <%s> \n", CLOUD_IF_nodeAcsResp);

        DeserializationError error;

        CLOUD_IF_cloudMqttNodeCredRxCnt ++;
        Serial.printf("C_I_nEA> Rx M F Cloud #%u \n", CLOUD_IF_cloudMqttNodeCredRxCnt);
          
        error = deserializeJson(CLOUD_IF_rcvdMqttNodeCred, 
                                  CLOUD_IF_nodeAcsResp);
        if (error) 
        {
            CLOUD_IF_deSerNodeCredJSONMsgFlrCnt ++;
            Serial.printf("C_I_nEA> D-S Node_Cred Json Flr #%u !!\n", CLOUD_IF_deSerNodeCredJSONMsgFlrCnt);
            Serial.println(error.f_str());
        }
        else
        {
           CLOUD_IF_deSerNodeCredJSONMsgOkCnt ++;
           Serial.printf("C_I_nEA> D-S Node_Cred Json OK #%u\n", CLOUD_IF_deSerNodeCredJSONMsgOkCnt);

           /* 
            * Msg Format from cloud Eg: {"macId":"fe:c2:xx:xx:xx:xx:xx:xx", "username":"st8T3pTKQ2LUBWBhCrzk"}
            * 
            */
          
           const char* nodeMacId = CLOUD_IF_rcvdMqttNodeCred["macId"];
           const char* nodeUserName = CLOUD_IF_rcvdMqttNodeCred["username"];
           Serial.printf("C_I_nEA> NodeMacId: ");
           Serial.printf(nodeMacId);
           Serial.printf("\n");
           Serial.printf("C_I_nEA> NodeUserName: ");
           Serial.printf(nodeUserName);
           Serial.printf("\n");

           Serial.printf("C_I_nEA> nodeMacId Len: %d, nodeUserName len: %d \n", strlen(nodeMacId), strlen(nodeUserName));
       
           char* nodeUserNameStr = (char*)nodeUserName;

           sscanf(nodeMacId+21, "%02hhX", &macIdCpy[7]);
           sscanf(nodeMacId+18, "%02hhX", &macIdCpy[6]);
           sscanf(nodeMacId+15, "%02hhX", &macIdCpy[5]);
           sscanf(nodeMacId+12, "%02hhX", &macIdCpy[4]);
           sscanf(nodeMacId+9 , "%02hhX", &macIdCpy[3]);
           sscanf(nodeMacId+6 , "%02hhX", &macIdCpy[2]);
           sscanf(nodeMacId+3 , "%02hhX", &macIdCpy[1]);
           sscanf(nodeMacId+0 , "%02hhX", &macIdCpy[0]);
                                                                                            
           const uint8_t WSN_COORD_SIM_nodeCmnExtAddr[] = {macIdCpy[0], 
                                                           macIdCpy[1], 
                                                           macIdCpy[2], 
                                                           macIdCpy[3], 
                                                           macIdCpy[4],
                                                           macIdCpy[5],
                                                           macIdCpy[6],
                                                           macIdCpy[7]
                                                          };    
            Serial.printf("C_I_nEA> E-NodeMacId: ");
            for(int indx =0; indx < 8; indx ++)
            {
                Serial.printf("%x ", WSN_COORD_SIM_nodeCmnExtAddr[indx]);
            }
            Serial.printf("\n");

            MQTT_allocNewSession(WSN_COORD_SIM_nodeCmnExtAddr,
                               (char*)nodeUserNameStr,
                               NULL);                    
        }
        memset(macIdCpy, NULL , sizeof(macIdCpy));
    }
}
