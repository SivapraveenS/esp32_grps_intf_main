/*
 *********************************************************************
 * File: mqtt_rx_proc.ino
 * 
 * Author: rkris@wisense.in / siva@wisense.in
 * 
 * Date: March/2021
 * 
 * (C) WiSense Technologies Pvt Ltd
 * All rights reserved
 * *******************************************************************
 */

#include <ArduinoJson.h>

byte CLOUD_IF_rcvdCloudMqttMsg[Q4_MSG_Q_ENTRY_SZ];


StaticJsonDocument<512> CLOUD_IF_rcvdCloudJsonDoc;


void GW_savReqHndlr(char *reqStr_p)
{
   // Parse the command string to extract addr, attrId and attrVal

   
}


void COORD_IF_gavReqHndlr(char *reqStr_p)
{
   int attrId = 0, nwkAddr = 0;
   
   // Parse the command string to extract addr and attrId,
  
   if (nwkAddr == 1)
   {
       COORD_IF_getCoordAttrVal(attrId);
   }
   else
   {
       COORD_IF_getNodeAttrVal(nwkAddr, attrId);
   }
}


void Cloud_IF_rxPathTask(void *params_p)
{
   static uint32_t CLOUD_IF_cloudMsgRxCnt = 0,
                   CLOUD_IF_deSerJSONMsgFlrCnt = 0,
                   CLOUD_IF_deSerJSONMsgOkCnt = 0;
   
   while (1)
   {
      // Q4 queue entry size is 320
      if (xQueueReceive(Q4Hndl, 
                        (void *)CLOUD_IF_rcvdCloudMqttMsg,
                        (portTickType)portMAX_DELAY))
      {
          DeserializationError error;

          CLOUD_IF_cloudMsgRxCnt ++;
          Serial.printf("C_I_rPT> Rx M F Cloud #%u \n", CLOUD_IF_cloudMsgRxCnt);
          
          error = deserializeJson(CLOUD_IF_rcvdCloudJsonDoc, 
                                  CLOUD_IF_rcvdCloudMqttMsg);
          if (error) 
          {
              CLOUD_IF_deSerJSONMsgFlrCnt ++;
              Serial.printf("C_I_rPT> D-S Json Flr #%u !!\n", CLOUD_IF_deSerJSONMsgFlrCnt);
              Serial.println(error.f_str());
              continue;
          }
          else
          {
              CLOUD_IF_deSerJSONMsgOkCnt ++;
              Serial.printf("C_I_rPT> D-S Json OK #%u\n", CLOUD_IF_deSerJSONMsgOkCnt);

              // Check if this is a GET or SET request

              if (1)
                 COORD_IF_gavReqHndlr("");

              if (2)
                 GW_savReqHndlr("");
          }
      }
   }
}
