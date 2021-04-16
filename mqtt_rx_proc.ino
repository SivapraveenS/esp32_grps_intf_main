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

//#define CLOUD_MAP_EXT_TO_SHORT_ENABLE

#include <ArduinoJson.h>

byte CLOUD_IF_rcvdCloudMqttMsg[Q4_MSG_Q_ENTRY_SZ];


StaticJsonDocument<512> CLOUD_IF_rcvdCloudJsonDoc;

#ifdef CLOUD_MAP_EXT_TO_SHORT_ENABLE

void COORD_IF_setNodeAttrVal(const uint8_t *extAddr_p, 
                             const uint32_t attrId, 
                             const int attrVal)
{
   
}


int COORD_IF_getNodeAttrVal(const uint8_t *extAddr_p, 
                            const uint32_t attrId)
{

  
}

#endif


void Cloud_IF_rxPathTask(void *params_p)
{
   static uint32_t CLOUD_IF_cloudMsgRxCnt = 0,
                   CLOUD_IF_deSerJSONMsgFlrCnt = 0,
                   CLOUD_IF_deSerJSONMsgOkCnt = 0,
                   CLOUD_IF_extA2ShortAMapFlrCnt = 0,
                   CLOUD_IF_extA2ShortAMapOkCnt = 0;
   
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
              const char *cmdStr_p = CLOUD_IF_rcvdCloudJsonDoc["cmdType"];
              JsonVariant forCoordVariant = CLOUD_IF_rcvdCloudJsonDoc["forCoord"].as<JsonVariant>();
              int forCoord = forCoordVariant.as<int>();
              JsonVariant attrIdVariant = CLOUD_IF_rcvdCloudJsonDoc["attrId"].as<JsonVariant>();
              unsigned int attrId = attrIdVariant.as<unsigned int>();
              
#ifdef CLOUD_MAP_EXT_TO_SHORT_ENABLE
              uint8_t *extAddr_p = NULL;
#else
              uint16_t extAddr_p = NULL;
#endif              
              CLOUD_IF_deSerJSONMsgOkCnt ++;
              Serial.printf("C_I_rPT> D-S Json OK #%u\n", CLOUD_IF_deSerJSONMsgOkCnt);

              if (forCoord)
              {
                  Serial.printf("C_I_rPT> Cmd for Coord\n");
                  extAddr_p = NULL;
              }

#ifdef CLOUD_MAP_EXT_TO_SHORT_ENABLE         
              else
              {                                
                  const char *wsnAddr_p = CLOUD_IF_rcvdCloudJsonDoc["wsnAddr"];
                  uint8_t nodeExtAddr[DIS_LPWMN_MAC_EXT_ADDR_LEN] = {0};
                  int wsnAddrLen = strlen(wsnAddr_p);
                  // Convert extended addr from string to bin fmt

                  Serial.printf("C_I_rPT> Node Addr <%s>, len <%d> \n", wsnAddr_p, wsnAddrLen);
                  
                  if(wsnAddrLen == 23)
                  {
                      int off = 0, idx = 0;

                      for (; idx<DIS_LPWMN_MAC_EXT_ADDR_LEN; idx++)
                      {
                        uint8_t nibH = *((uint8_t *)(wsnAddr_p + off)),
                                nibL = *((uint8_t *)(wsnAddr_p + off + 1));
                        uint32_t byte = __asciihex2uint(nibL, nibH);
    
                        if (byte > 0xff)
                            break;
    
                        nodeExtAddr[idx] = (uint8_t)byte;
                               
                        off += 2;
                        
                        char delim = *(wsnAddr_p + off);
                        if (delim != '\0' && delim != ':')
                            break;
                            
                        off ++;
                      }
    
                      Serial.printf("C_I_rPT> Ext-NodeAddr: ");
                      for(int indx = 0; indx < 8; indx ++)
                      {
                          Serial.printf("%02x |", nodeExtAddr[indx]);
                      }
                      Serial.printf("\n");
                      extAddr_p = nodeExtAddr;
                  }
              }

              if (strcmp(cmdStr_p, "gav") == 0)
              {   
                  Serial.printf("C_I_rPT> cmd:%s id:%d\n", cmdStr_p, attrId);    
                                
                  if (extAddr_p == NULL)
                  {
                      int rc = COORD_IF_getCoordAttrVal(attrId);
                      if(rc != 1)
                          Serial.printf("C_I_rPT> get-CoordAttrVal failed !! \n");
                  }
                  else
                  {   
                      uint16_t shortAddr = COORD_IF_mapExt2ShortAddr(extAddr_p);

                      if (shortAddr != WSN_SHORT_BROADCAST_ADDR)
                      {
                          CLOUD_IF_extA2ShortAMapOkCnt ++;
                          Serial.printf("C_I_rPT> SA:%u SALU Ok #%u \n", 
                                        (uint32_t)shortAddr, CLOUD_IF_extA2ShortAMapOkCnt);
                          COORD_IF_getNodeAttrVal(extAddr_p, attrId);
                      }
                      else
                      {
                          CLOUD_IF_extA2ShortAMapFlrCnt ++;
                          Serial.printf("C_I_rPT> SALU F !! #%u\n", CLOUD_IF_extA2ShortAMapFlrCnt);
                      }
                  }
              }
              
              if (strcmp(cmdStr_p, "sav") == 0)
              {
                  JsonVariant attrValVariant = CLOUD_IF_rcvdCloudJsonDoc["attrVal"].as<JsonVariant>();
                  int attrVal = attrValVariant.as<int>();

                  Serial.printf("C_I_rPT> cmd:%s id:%d, val:%d\n", 
                                cmdStr_p, attrId, attrVal);         

                  if (extAddr_p == NULL)
                  {
                      int rc = COORD_IF_setCoordAttrVal(attrId, attrVal);
                      if(rc != 1)
                          Serial.printf("C_I_rPT> set-CoordAttrVal failed !! \n");
                  }
                  else
                  {
                      uint16_t shortAddr = COORD_IF_mapExt2ShortAddr(extAddr_p);
                      
                      if (shortAddr != WSN_SHORT_BROADCAST_ADDR)
                      {
                          CLOUD_IF_extA2ShortAMapOkCnt ++;
                          Serial.printf("C_I_rPT> SA:%u SALU Ok #%u \n", 
                                        (uint32_t)shortAddr, CLOUD_IF_extA2ShortAMapOkCnt);
                          COORD_IF_setNodeAttrVal(extAddr_p, attrId, attrVal);
                      }
                      else
                      {
                          CLOUD_IF_extA2ShortAMapFlrCnt ++;
                          Serial.printf("C_I_rPT> SALU F !! #%u\n", CLOUD_IF_extA2ShortAMapFlrCnt);
                      }
                  }
              } //extAddr - confg

#else
           else
              {
                  JsonVariant wsnAddrVariant = CLOUD_IF_rcvdCloudJsonDoc["wsnAddr"].as<JsonVariant>();
                  uint16_t wsnSrtAddr = wsnAddrVariant.as<unsigned int>();                                

                  Serial.printf("C_I_rPT> Node short Addr <%u>/<%d> \n", wsnSrtAddr);
                  
                  extAddr_p = wsnSrtAddr;
                
              }
              
              if (strcmp(cmdStr_p, "gav") == 0)
              {   
                  Serial.printf("C_I_rPT> cmd:%s id:%d\n", cmdStr_p, attrId);    
                                
                  if (extAddr_p == NULL)
                  {
                      int rc = COORD_IF_getCoordAttrVal(attrId);
                      if(rc != 1)
                          Serial.printf("C_I_rPT> get-CoordAttrVal failed !! \n");
                  }
                  else
                  {                     
                      uint16_t shortAddr = (uint16_t)extAddr_p;

                      Serial.printf("C_I_rPT> shortAddr: %u \n", shortAddr);
                      if (shortAddr != WSN_SHORT_BROADCAST_ADDR)
                      {
                          CLOUD_IF_extA2ShortAMapOkCnt ++;
                          Serial.printf("C_I_rPT> SA:%u SALU Ok #%u \n", 
                                                                       (uint32_t)shortAddr, CLOUD_IF_extA2ShortAMapOkCnt);
                          COORD_IF_getNodeAttrVal(shortAddr, attrId);
                      }
                      else
                      {
                          CLOUD_IF_extA2ShortAMapFlrCnt ++;
                          Serial.printf("C_I_rPT> SALU F !! #%u\n", CLOUD_IF_extA2ShortAMapFlrCnt);
                      }
                  }
              }
              
              if (strcmp(cmdStr_p, "sav") == 0)
              {
                  JsonVariant attrValVariant = CLOUD_IF_rcvdCloudJsonDoc["attrVal"].as<JsonVariant>();
                  int attrVal = attrValVariant.as<int>();

                  Serial.printf("C_I_rPT> cmd:%s id:%d, val:%d\n", 
                                cmdStr_p, attrId, attrVal);         

                  if (extAddr_p == NULL)
                  {
                      int rc = COORD_IF_setCoordAttrVal(attrId, attrVal);
                      if(rc != 1)
                          Serial.printf("C_I_rPT> set-CoordAttrVal failed !! \n");
                  }
                  else
                  {
                      char *strAttrVal_p = NULL;                    
                      uint16_t shortAddr = (uint16_t)extAddr_p;
                      Serial.printf("C_I_rPT> shortAddr: %u \n", shortAddr);
                      if (shortAddr != WSN_SHORT_BROADCAST_ADDR)
                      {
                          CLOUD_IF_extA2ShortAMapOkCnt ++;
                          Serial.printf("C_I_rPT> SA:%u SALU Ok #%u \n", 
                                        (uint32_t)shortAddr, CLOUD_IF_extA2ShortAMapOkCnt);
                          COORD_IF_setNodeAttrVal(shortAddr, attrId, attrVal, strAttrVal_p);
                      }
                      else
                      {
                          CLOUD_IF_extA2ShortAMapFlrCnt ++;
                          Serial.printf("C_I_rPT> SALU F !! #%u\n", CLOUD_IF_extA2ShortAMapFlrCnt);
                      }
                  }
              } //shortAddr - confg
#endif 
          }
       }
    }
}
