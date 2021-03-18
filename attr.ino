
int GW_getRadioBaudRate(void)
{
  int rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_GET_RADIO_BAUD_RATE, NULL, 0x0);
  if (rc != 1)
    return 5;

  return rc;
}

int GW_getRadioChannReqHndlr( )
{
  int rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_GET_NWK_CHANNEL, NULL, 0x0);
  if (rc != 1)
    return 5;
  else
    Serial.printf("\nRequest Sent...");
  return rc;
}

void GW_procImageAttrVal(unsigned int attrId, unsigned char *buff2_p)
{
   switch (attrId)
   {
      case FU_IMAGE_STORE_IMAGE_FLAGS_ATTR_ID:
           {
              if (buff2_p[0] == 0x0)
                  Serial.printf("No storage for images on this node !! \n");
              else
              {
                  int idx;
                  Serial.printf("AttrVal: 0x%x 0x%x 0x%x 0x%x\n",
                         buff2_p[0], buff2_p[1],
                         buff2_p[2], buff2_p[3]);
                  for (idx=0; idx<7; idx++)
                  {
                       if (!(buff2_p[0] & (1 << idx)))
                           continue;
                       Serial.printf("Image #%d : %s    %s    %s\n",
                              idx + 1, (buff2_p[1] & (1 << idx)) ? "Valid  " : "Invalid",
                              (buff2_p[2] & (1 << idx)) ? "Active    " : "Not Active",
                              (buff2_p[3] & (1 << idx)) ? "Update in Progress" : "       --");
                  }
              }
           }
           break;

      case FU_IMAGE_STORE_IMAGE_1_IMAGE_RCVD_TIME_STAMP_ATTR_ID:
      case FU_IMAGE_STORE_IMAGE_2_IMAGE_RCVD_TIME_STAMP_ATTR_ID:
           {
              char timeBuff[32];
              unsigned int timeStamp = UTIL_ntohl(buff2_p);

              ctime_r((time_t *)&timeStamp, timeBuff);

              Serial.printf("AttrVal: %s", timeBuff);
           }
           break;

      default:
           {
              Serial.printf("AttrVal<%u> \n", UTIL_ntohl(buff2_p));
           }
           break;
   }

   return;
}


int GW_cfgNodeDataPushInterval(unsigned int shortAddr, unsigned int pi)
{
   int rc, pyldLen, off = 0;
   unsigned char *pyld_p;

   pyldLen = LPWMN_MAC_SHORT_ADDR_LEN
             + DIS_MSG_TYPE_SZ
             + DIS_TLV_HDR_SZ   // DIS_TLV_TYPE_PUSH_INTERVAL
             + LPWMN_GW_MSG_NODE_DATA_PUSH_INTERVAL_FIELD_LEN;

   pyld_p = (unsigned char *)malloc(pyldLen);
   if (pyld_p == NULL)
   {
       printf("malloc(%d) failed !! \n", pyldLen);
       return 1;
   }

   UTIL_htons(pyld_p, shortAddr);
   off = LPWMN_MAC_SHORT_ADDR_LEN;
   pyld_p[off ++] = DIS_MSG_TYPE_CFG_NODE_DATA_PUSH_INTERVAL;
   pyld_p[off ++] = DIS_TLV_TYPE_PUSH_INTERVAL;
   pyld_p[off ++] = LPWMN_GW_MSG_NODE_DATA_PUSH_INTERVAL_FIELD_LEN;
   UTIL_htons(pyld_p + off, pi);

   rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_RELAY_TO_NODE,
                        pyld_p, pyldLen);
   if (rc != 1)
      return rc;
   if (rc == 1)
   {
      Serial.printf("Header Packet Sent, Waiting for Ack Response from Coord !! \n");
   }  
   Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);
   if (xQueueReceive(Q5Hndl, (void *)&hdrAckSts, (portTickType)100))
   {
      if (hdrAckSts == 1)
      {
          Serial.printf("Hdr acked <%s> !! \n", __FUNCTION__);
          for (int i_a = 0; i_a < pyldLen; i_a++)
          {
              Serial.printf("%x ", pyld_p[i_a]);
          }
          rc = UTIL_writeToUART(pyld_p, pyldLen);
          if (rc != 1)
          {
              Serial.printf("<%s> UTIL_writeToUART(%d) failed !! \n",
                            __FUNCTION__, pyldLen);
              return 3;
          }
          else
          {
              Serial.printf("Payload Request Send <%s> !! \n", __FUNCTION__);
          }
        }
        if (hdrAckSts == 0)
        {
            Serial.printf("Header not acked <%s> !! \n", __FUNCTION__);
            return 2;
        }
        hdrAckSts = 0;
      }

   Serial.printf("Request sent ...<%s> !! \n", __FUNCTION__);

   return rc;
}

int GW_getNodeAttrVal(unsigned int shortAddr, unsigned int attrId)
{
   int rc, pyldLen, off = 0;
   unsigned char *pyld_p;

   pyldLen = LPWMN_MAC_SHORT_ADDR_LEN
             + DIS_MSG_TYPE_SZ
             + DIS_ATTR_ID_TLV_SZ;

   pyld_p = (unsigned char *)malloc(pyldLen);
   if (pyld_p == NULL)
   {
       Serial.printf("malloc(%d) failed !! \n", pyldLen);
       return 1;
   }

   UTIL_htons(pyld_p, shortAddr);
   off = LPWMN_MAC_SHORT_ADDR_LEN;
   pyld_p[off ++] = DIS_MSG_TYPE_GET_ATTR_VAL;
   pyld_p[off ++] = DIS_TLV_TYPE_ATTR_ID;
   pyld_p[off ++] = DIS_ATTR_ID_FIELD_SZ;
   UTIL_htons(pyld_p + off, attrId);

   rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_RELAY_TO_NODE,
                        pyld_p, pyldLen);
   if (rc != 1)
      return rc;
   if (rc == 1)
   {
      Serial.printf("Header Packet Sent, Waiting for Ack Response from Coord !! \n");
   }  
   Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);
   if (xQueueReceive(Q5Hndl, (void *)&hdrAckSts, (portTickType)100))
   {
      if (hdrAckSts == 1)
      {
          Serial.printf("Hdr acked <%s> !! \n", __FUNCTION__);
          for (int i_a = 0; i_a < pyldLen; i_a++)
          {
              Serial.printf("%x ", pyld_p[i_a]);
          }
          rc = UTIL_writeToUART(pyld_p, pyldLen);
          if (rc != 1)
          {
              Serial.printf("<%s> UTIL_writeToUART(%d) failed !! \n",
                            __FUNCTION__, pyldLen);
              return 3;
          }
          else
          {
              Serial.printf("Payload Request Send <%s> !! \n", __FUNCTION__);
          }
        }
        if (hdrAckSts == 0)
        {
            Serial.printf("Header not acked <%s> !! \n", __FUNCTION__);
            return 2;
        }
        hdrAckSts = 0;
      }
      Serial.printf("Request sent. Waiting for response from Node ..... \n");
      expDisMsgType = DIS_MSG_TYPE_GET_ATTR_VAL;
      expDisMsgSrcShortAddr = shortAddr;
      
      return rc;
}

int GW_getCoordAttrVal(unsigned int attrId)
{
  int rc, pyldLen, off = 0;
  unsigned char *pyld_p;

  pyldLen = DIS_ATTR_ID_FIELD_SZ;

  pyld_p = (unsigned char *)malloc(pyldLen);
  if (pyld_p == NULL)
  {
    printf("malloc(%d) failed !! \n", pyldLen);
    return 1;
  }

  UTIL_htons(pyld_p, attrId);

  GW_coordPendingAttrId = attrId;

  rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_GET_COORD_ATTR_VAL,
                       pyld_p, pyldLen);
  if (rc != 1)
    return rc;
  if (rc == 1)
  {
    Serial.printf("Header Packet Sent, Waiting for Ack Response from Coord !! \n");
  }
  
  Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);
  if (xQueueReceive(Q5Hndl, (void *)&hdrAckSts, (portTickType)100))
  {
    if (hdrAckSts == 1)
    {
      Serial.printf("Hdr acked <%s> !! \n", __FUNCTION__);
      for (int i_a = 0; i_a < pyldLen; i_a++)
      {
        Serial.printf("%x ", pyld_p[i_a]);
      }
      rc = UTIL_writeToUART(pyld_p, pyldLen);
      if (rc != 1)
      {
        Serial.printf("<%s> UTIL_writeToUART(%d) failed !! \n",
                      __FUNCTION__, pyldLen);
        return 3;
      }
      else
      {
        Serial.printf("Payload Request Send <%s> !! \n", __FUNCTION__);
      }
    }
    if (hdrAckSts == 0)
    {
      Serial.printf("Header not acked <%s> !! \n", __FUNCTION__);
      return 2;
    }
    hdrAckSts = 0;
  }
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
      attrValLen = 2;
      break;

    case RADIO_RF_TX_TEST_CHANN_ID_ATTR_ID:
      attrValLen = 1;
      break;

    case RADIO_RF_TX_TEST_PA_CFG_ATTR_ID:
      attrValLen = 1;
      break;

    case RADIO_START_CW_UNMOD_TX_TEST_ATTR_ID:
      attrValLen = 2;
      break;

    case RADIO_START_CW_MOD_TX_TEST_ATTR_ID:
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

  UTIL_htons(pyld_p, shortAddr);
  off = LPWMN_MAC_SHORT_ADDR_LEN;
  pyld_p[off ++] = DIS_MSG_TYPE_SET_ATTR_VAL;
  pyld_p[off ++] = DIS_TLV_TYPE_ATTR_INFO;
  pyld_p[off ++] = DIS_ATTR_ID_TLV_SZ + DIS_TLV_HDR_SZ + attrValLen;
  pyld_p[off ++] = DIS_TLV_TYPE_ATTR_ID;
  pyld_p[off ++] = DIS_ATTR_ID_FIELD_SZ;
  UTIL_htons(pyld_p + off, attrId);
  off += DIS_ATTR_ID_FIELD_SZ;
  pyld_p[off ++] = DIS_TLV_TYPE_ATTR_VAL;
  pyld_p[off ++] = attrValLen;
  switch (attrValLen)
  {
    case 1:
      pyld_p[off] = attrVal;
      break;

    case 2:
      UTIL_htons(pyld_p + off, attrVal);
      break;

    case 4:
      UTIL_htonl(pyld_p + off, attrVal);
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

  rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_RELAY_TO_NODE,
                       pyld_p, pyldLen);
  if (rc != 1)
    return rc;
  if (rc == 1)
  {
    Serial.printf("Header Packet Sent, Waiting for Ack Response from Coord !! \n");
  }

  Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);
  if (xQueueReceive(Q5Hndl, (void *)&hdrAckSts, (portTickType)100))
  {
    if (hdrAckSts == 1)
    {
      Serial.printf("Hdr acked...<%s> !! \n", __FUNCTION__);
      for (int i_a = 0; i_a < pyldLen; i_a++)
      {
        Serial.printf("%x ", pyld_p[i_a]);
      }
      rc = UTIL_writeToUART(pyld_p, pyldLen);
      if (rc != 1)
      {
        Serial.printf("<%s> UTIL_writeToUART(%d) failed !! \n",
                      __FUNCTION__, pyldLen);
        return 3;
      }
      else
      {
        Serial.printf("Payload Request Send <%s> !! \n", __FUNCTION__);
      }
    }
    if (hdrAckSts == 0)
    {
      Serial.printf("Header not acked <%s> !! \n", __FUNCTION__);
      return 2;
    }
    hdrAckSts = 0;
  }
  return rc;
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
      pyldLen += 2;
      break;

    case PHY_RAW_BAUD_RATE_ATTR_ID:
      pyldLen += 4;
      break;

    case RADIO_RF_TX_TEST_CHANN_ID_ATTR_ID:
      pyldLen += 1;
      break;

    case RADIO_RF_TX_TEST_PA_CFG_ATTR_ID:
      pyldLen += 1;
      break;

    case RADIO_START_CW_UNMOD_TX_TEST_ATTR_ID:
      pyldLen += 2;
      break;

    case RADIO_START_CW_MOD_TX_TEST_ATTR_ID:
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
  UTIL_htons(pyld_p, attrId);
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
      UTIL_htons(pyld_p + DIS_ATTR_ID_FIELD_SZ, attrVal);
      break;

    case PHY_RAW_BAUD_RATE_ATTR_ID:
      UTIL_htonl(pyld_p + DIS_ATTR_ID_FIELD_SZ, attrVal);
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
  rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_SET_COORD_ATTR_VAL,
                       pyld_p, pyldLen);
  if (rc != 1)
    return rc;
  if (rc == 1)
    Serial.printf("Header Packet Sent, Waiting for Ack Response from Coord !! \n");
    
  Serial.printf("HdrACkSts before Ack <%d> \n", hdrAckSts);
  if (xQueueReceive(Q5Hndl, (void *)&hdrAckSts, (portTickType)100))
  {
    if (hdrAckSts == 1)
    {
      Serial.printf("Hdr acked <%s> !! \n", __FUNCTION__);
      for (int i_a = 0; i_a < pyldLen; i_a++)
      {
        Serial.printf("%x ", pyld_p[i_a]);
      }
      rc = UTIL_writeToUART(pyld_p, pyldLen);
      if (rc != 1)
      {
        Serial.printf("<%s> UTIL_writeToUART(%d) failed !! \n",
                      __FUNCTION__, pyldLen);
        return 3;
      }
      else
      {
        Serial.printf("Payload Request Send <%s> !! \n", __FUNCTION__);
      }
    }
    if (hdrAckSts == 0)
    {
      Serial.printf("Header not acked <%s> !! \n", __FUNCTION__);
      return 2;
    }
    hdrAckSts = 0;
  }

  return rc;
}

int GW_rebootCoordReq(void)
{
  int rc;

  rc = CoordIf_buildSendHdrToCoord(LPWMN_GW_MSG_TYPE_REBOOT_COORD, NULL, 0x0);
  if (rc != 1)
  {
    Serial.printf("failed !! \n");
    rc = 5;
  }
  else
    Serial.printf("Request sent ... ");

  return rc;
}
