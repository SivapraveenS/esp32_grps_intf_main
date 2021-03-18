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

void UTIL_htons(unsigned char *buff_p, unsigned short val)
{
  buff_p[0] = (val >> 8) & 0xff;
  buff_p[1] = (val) & 0xff;
}

void UTIL_htonl(unsigned char *buff_p, unsigned int val)
{
  buff_p[0] = (val >> 24) & 0xff;
  buff_p[1] = (val >> 16) & 0xff;
  buff_p[2] = (val >> 8) & 0xff;
  buff_p[3] = (val) & 0xff;
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
