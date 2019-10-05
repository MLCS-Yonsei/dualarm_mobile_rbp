#ifndef CAN_UTILS_LIB_H
#define CAN_UTILS_LIB_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <linux/can.h>
#include <linux/can/error.h>

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'



/*
 * Returns the decimal value of a given ASCII hex character.
 *
 * While 0..9, a..f, A..F are valid ASCII hex characters.
 * On invalid characters the value 16 is returned for error handling.
 */
unsigned char asc2nibble(char c) {

  if ((c >= '0') && (c <= '9'))
    return c - '0';

  if ((c >= 'A') && (c <= 'F'))
    return c - 'A' + 10;

  if ((c >= 'a') && (c <= 'f'))
    return c - 'a' + 10;

  return 16; /* error */
}


/*
 * Converts a given ASCII hex string to a (binary) byte string.
 *
 * A valid ASCII hex string consists of an even number of up to 16 chars.
 * Leading zeros '00' in the ASCII hex string are interpreted.
 *
 * Examples:
 *
 * "1234"   => data[0] = 0x12, data[1] = 0x34
 * "001234" => data[0] = 0x00, data[1] = 0x12, data[2] = 0x34
 *
 * Return values:
 * 0 = success
 * 1 = error (in length or the given characters are no ASCII hex characters)
 *
 * Remark: The not written data[] elements are initialized with zero.
 *
 */
int hexstring2data(char *arg, unsigned char *data, int maxdlen) {

  int len = strlen(arg);
  int i;
  unsigned char tmp;

  if (!len || len%2 || len > maxdlen*2)
    return 1;

  memset(data, 0, maxdlen);

  for (i=0; i < len/2; i++) {

    tmp = asc2nibble(*(arg+(2*i)));
    if (tmp > 0x0F)
      return 1;

    data[i] = (tmp << 4);

    tmp = asc2nibble(*(arg+(2*i)+1));
    if (tmp > 0x0F)
      return 1;

    data[i] |= tmp;
  }

  return 0;
}


/*
 * Transfers a valid ASCII string decribing a CAN frame into struct canfd_frame.
 *
 * CAN 2.0 frames
 * - string layout <can_id>#{R{len}|data}
 * - {data} has 0 to 8 hex-values that can (optionally) be separated by '.'
 * - {len} can take values from 0 to 8 and can be omitted if zero
 * - return value on successful parsing: CAN_MTU
 *
 * CAN FD frames
 * - string layout <can_id>##<flags>{data}
 * - <flags> a single ASCII Hex value (0 .. F) which defines canfd_frame.flags
 * - {data} has 0 to 64 hex-values that can (optionally) be separated by '.'
 * - return value on successful parsing: CANFD_MTU
 *
 * Return value on detected problems: 0
 *
 * <can_id> can have 3 (standard frame format) or 8 (extended frame format)
 * hexadecimal chars
 *
 *
 * Examples:
 *
 * 123# -> standard CAN-Id = 0x123, len = 0
 * 12345678# -> extended CAN-Id = 0x12345678, len = 0
 * 123#R -> standard CAN-Id = 0x123, len = 0, RTR-frame
 * 123#R0 -> standard CAN-Id = 0x123, len = 0, RTR-frame
 * 123#R7 -> standard CAN-Id = 0x123, len = 7, RTR-frame
 * 7A1#r -> standard CAN-Id = 0x7A1, len = 0, RTR-frame
 *
 * 123#00 -> standard CAN-Id = 0x123, len = 1, data[0] = 0x00
 * 123#1122334455667788 -> standard CAN-Id = 0x123, len = 8
 * 123#11.22.33.44.55.66.77.88 -> standard CAN-Id = 0x123, len = 8
 * 123#11.2233.44556677.88 -> standard CAN-Id = 0x123, len = 8
 * 32345678#112233 -> error frame with CAN_ERR_FLAG (0x2000000) set
 *
 * 123##0112233 -> CAN FD frame standard CAN-Id = 0x123, flags = 0, len = 3
 * 123##1112233 -> CAN FD frame, flags = CANFD_BRS, len = 3
 * 123##2112233 -> CAN FD frame, flags = CANFD_ESI, len = 3
 * 123##3 -> CAN FD frame, flags = (CANFD_ESI | CANFD_BRS), len = 0
 *     ^^
 *     CAN FD extension to handle the canfd_frame.flags content
 *
 * Simple facts on this compact ASCII CAN frame representation:
 *
 * - 3 digits: standard frame format
 * - 8 digits: extendend frame format OR error frame
 * - 8 digits with CAN_ERR_FLAG (0x2000000) set: error frame
 * - an error frame is never a RTR frame
 * - CAN FD frames do not have a RTR bit
 */
int parse_canframe(char *cs, struct canfd_frame *cf)
{

  int i, idx, dlen, len;
  int maxdlen = CAN_MAX_DLEN;
  int ret = CAN_MTU;
  unsigned char tmp;

  len = strlen(cs);

  memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

  if (len < 4)
    return 0;

  if (cs[3] == CANID_DELIM) /* 3 digits */
  {

    idx = 4;
    for (i=0; i<3; i++){
      if ((tmp = asc2nibble(cs[i])) > 0x0F)
        return 0;
      cf->can_id |= (tmp << (2-i)*4);
    }

  }
  else if (cs[8] == CANID_DELIM) /* 8 digits */
  {

    idx = 9;
    for (i=0; i<8; i++){
      if ((tmp = asc2nibble(cs[i])) > 0x0F)
        return 0;
      cf->can_id |= (tmp << (7-i)*4);
    }
    if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
      cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

  }
  else
  {
    return 0;
  }

  if((cs[idx] == 'R') || (cs[idx] == 'r')) /* RTR frame */
  {
    cf->can_id |= CAN_RTR_FLAG;

    /* check for optional DLC value for CAN 2.0B frames */
    if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
      cf->len = tmp;

    return ret;
  }

  if (cs[idx] == CANID_DELIM) /* CAN FD frame escape char '##' */
  {

    maxdlen = CANFD_MAX_DLEN;
    ret = CANFD_MTU;

    /* CAN FD frame <canid>##<flags><data>* */
    if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
      return 0;

    cf->flags = tmp;
    idx += 2;
  }

  for (i=0, dlen=0; i < maxdlen; i++){

    if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
      idx++;

    if(idx >= len) /* end of string => end of data */
      break;

    if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
      return 0;
    cf->data[i] = (tmp << 4);
    if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
      return 0;
    cf->data[i] |= tmp;
    dlen++;
  }
  cf->len = dlen;

  return ret;

}



#endif // _CAN_UTILS_LIB_H_