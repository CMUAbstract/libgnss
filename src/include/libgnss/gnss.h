#ifndef _GNSS_H_
#define _GNSS_H_

#include <stdint.h>

#define GNSS_START '$'
#define GNSS_SEP ','
#define GNSS_CHECKSUM_START '*'
#define GNSS_END '\n'
#define SENTENCE_LEN 42
#define FULL_SENTENCE_LEN 47

#define DISABLE_MSG_LEN 25


// For all of the following definitions, you pass in the gps_data field that's
// in use, Ex:
// DEGS_LAT(pkt->lat)
#define DEGS_LAT(x) \
((x[0] - 0x30)*10 + (x[1] - 0x30))

#define DEGS_LONG(x) \
((x[0] - 0x30)*100 + (x[1] - 0x30)*10 + (x[2] - 0x30))

#define NS(x)\
(x[9] == 'N')

#define EW(x)\
(x[10] == 'E')

#define MIN_LAT(x) \
((x[2] - 0x30)*10 + (x[3] - 0x30))

#define MIN_LONG(x) \
((x[3] - 0x30)*10 + (x[4] - 0x30))

#define SECS_LAT(x) \
((x[5] - 0x30)*1000 +(x[6] - 0x30)*100 + (x[7] - 0x30)*10 + (x[8] - 0x30))

#define SECS_LONG(x) \
((x[6] - 0x30)*1000 +(x[7] - 0x30)*100 + (x[8] - 0x30)*10 + (x[9] - 0x30))

#define UTC_HRS(x) \
((x[0] - 0x30)*10 + (x[1] - 0x30))

#define UTC_MMS(x) \
((x[2] - 0x30)*10 + (x[3] - 0x30))

#define UTC_SECS(x) \
((x[4] - 0x30)*10 + (x[5] - 0x30))

extern const char disable01[DISABLE_MSG_LEN];
extern const char disable02[DISABLE_MSG_LEN];
extern const char disable03[DISABLE_MSG_LEN];
extern const char disable04[DISABLE_MSG_LEN];
extern const char disable05[DISABLE_MSG_LEN];

typedef enum fix_types_ {
  FIX_OK,
  FIX_INVALID
} fix_type;

typedef enum sentence_types_ {
  IN_PROGRESS,
  INVALID,
  GPGGA,
  GPGLL,
  GPRMC
} sentence_type;

typedef struct sentence_ {
  char buffer[SENTENCE_LEN];
  sentence_type type;
} sentence;

typedef struct gps_data_ {
  char time[13];
  char lat[10];
  char longi[11];
  fix_type fix[1];
  int complete;
} gps_data;

typedef  enum locations_ {
  BURGH,
  EURO,
  USA,
  NOWHERE
} locations;

void get_sentence_type(char data);
int get_sentence_pkt(char data);
int process_sentence_pkt(sentence *pkt_in, gps_data *pkt_out);
locations good_location(gps_data *pkt);

extern sentence *cur_gnss_ptr;
extern sentence_type pkt_type;
extern uint8_t active_pkt;
extern uint8_t gnss_pkt_counter;

#endif
