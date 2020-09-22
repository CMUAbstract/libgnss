#ifndef _GNSS_H_
#define _GNSS_H_

#include <stdint.h>

#define GNSS_START '$'
#define GNSS_SEP ','
#define GNSS_CHECKSUM_START '*'
#define GNSS_END '\n'
#define SENTENCE_LEN 42
#define FULL_SENTENCE_LEN 47

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
