#include "gnss.h"

#include <libio/console.h>
#include <libmsp/mem.h>

// All volatile variables that are reset on boot
sentence_type pkt_type = INVALID;
uint8_t active_pkt = 0;
uint8_t gnss_pkt_counter = 0;

char type_buff[5];

// Process incoming data to figure out the NMEA sentence type
void get_sentence_type(char data) {
  // GPGGA, GPGLL, GPRMC
  switch (gnss_pkt_counter) {
    case 0:
      //if (data != 'G') { // removed to allow for Baidu
      if (data < 'A' || data > 'Z') {
        pkt_type = INVALID;
      }
        break;
    case 1:
      //if (data != 'P' && data != 'N') { // removed to allow for Baidu?
      if (data < 'A' || data > 'Z') {
        pkt_type = INVALID;
      }
        break;
    case 2:
      if (data != 'G' && data != 'R') {
        pkt_type = INVALID;
      }
        break;
    case 3:
      if (data == 'G' || (data == 'L' && type_buff[2] == 'G')) {
        break;
      }
      else if (data == 'M' && type_buff[2] == 'R') {
        break;
      }
      else {
        pkt_type = INVALID;
        break;
      }
    case 4:
      if (data == 'A' && type_buff[3] == 'G') {
        active_pkt = 1;
        pkt_type = GPGGA;
        break;
      }
      if (data == 'L' && type_buff[3] == 'L') {
        active_pkt = 1;
        pkt_type = GPGLL;
        break;
      }
      if (data == 'C' && type_buff[3] == 'M') {
        active_pkt =1;
        pkt_type = GPRMC;
        break;
      }
  }
  type_buff[gnss_pkt_counter] = data;
  gnss_pkt_counter++;
  return;
}


__nv sentence gnss_sentence1;
sentence *cur_gnss_ptr = &gnss_sentence1;

// Returns 1 if packet is complete, 0 if still processing
int get_sentence_pkt(char data) {
  //TODO field filtering
  if (gnss_pkt_counter < FULL_SENTENCE_LEN) {
    cur_gnss_ptr->buffer[gnss_pkt_counter - 5] = data;
    gnss_pkt_counter++;
    return 0;
  }
  else {
    // Set packet type
    cur_gnss_ptr->type = pkt_type;
    return 1;
  }
}


// Squish pkt_in generic buffer into real gps data pkt_out
int process_sentence_pkt(sentence *pkt_in, gps_data *pkt_out) {
  //GGA: time (6.6 digits), lat (x.x digits), n/s (1, N/S), long (x.x digits), e/w
  //(1, E/W),fix (1 digit).... 13,9,1,10,1,1,...
  //GLL: lat (4.2 digits), N/S, long (5.2 digits), E/W, time (6 digits), fix
  //(A/V)...
  //RMC: time (6 digits), fix (A/V), lat, N/S, long, E/W,...
  // Make sure we can't partially fill the data
  pkt_out->complete = 0;
  if (pkt_in->buffer[0] != ',') {
    // Error processing
    return 1;
  }
  LOG("checking time %i\r\n",pkt_in->type); switch (pkt_in->type) {
    case GPGGA:
      for (int i = 0; i < 13; i++) {
        pkt_out->time[i] = pkt_in->buffer[i + 1];
        LOG("time: %c \r\n",pkt_in->buffer[i+1]);
      }
      if (pkt_in->buffer[14] != ',') { return 1; }
      for (int i = 0; i < 9; i++) {
        pkt_out->lat[i] = pkt_in->buffer[i + 15];
        LOG("lat: %c \r\n",pkt_in->buffer[i+15]);
      }
      pkt_out->lat[9] = pkt_in->buffer[25];
      if (pkt_in->buffer[24] != ',' || pkt_in->buffer[26] != ',') { return 1; }
      for (int i = 0; i < 10; i++) {
        pkt_out->longi[i] = pkt_in->buffer[i + 27];
        LOG("long: %c \r\n",pkt_in->buffer[i+27]);
      }
      if (pkt_in->buffer[37] != ',' || pkt_in->buffer[39] != ',' ||
        pkt_in->buffer[41] != ',') { 
        LOG("comma error %c %c %c \r\n", pkt_in->buffer[37],
        pkt_in->buffer[39],pkt_in->buffer[41]);
        return 1;
      }
      pkt_out->longi[10] = pkt_in->buffer[38];
      // Fix > 1 == OK
      if (pkt_in->buffer[40] > '0' && pkt_in->buffer[40] < '8') {
        pkt_out->fix[0] = FIX_OK;
        LOG("Fix valid!\r\n");
      }
      else {
        pkt_out->fix[0] = FIX_INVALID;
        LOG("Fix invalid!!!! %c\r\n", pkt_in->buffer[40]);
      }
      LOG("Good PKT!\r\n");
      break;
    case GPGLL:
      // Capture ,9,1,10,1,13,1, = 42
      for (int i = 0; i < 9; i++) {
        pkt_out->lat[i] = pkt_in->buffer[i + 1];
      }
      if (pkt_in->buffer[10] != ',') { return 1; }
      pkt_out->lat[9] = pkt_in->buffer[11];
      if (pkt_in->buffer[12] != ',' || pkt_in->buffer[23] != ',') { return 1; }
      for (int i = 0; i < 10; i++) {
        pkt_out->longi[i] = pkt_in->buffer[i + 13];
      }
      pkt_out->longi[10] = pkt_in->buffer[24];
      if (pkt_in->buffer[25] != ',' || pkt_in->buffer[39] != ',' ||
        pkt_in->buffer[41] != ',') { return 1; }
      for (int i = 0; i < 13; i++) {
        pkt_out->time[i] = pkt_in->buffer[i + 26];
      }
      // A = OK
      if (pkt_in->buffer[40] == 'A') {
        pkt_out->fix[0] = FIX_OK;
        LOG("Fix valid!\r\n");
      }
      else {
        pkt_out->fix[0] = FIX_INVALID;
        LOG("Fix invalid!!!! %c\r\n", pkt_in->buffer[40]);
      }
      LOG("Good PKT GLL!\r\n");
      break;
    case GPRMC:
        // Capture ,13,1,9,1,10,1, = 42
      for (int i = 0; i < 13; i++) {
        pkt_out->time[i] = pkt_in->buffer[i + 1];
      }
      if (pkt_in->buffer[14] != ',') { return 1; }
      // A = ok
      if (pkt_in->buffer[15] == 'A') {
        pkt_out->fix[0] = FIX_OK;
        LOG("Fix valid!\r\n");
      }
      else {
        pkt_out->fix[0] = FIX_INVALID;
        LOG("Fix invalid!!!! %c\r\n", pkt_in->buffer[15]);
      }
      for (int i = 0; i < 9; i++) {
        pkt_out->lat[i] = pkt_in->buffer[i + 17];
      }
      if (pkt_in->buffer[16] != ',' || pkt_in->buffer[26] != ',') { return 1; }
      pkt_out->lat[9] = pkt_in->buffer[27];
      for (int i = 0; i < 10; i++) {
        pkt_out->longi[i] = pkt_in->buffer[i + 29];
      }
      pkt_out->longi[10] = pkt_in->buffer[40];
      if (pkt_in->buffer[28] != ',' || pkt_in->buffer[39] != ',' ||
        pkt_in->buffer[41] != ',') { return 1; }
      LOG("Good PKT RMC!\r\n");
      break;
    default:
      break;
  }
  pkt_out->complete = 1;
  return 0;
}

#define DEGS(x) \
((x[0] - 0x30)*10 + (x[1] - 0x30))


// To add more "good" locations, update this function with their lat/long and
// extend the locations enum with the new location
locations good_location(gps_data *pkt) {
  int lat = DEGS(pkt->lat);
  char lat_d = pkt->lat[9];
  int longi = DEGS(pkt->longi);
  char longi_d = pkt->longi[10];
  // Over pittsburgh
  if (lat < 41 && lat > 39 && lat_d == 'N') {
    if (longi < 81 && longi > 79 && longi_d == 'W')
      return BURGH;
  }
  // Over US in general
  if (lat < 45 && lat > 25 && lat_d == 'N') {
    if (longi < 130 && longi > 70 && longi_d == 'W')
      return USA;
  }
  // Over EURO
  if (lat > 35 && lat < 70 && lat_d == 'N') {
    if (longi_d == 'W' && longi < 10)
      return EURO;
    if (longi_d == 'E' && longi < 60)
      return EURO;
  }
  return NOWHERE;
}




