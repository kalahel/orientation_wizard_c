//
// Created by Henri on 27/05/2019.
//

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions      */
#include <errno.h>   /* ERROR Number Definitions           */
#include <assert.h>
#include <string.h>

#ifndef SERIAL_GPS_GPS
#define SERIAL_GPS_GPS

#define READ_BUFFER_SIZE 1024
#define POSITION_LINE_PREFIX_GPGGA "$GPGGA"

struct Gps_data {
    char *tag;
    char *utc_time;
    char *latitude;
    char *n_s_indicator;
    char *longitude;
    char *e_w_indicator;
    char *pos_fix_indicator;
    char *nb_sat_used;
    char *hdop;
    char *msl_altitude;
    char *units;
    char *geoid_separation;
    char *checksum;
} typedef Gps_data;

char **str_split(char *a_str, const char a_delim);
_Bool starts_with(const char *string, const char *prefix);
void init_serial_read();
void print_gps_data();
void get_gps_data();

extern Gps_data gps_data;

#endif //SERIAL_GPS_GPS