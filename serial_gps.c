//
// Created by Henri on 27/05/2019.
//
#include "serial_gps.h"

Gps_data gps_data;
FILE *fo;
int file_descriptor;/*File Descriptor*/
FILE *file_output;

const char mode[1] = {'r'};
size_t len = 0;
char *line = NULL;
ssize_t line_read_size;

char **str_split(char *a_str, const char a_delim) {
    char **result = 0;
    size_t count = 0;
    char *tmp = a_str;
    char *last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;
    /* Count how many elements will be extracted. */
    while (*tmp) {
        if (a_delim == *tmp) {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }
    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);
    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;
    result = (char**)(malloc(sizeof(char *) * count));
    if (result) {
        size_t idx = 0;
        char *token = strtok(a_str, delim);
        while (token) {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        //printf("idx : %d\n count : %d\n", idx, count);
        //assert(idx == count - 1);
        *(result + idx) = 0;
    }
    return result;
}

int starts_with(const char *string, const char *prefix) {
    while (*prefix) {
        if (*prefix++ != *string++)return 0;
    }
    return 1;
}

void init_serial_read() {

    printf("\n +----------------------------------+");
    printf("\n |        Serial Port Read          |");
    printf("\n +----------------------------------+");

    /*------------------------------- Opening the Serial Port -------------------------------*/

    file_descriptor = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
    /* O_RDWR   - Read/Write access to serial port       */
    /* O_NOCTTY - No terminal will control the process   */

    if (file_descriptor == -1)                        /* Error Checking */
        printf("\n  Error! in Opening ttyAMA0");
    else
        printf("\n  ttyAMA0 Opened Successfully ");


    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    struct termios SerialPortSettings;  /* Create the structure                          */

    tcgetattr(file_descriptor, &SerialPortSettings); /* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&SerialPortSettings, B9600); /* Set Read  Speed as 19200                       */
    cfsetospeed(&SerialPortSettings, B9600); /* Set Write Speed as 19200                       */

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;    /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |= CS8;      /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 127; /* Read at least 10 characters */
    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


    if ((tcsetattr(file_descriptor, TCSANOW, &SerialPortSettings)) !=
        0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 9600 \n  StopBits = 1 \n  Parity   = none\n");

    /*------------------------------- Read data from serial port -----------------------------*/

    char read_buffer[READ_BUFFER_SIZE];   /* Buffer to store the data received              */
    int bytes_read = 0;    /* Number of bytes read by the read() system call */
    int i = 0;

    /*
     *
     */

    /*
     *
     */



    tcflush(file_descriptor, TCIFLUSH);   /* Discards old data in the rx buffer */
}

void print_gps_data(){
    printf("tag : %s\n"
           "utc_time : %s\n"
           "latitude : %s\n"
           "n_s_indicator : %s\n"
           "longitude : %s\n"
           "e_w_indicator : %s\n"
           "pos_fix_indicator : %s\n"
           "nb_sat_used : %s\n"
           "hdop : %s\n"
           "msl_altitude : %s\n"
           "units : %s\n"
           "geoid_separation : %s\n"
           "checksum : %s\n\n\n",
           gps_data.tag, gps_data.utc_time, gps_data.latitude, gps_data.n_s_indicator, gps_data.longitude,
           gps_data.e_w_indicator, gps_data.pos_fix_indicator, gps_data.nb_sat_used, gps_data.hdop,
           gps_data.msl_altitude, gps_data.units, gps_data.geoid_separation, gps_data.checksum);
}

void get_gps_data() {
    file_output = fdopen(file_descriptor, mode);
    //bytes_read = read(fd, &read_buffer, READ_BUFFER_SIZE -1); /* Read the data

    /*
     *
     */

    line_read_size = getline(&line, &len, file_output);
    //printf("Got line of length : %zu:/n", read);
    if (starts_with(line, POSITION_LINE_PREFIX_GPGGA)) {//printf("%s", line);

        char **splited_line;
        const char delimiter = ',';
        splited_line = str_split(line, delimiter);
        gps_data.tag = splited_line[0];
        gps_data.utc_time = splited_line[1];
        gps_data.latitude = splited_line[2];
        gps_data.n_s_indicator = splited_line[3];
        gps_data.longitude = splited_line[4];
        gps_data.e_w_indicator = splited_line[5];
        gps_data.pos_fix_indicator = splited_line[6];
        gps_data.nb_sat_used = splited_line[7];
        gps_data.hdop = splited_line[8];
        gps_data.msl_altitude = splited_line[9];
        gps_data.units = splited_line[10];
        gps_data.geoid_separation = splited_line[11];
        gps_data.checksum = splited_line[15];
    }
}


    //init_serial_read();
    //get_gps_data();
    //close(file_descriptor); /* Close the serial port */

    /*int i;
    splited_line = strtok(line, delimiter);
    printf("%s\n", splited_line);
    for(i = 0; i<6 ; i++){
        splited_line = strtok(NULL, delimiter);
        printf("%s\n", splited_line);
    }*/
    //splited_line = strtok(line,delimiter);
    //printf("%s\n", splited_line);
    //splited_line = strtok(line, delimiter);

    //printf("\n\n  Bytes Rxed : %d", bytes_read); /* Print the number of bytes read */
    //printf("\n\n  ");
    //for (i = 0; i < 255; i++)    /*printing only the needed bytes*/
    //    printf("%c", read_buffer[i]);
    //printf("\n +----------------------------------+\n\n\n");
