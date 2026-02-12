#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/ioctl.h>


#define INPUT_SIZE 8 * 1000000
#define RUN_COUNT 100

#define READ_FLAG 0
#define WORD_PER_VAL 1
#define TYPE_FLAG 3
#define INDEX_START 4
#define SIZE_START 10

#define HEADER_VALUE 0x54 // 01010100
#define FOOTER_VALUE 0x00 // 01010100

#define START_CALIBRATION_INDEX 0

uint8_t input_information[INPUT_SIZE];

typedef struct {
    bool read_flag;
    uint8_t words_per_val;
    bool type_flag;
    uint8_t index_location;
} InfoPacket;

void write_to_hard_drive(){
    const char* filename = "binary_file.bin";
    float seconds_sum = 0;
    for (uint16_t i; i < RUN_COUNT; i = i + 1){
        FILE *file_pointer = fopen(filename, "wb");
        clock_t start = clock();
        // write to hard drive
        if (file_pointer != NULL) {
            fwrite(input_information, sizeof(uint8_t), INPUT_SIZE, file_pointer);
        }
        clock_t end = clock();
        remove(filename);
        seconds_sum = seconds_sum + (float)(end - start) / CLOCKS_PER_SEC;
    }
    seconds_sum = seconds_sum / RUN_COUNT;
    printf("Average Number of seconds passed to write to hard drive %f.\n", seconds_sum);

}

int setup_serial(){
    // open usb port
    int serial_port = open("/dev/serial0", O_RDWR);

    // setup tty and error out if failure
    if (serial_port < 0){
        perror("Port Error Check");
        return 1;
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Port Error Definition");
        return 1;
    }

    // read baud rate
    cfsetispeed(&tty, B115200);
    // write baud rate
    cfsetospeed(&tty, B115200);

    // No parity
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN | ECHONL);
    tty.c_iflag |= IGNBRK | IGNPAR;
    tty.c_iflag &= ~(BRKINT | PARMRK | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY | ISTRIP | INPCK); 
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 2;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Port Error Flags");
        return 1;
    }
    return serial_port;
}

void send_data_to_arduino(int serial_port, uint16_t packets[], size_t list_length, InfoPacket info){

    uint8_t header_message[] = {HEADER_VALUE, HEADER_VALUE};
    write(serial_port, header_message, sizeof(header_message));
    uint16_t checksum = 0;
    uint16_t info_packet = 0;
    info_packet |= info.read_flag << READ_FLAG;
    info_packet |= (info.words_per_val << WORD_PER_VAL) & 0b11;
    info_packet |= info.type_flag << TYPE_FLAG;
    info_packet |= (info.index_location << INDEX_START) & 0b111111;
    info_packet |= (list_length << SIZE_START) & 0b111111;
    checksum = checksum + info_packet;
    write(serial_port, &info_packet, sizeof(info_packet));

    for (uint8_t i = 0; i < list_length; i = i + 1){

        write(serial_port, &packets[i], sizeof(packets[i]));
        checksum = checksum + packets[i];
    }
    checksum = ~checksum;
    printf("%i", checksum);
    fflush(stdout);
    // invert checksum to use summation instead and avoid power off issue
    write(serial_port, &checksum, sizeof(checksum)); 
}

uint8_t * read_packet(uint8_t serial_port){
    uint8_t current_packet[2];
    int n = read(serial_port, current_packet, sizeof(current_packet));
    if (n < 0){
        if (errno == EAGAIN || errno == EWOULDBLOCK){
            usleep(1000);
        } else{
            perror("Read Error");
        }
    } else{
        return current_packet; 
    }
}


void main(){
    InfoPacket info_packet = {
        .read_flag = false,
        .words_per_val = 0,
        .type_flag = false,
        .index_location = START_CALIBRATION_INDEX,
    };
    // write_to_hard_drive();
    int serial_port = setup_serial();
    uint16_t packet[1] = {0x0001};
    send_data_to_arduino(serial_port, packet, 3, info_packet);

    uint8_t single_value;

    uint8_t dump[256];
    bool packet_found = false;
    tcflush(serial_port, TCIOFLUSH);
    while (true){
        // printf("Bytes Available %i", bytesavail);



        // if (bytesavail > 2){
        //     read(serial_port, current_packet, 1);
        //     if (current_packet[0] == 0x4A && current_packet[1] == 0x4A){
        //         packet_found = true;
        //     } else if (current_packet[0] == 0x22 && current_packet[1] == 0x22){
        //         packet_found = false;
        //     } else if (packet_found == true){
        //         printf("Packet Received %i.\n", current_packet[0] << 8 || current_packet[1]);
        //     }
        // }
    }
}