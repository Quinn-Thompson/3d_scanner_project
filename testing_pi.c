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


#define INPUT_SIZE 8 * 1000
#define RUN_COUNT 100

#define READ_FLAG 0
#define WORDS_PER_VAL 1
#define TYPE_FLAG 3
#define INDEX_START 4
#define SIZE_START 10

#define HEADER_VALUE 0x5454 // 01010100
#define HOLD_BUS 0x1414
#define HEADER_VALUE 0x5454
#define FAILURE_VALUE 0x6767
#define SUCCESS_VALUE 0x7272
#define BUS_HELD 0x3232

#define FLAG_INDEX 0

uint8_t input_information[INPUT_SIZE];

// pass in struct for constructing UART data word
typedef struct {
    bool read_flag;
    uint8_t words_per_val;
    bool type_flag;
    uint8_t index_location;
} InfoPacket;

// queue for holding write instructions for uart
typedef struct {
    uint16_t data[1024];
    int8_t front;
    int8_t rear;
    uint8_t count; 
} Queue;

Queue uart_write_queue;

void init_queue(Queue *q) {
    // initialize the queue placement variables
    q->front = 0;
    q->rear = -1;
    q->count = 0;
}

bool is_empty(Queue *q) {
    // check if the queue is empty
    return q->count == 0;
}

bool is_full(Queue *q) {
    // check if the queue is full
    return q->count == 1024;
}

bool enqueue(Queue *q, uint16_t values[], size_t number_of_values) {
    // queue in a list of 16 bit values to the back
    if (is_full(q)) return false;
    for (uint8_t i = 0; i < number_of_values; i++){
        q->rear = (q->rear + 1) % 1024;
        q->data[q->rear] = values[i];
        q->count++;
    }
    return true;
}

bool dequeue(Queue *q, uint16_t values[], size_t number_of_values) {
    // pop off a list of 16 bit values from the front
    if (is_empty(q)) return false;
    for (uint8_t i = 0; i < number_of_values; i++){
        values[i] = q->data[q->front];
        q->front = (q->front + 1) % 1024;
        q->count--;
    }
    return true;
}
struct timespec start, end;

void write_to_hard_drive(){
    // a test to see the average time to write to the hard drive
    const char* filename = "binary_file.bin";
    double seconds_sum = 0;
    for (uint16_t i = 0; i < RUN_COUNT; i = i + 1){
        clock_gettime(CLOCK_MONOTONIC, &start);

        FILE *file_pointer = fopen(filename, "wb");
        // write to hard drive
        if (file_pointer != NULL) {
            fwrite(input_information, sizeof(uint8_t), INPUT_SIZE, file_pointer);
            fflush(file_pointer);
            fsync(fileno(file_pointer));
        }
        clock_gettime(CLOCK_MONOTONIC, &end);
        fclose(file_pointer);
        remove(filename);

        double time_taken = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
        seconds_sum = seconds_sum + time_taken;
    }
    seconds_sum = seconds_sum / RUN_COUNT;
    printf("Average Number of seconds passed to write to hard drive %f.\n", seconds_sum);

}

int setup_serial(){
    // initialize the uart port and setup the paramaters for it
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

    // just check to make sure nothing is enabled especially canonicalization
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


void send_write_buffer(uint8_t serial_port){
    // write a packet from the front of the queue
    if (!is_empty(&uart_write_queue)){
        clock_gettime(CLOCK_MONOTONIC, &start);

        // first word will be the length of the packet
        uint16_t packet_length[1];
        dequeue(&uart_write_queue, packet_length, 1);
        // write the actual info
        uint16_t packet[packet_length[0]];
        dequeue(&uart_write_queue, packet, packet_length[0]);
        write(serial_port, packet, sizeof(packet));
        
    }
}

void setup_packet_for_sending(int serial_port, uint16_t data[], size_t list_length, InfoPacket info){
    // based on passed in info, enqueue a list of 16 bit values so it wil be sent at the next write
    uint16_t to_queue_packet[list_length + 4];
    to_queue_packet[0] = list_length + 3;
    to_queue_packet[1] = HEADER_VALUE;
    uint16_t checksum = 0;
    uint16_t info_packet = 0;
    // devlutter the information word
    info_packet |= info.read_flag << READ_FLAG;
    info_packet |= info.words_per_val << WORDS_PER_VAL;
    info_packet |= info.type_flag << TYPE_FLAG;
    info_packet |= info.index_location << INDEX_START;
    info_packet |= list_length << SIZE_START;
    checksum = checksum + info_packet;
    to_queue_packet[2] = info_packet;

    for (uint8_t i = 0; i < list_length; i = i + 1){

        to_queue_packet[i + 3] = data[i];
        checksum = checksum + data[i];
    }
    // invert checksum to use summation instead and avoid power off issue
    checksum = ~checksum;
    to_queue_packet[list_length + 3] = checksum;
    enqueue(&uart_write_queue, to_queue_packet, list_length + 4);

    
}

// global variables for now
uint8_t packet_index = 0;
uint8_t data_length = 0;
uint16_t read_buffer[32];
uint16_t data_buffer[64];
bool type_flags[64];
bool bus_held = false;
bool packet_found = false;


uint16_t read_word(uint8_t serial_port, uint16_t *word){
    // read a word from the rx buffer if there are two bytes
    uint8_t current_packet[2];
    int8_t response_value = read(serial_port, current_packet, sizeof(current_packet));
    if (response_value < 0){
        // if there is an error from the response, print it out and return 1
        if (errno == EAGAIN){
            usleep(1000);
        } else{
            perror("Read Error");
            return 1;
        }
    } else {
        *word = (current_packet[1] << 8) | current_packet[0];
        return 0;
    }
}

uint8_t read_packet(uint8_t serial_port){
    // read the entire packet if it is available in full
    uint8_t bytes_available;
    ioctl(serial_port, FIONREAD, &bytes_available);
    while (bytes_available >= 2){
        uint16_t current_word;
        read_word(serial_port, &current_word);

        read_buffer[packet_index] = current_word;
        if (current_word == HEADER_VALUE){
            // if we have a header inform further reads that it is a data packet
            packet_found = true;
        } else if (packet_found) {
            if (packet_index == 1) {
                // extract length from info word
                data_length = ((read_buffer[packet_index] >> 10) & 0b111111);
            } if (packet_index - 1 > data_length){
                // we ended, this should return an enum
                packet_index = 0;
                return 2;
            }
        } else {
            // we found a value, but it isn't for a data packet
            packet_index = 0;
            return 1;
        }
        packet_index += 1;
    }
    // no packet
    return 0;
}

void post_process_packet(uint8_t serial_port){
  // process the entire packet, wehther it be single message or a data packet
  uint8_t packet_finished = read_packet(serial_port);

  // if we are just a single message
  if (packet_finished == 1){
    clock_gettime(CLOCK_MONOTONIC, &end);
    printf("Found %i\n", read_buffer[0]);
    uint32_t seconds = end.tv_sec - start.tv_sec;
    uint32_t nseconds = end.tv_nsec - start.tv_nsec;

    if (nseconds < 0) {
        seconds --;
        nseconds += 1000000000;
    }
    printf("Time taken: %i\n", seconds * 1000000 + nseconds / 1000);
    fflush(stdout);
    if (!bus_held && read_buffer[0] == HOLD_BUS){
      bus_held = true;

      uint16_t to_queue_packet[2] = {1, BUS_HELD};
      enqueue(&uart_write_queue, to_queue_packet, 2);
    } if (bus_held && read_buffer[0] == (SUCCESS_VALUE << 8) | SUCCESS_VALUE){
      bus_held = false;
    } 
  } else if (packet_finished == 2){
    // we are a data packet
      uint16_t calculated_checksum = 0;
      // sum the data and info word
      for (uint8_t i = 1; i < data_length + 2; i++){
        calculated_checksum = calculated_checksum + read_buffer[i];
      }
      packet_found = false;
      // get the last word, it will be the reported inverted checksum, it should add with the checksum to be all 1s
      if (calculated_checksum + read_buffer[data_length + 2] != 0xFFFF){
        // tell the raspberry pi we have a bad packet
        uint16_t to_queue_packet[2] = {1, FAILURE_VALUE};
        enqueue(&uart_write_queue, to_queue_packet, 2);
      } else {
        // we have a good packet
        uint16_t to_queue_packet[2] = {1, SUCCESS_VALUE};
        enqueue(&uart_write_queue, to_queue_packet, 2);
        // retreive individual items from info word
        bool read_flag = read_buffer[1] & 0b1;
        bool type_flag = (read_buffer[1] >> TYPE_FLAG) & 0b1;
        uint8_t index = (read_buffer[1] >> INDEX_START) & 0b111111;
        uint8_t words_per_val = (read_buffer[1] >> WORDS_PER_VAL) & 0b11;
        uint8_t packet_length = read_buffer[1] >> SIZE_START;
        // calculate number of bytes per data point
        uint8_t base = 2;
        uint8_t bytes_per_packet = 1;
        while (words_per_val > 0) {
            if (words_per_val % 2 == 1)
                bytes_per_packet *= base;

            base *= base;
            words_per_val /= 2;
        }

        // check the set flag for reading the array or writing to it
        if (read_flag){
            ;
        } else {
          bus_held = false;
          ;  
        //   write_to_array(&read_buffer[2], packet_length, index, type_flag, words_per_val / 2);
        }
      }
      
    }
}

void start_calibration(uint8_t serial_port){

    // inform the arduino that it should start calibration as warmup is done
    InfoPacket info_packet = {
        .read_flag = false,
        .words_per_val = 0,
        .type_flag = false,
        .index_location = FLAG_INDEX,
    };
    uint8_t offset = 0;
    uint8_t length = 0;
    uint8_t data = 1;
    uint16_t packet[1] = {data | (length << 8) | (offset << 11)};
    setup_packet_for_sending(serial_port, packet, 1, info_packet);
}

void main(){
    // write_to_hard_drive();

    init_queue(&uart_write_queue);
    uint8_t serial_port = setup_serial();
    
    uint8_t single_value;

    uint8_t dump[256];
    bool packet_found = false;
    tcflush(serial_port, TCIOFLUSH);
    while (true){
        start_calibration(serial_port);
        sleep(1);
        send_write_buffer(serial_port);
        post_process_packet(serial_port);

    }
}