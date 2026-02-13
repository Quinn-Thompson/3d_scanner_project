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

#define HEADER_VALUE 0x5454 // 01010100
#define HOLD_BUS 0x1414


#define FLAG_INDEX 0

uint8_t input_information[INPUT_SIZE];

typedef struct {
    bool read_flag;
    uint8_t words_per_val;
    bool type_flag;
    uint8_t index_location;
} InfoPacket;

typedef struct {
    uint16_t data[1024];
    int8_t front;
    int8_t rear;
    uint8_t count; 
} Queue;

Queue uart_write_queue;

void init_queue(Queue *q) {
    q->front = 0;
    q->rear = -1;
    q->count = 0;
}

bool is_empty(Queue *q) {
    return q->count == 0;
}

bool is_full(Queue *q) {
    return q->count == 1024;
}

bool enqueue(Queue *q, uint16_t values[], size_t number_of_values) {
    if (is_full(q)) return false;
    for (uint8_t i = 0; i < number_of_values; i++){
        q->rear = (q->rear + 1) % 1024;
        q->data[q->rear] = values[i];
        q->count++;
    }
    return true;
}

bool dequeue(Queue *q, uint16_t values[], size_t number_of_values) {
    if (is_empty(q)) return false;
    for (uint8_t i = 0; i < number_of_values; i++){
        values[i] = q->data[q->front];
        q->front = (q->front + 1) % 1024;
        q->count--;
    }
    return true;
}

bool peek(Queue *q, uint8_t *value) {
    if (is_empty(q)) return false;
    *value = q->data[q->front];
    return true;
}


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
    if (!is_empty(&uart_write_queue)){
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
    uint16_t to_queue_packet[list_length + 4];
    to_queue_packet[0] = list_length + 3;
    to_queue_packet[1] = HEADER_VALUE;
    uint16_t checksum = 0;
    uint16_t info_packet = 0;
    info_packet |= info.read_flag << READ_FLAG;
    info_packet |= info.words_per_val << WORD_PER_VAL;
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

uint16_t read_words(uint8_t serial_port, uint16_t *word){
    uint8_t current_packet[2];
    int8_t response_value = read(serial_port, current_packet, sizeof(current_packet));
    if (response_value < 0){
        if (errno == EAGAIN){
            usleep(1000);
        } else{
            perror("Read Error");
            return 1;
        }
    } else {
        *word = (second_byte << 8) | first_byte;
        return 0
    }
}

uint8_t read_packet(uint8_t serial_port){
  while (ioctl(fd, FIONREAD, &bytes_available) >= 2){
      uint16_t current_word;
      read_words(serial_port, &current_word);
      
      read_buffer[packet_index] = current_word;
      if (current_word == HEADER_VALUE){
        packet_found = true;
      } else if (packet_found) {
        if (packet_index == 1) {
          data_length = ((read_buffer[packet_index] >> 10) & 0b111111);
        } if (packet_index - 1 > data_length){
          packet_index = 0;
          return 2;
        }
      } else {
        packet_index = 0;
        return 1;
      }
      packet_index += 1;
    }
  return 0;
}

void post_process_packet(){
  // process the entire packet, wehther it be single message or a data packet
  uint8_t packet_finished = read_packet();

  // if we are just a single message
  if (packet_finished == 1){
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
          noop;        
        } else {
          bus_held = false;
          write_to_array(&read_buffer[2], packet_length, index, type_flag, words_per_val / 2);
        }
      }
      
    }
}

void start_calibration(uint8_t serial_port){
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
    init_queue(&uart_write_queue);
    // write_to_hard_drive();
    uint8_t serial_port = setup_serial();
    

    uint8_t single_value;

    uint8_t dump[256];
    bool packet_found = false;
    tcflush(serial_port, TCIOFLUSH);
    while (true){
        start_calibration(serial_port);
        sleep(1);
        send_write_buffer(serial_port);
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