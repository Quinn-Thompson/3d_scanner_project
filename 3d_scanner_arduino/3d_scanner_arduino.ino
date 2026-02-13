#include <Wire.h>
#include <EEPROM.h>

#define EEPROM_BIAS_WRITTEN_0_ADDR 0
#define EEPROM_BIAS_WRITTEN_1_ADDR 1

#define EEPROM_STATIC_BIAS_PLATE_ADDR 2
#define EEPROM_SERVO_BIAS_PLATE_ADDR 4
#define EEPROM_FORWARD_BIAS_PLATE_ADDR 6
#define EEPROM_BACKWARD_BIAS_PLATE_ADDR 10

#define EEPROM_STATIC_BIAS_CAMERA_ADDR 14
#define EEPROM_SERVO_BIAS_CAMERA_ADDR 16
#define EEPROM_FORWARD_BIAS_CAMERA_ADDR 18
#define EEPROM_BACKWARD_BIAS_CAMERA_ADDR 22

#define EEPROM_BIAS_CHECK_0_VAL 0x4A
#define EEPROM_BIAS_CHECK_1_VAL 0xB1

#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define FIFO_EN 0x23
#define GYRO_ZOUT_H 0x47
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define FIFO_COUNTH 0x72
#define FIFO_R_W 0x74

#define PLATE_IMU_ADDR 0x68
#define CAMERA_IMU_ADDR 0x69

#define PLATE_SERVO 1
#define CAMERA_SERVO 2

#define PLATE_ID 0
#define CAMERA_ID 1

#define HEADER_1_INDEX 0
#define HEADER_2_INDEX 1
#define SERVO_ID_INDEX 2
#define LENGTH_INDEX 3
#define COMMAND_INDEX 4
#define PARAM_START_INDEX 5
#define SMALLEST_BUS_SIZE 6

#define SERVO_MOVE_TIME_WRITE 1
#define SERVO_ID_WRITE 13
#define SERVO_POS_READ 28
#define SERVO_OR_MOTOR_MODE_WRITE 29
#define SERVO_OR_MOTOR_MODE_READ 30

#define PLATE_IMU_ADDR_PIN 2
#define CAMERA_IMU_ADDR_PIN 3
#define EN_SERIAL_PIN 4
#define EN_SERVO_PLATE 5
#define EN_SERVO_CAMERA 6
#define FLAG_INDEX 0

#define FRAME_HEADER 0x55

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define BYTE_TO_WORD(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define NO_VAL 0xFF
#define FSR (500.0 / 32768.0)
#define IMU_DIVIDER 2
#define IMU_DIVIDED 1.0f / (1000.0f / IMU_DIVIDER)

#define JUMP_POS 30
#define DYNAMIC_CALIB_RUNS 50
#define SERVO_WAIT 100000
#define GYRO_SERVO_WAIT 30

#define START_CALIBRATION 0x0001
#define HEADER_VALUE 0x5454 // 00101010
#define FAILURE_VALUE 0x6767 // 00101010
#define SUCCESS_VALUE 0x7272 // 00101010
#define BUS_HELD 0x3232 // 00101010
#define HOLD_BUS 0x1414 // 00101010


#define READ_FLAG 0
#define WORDS_PER_VAL 1
#define TYPE_FLAG 3
#define INDEX_START 4
#define SIZE_START 10

#define TICK_TO_ANGLE(tick) (tick * 240.0f) / 1000.0f
#define ANGLE_TO_TICK(angle) (angle * 1000) / 240

#define ASSERT_TRUE(cond, msg)                          \
    do {                                                \
        if ((cond)) {                                   \
            Serial.print("ERROR: ");                    \
            Serial.println(msg);                        \
            Serial.print("File: ");                     \
            Serial.println(__FILE__);                   \
            Serial.print("Line: ");                     \
            Serial.println(__LINE__);                   \
            while (1);                                  \
        }                                               \
    } while (0)

#define WARN_TRUE(cond, msg)             \
    do {                                 \
        if ((cond)) {                    \
          Serial.print("WARN: ");        \
          Serial.println(msg);           \
        }                                \
    } while (0)

#define WARN(msg)                        \
    do {                                 \
        Serial.print("WARN: ");          \
        Serial.println(msg);             \
    } while (0)

#define WARN_TRUE_MULTI(cond, msg1, var1, msg2)      \
    do {                                             \
      if ((cond)) {                                  \
        Serial.print("WARN: ");                      \
        Serial.print(msg1);                          \
        Serial.print(" ");                           \
        Serial.print(var1);                          \
        Serial.print(" ");                           \
        Serial.println(msg2);                        \
      }                                              \
    } while (0)

uint16_t current_roll = 0;
uint32_t last_read = 0;
uint16_t horizontal_angle = 0;
uint16_t vertical_angle = 0;

uint16_t started_plate_rotation = 0;
uint16_t stopped_plate_rotation = 0;
uint16_t horizontal_step_angle = 0;
uint16_t vertical_step_angle = 0;
uint16_t close_to_location = 0;
int16_t plate_start_location = 0;
bool calibration_mode = false;
bool begin_state_rotation = false;

uint8_t servo_addr[2] = {PLATE_SERVO, CAMERA_SERVO};
uint8_t servo_en_addr[2] = {EN_SERVO_PLATE, EN_SERVO_CAMERA};
uint8_t serial_addr[2] = {PLATE_IMU_ADDR, CAMERA_IMU_ADDR};
uint8_t enable_pins[2] = {EN_SERVO_PLATE, EN_SERVO_CAMERA};
int16_t static_bias[2] = {NO_VAL, NO_VAL};
int16_t servo_bias[2] = {NO_VAL, NO_VAL};
float forward_bias[2] = {NO_VAL, NO_VAL};
float negative_bias[2] = {NO_VAL, NO_VAL};

float current_gyro_rotation = 0.0f;
float current_plate_rotation = 0.0f;
float alpha = 0.4f;

bool packet_found = false;

enum ReadStatus {
    READ_OK,
    READ_CHECKSUM_ERROR,
    READ_COMMAND_ERROR,
    READ_ID_ERROR,
    READ_NO_DATA,
    READ_TIMEOUT,
};

struct InfoPacket{
    bool read_flag;
    uint8_t words_per_val;
    bool type_flag;
    uint8_t index_location;
};

struct CalibrationItems{
  bool moving = false;
  bool started_static = true;
  bool started_dynamic = false;
  bool started_reset = true;
  int16_t calib_runs = 0;
  int16_t bcalib_runs = 0;
  int16_t forwardb_runs = 0;
  int32_t scalib_total = 0;
  float fbcalib_total = 0.0f;
  float bbcalib_total = 0.0f;
  uint32_t finished_reset_time = NO_VAL;
  uint32_t finished_dynamic_time = NO_VAL;
  float current_rotation = 0.0f;
  int16_t reset_position = NO_VAL;
};

typedef struct {
    uint16_t data[64];
    int8_t front;
    int8_t rear;
    uint8_t count; 
} Queue;

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

void write_serial_buffer(uint8_t servo_id, uint8_t command, uint16_t *parameters, size_t param_length, bool only_write){
  if (servo_id != 0xFE){
     digitalWrite(servo_en_addr[servo_id-1], LOW);
  }
  UCSR1A |= (1 << TXC1);
  
  digitalWrite(EN_SERIAL_PIN, HIGH);
  uint8_t checksum = 0;
  uint8_t buffer[6 + param_length];
  
	buffer[HEADER_1_INDEX] = FRAME_HEADER;
	buffer[HEADER_2_INDEX] = FRAME_HEADER;
  buffer[SERVO_ID_INDEX] = servo_id;
  checksum = checksum + servo_id;
  buffer[LENGTH_INDEX] = 3 + param_length;
  checksum = checksum + buffer[LENGTH_INDEX];
	buffer[COMMAND_INDEX] = command;
  checksum = checksum + command;
  for (int i = 0; i < param_length / 2; i++){
    buffer[PARAM_START_INDEX + i*2] = GET_LOW_BYTE(parameters[i]);
    buffer[PARAM_START_INDEX + 1 + i*2] = GET_HIGH_BYTE(parameters[i]);
    checksum = checksum + buffer[PARAM_START_INDEX + i*2] + buffer[PARAM_START_INDEX + 1 + i*2];
  }
  buffer[PARAM_START_INDEX + param_length] = (uint8_t)(~checksum);
  
  Serial1.write(buffer, param_length + 6);
  Serial1.flush();
  digitalWrite(EN_SERIAL_PIN, LOW);
  if (only_write && servo_id != 0xFE){
    digitalWrite(servo_en_addr[servo_id-1], HIGH);
  }
}

ReadStatus read_serial_buffer(uint8_t* read_buffer, size_t buffer_size){
  uint8_t checksum = 0;
  uint8_t single_byte;
  uint8_t header_1 = Serial1.read();
  uint8_t header_2 = Serial1.read();
  uint8_t servo_id = Serial1.read();
  checksum = checksum + servo_id;
  uint8_t length = Serial1.read();
  checksum = checksum + length;
  uint8_t command = Serial1.read();
  checksum = checksum + command;
  bool warn_condition = length + 3 > buffer_size;
  WARN_TRUE_MULTI(warn_condition, "ON SERVO", servo_id, "NO DATA TO READ.");
  WARN_TRUE_MULTI(warn_condition, "LENGTH", length + 3 > buffer_size, buffer_size);
  if (warn_condition) return READ_NO_DATA;

  read_buffer[HEADER_1_INDEX] = header_1;
  read_buffer[HEADER_2_INDEX] = header_2;
  read_buffer[SERVO_ID_INDEX] = servo_id;
  read_buffer[LENGTH_INDEX] = length;
  read_buffer[COMMAND_INDEX] = command;
  
  for (int i = 0; i < (length - 2) / 2; i++){
    read_buffer[PARAM_START_INDEX + i*2] = Serial1.read();
    read_buffer[PARAM_START_INDEX + 1 + i*2] = Serial1.read();
    checksum = checksum + read_buffer[PARAM_START_INDEX + i*2] + read_buffer[PARAM_START_INDEX + 1 + i*2];
  }
  
  uint8_t read_checksum = Serial1.read();
  read_buffer[buffer_size - 1] = read_checksum;

  warn_condition = (uint8_t)(~checksum) != read_buffer[buffer_size - 1];
  WARN_TRUE_MULTI(warn_condition, "ON SERVO", servo_id, "CHECKSUM FAILED.");
  if (warn_condition) return READ_CHECKSUM_ERROR;

  return READ_OK;
}

ReadStatus write_and_read_buffer(uint8_t *read_buffer, size_t buffer_size, uint8_t servo_id, uint8_t command, uint16_t *parameters, size_t param_length){
  write_serial_buffer(servo_id, command, parameters, param_length, false);
  unsigned long start = millis();
  delayMicroseconds(43);
  
  while (Serial1.available() < buffer_size) {
    delayMicroseconds(43);

    if (millis() - start > 5) {
      WARN("TIMEOUT.");
      return READ_TIMEOUT;
    }
  }
  ReadStatus status = read_serial_buffer(read_buffer, buffer_size);
  digitalWrite(servo_en_addr[servo_id-1], HIGH);
  Serial1.flush();
  if (status == READ_OK){
    if (command != read_buffer[COMMAND_INDEX]){
      WARN("COMMAND MISMATCH BETWEEN SEND AND RETURN.");
      return READ_COMMAND_ERROR;
    } else if (servo_id != read_buffer[SERVO_ID_INDEX]){
      WARN("ID MISMATCH BETWEEN SEND AND RETURN.");
      return READ_ID_ERROR;
    } else {
      return READ_OK;
    }
  } else {
    return status;
  }
}

void servo_id_write(uint8_t servo_id, uint16_t id)
{
  uint16_t parameters[1];
  parameters[0] = id;
  size_t length = sizeof(parameters) / sizeof(uint8_t);
  write_serial_buffer(servo_id, SERVO_ID_WRITE, parameters, length, true);
}

void servo_move_time_write(uint8_t servo_id, uint16_t tick_position, uint16_t speed)
{
  uint16_t parameters[2];
  parameters[0] = tick_position;
  parameters[1] = speed;
  size_t length = sizeof(parameters) / sizeof(uint8_t);
  write_serial_buffer(servo_id, SERVO_MOVE_TIME_WRITE, parameters, length, true);
}

uint16_t servo_get_pos(uint8_t servo_id, uint8_t attempt_count){
  uint8_t return_bytes = 2;
  uint8_t read_buffer[SMALLEST_BUS_SIZE + return_bytes];
  ReadStatus status;
  for (uint8_t i = 0; i < attempt_count; i++){
    status = write_and_read_buffer(read_buffer, SMALLEST_BUS_SIZE + return_bytes, servo_id, SERVO_POS_READ, {}, 0);
    if (status == READ_OK){
      uint16_t position = (read_buffer[PARAM_START_INDEX] | read_buffer[PARAM_START_INDEX+1] << 8);
      return position;
      break;
    }
  }
}

void servo_set_motor_mode(uint8_t servo_id, uint16_t speed)
{
  uint16_t parameters[2];
  parameters[0] = 0;
  parameters[1] = speed;
  size_t length = sizeof(parameters) / sizeof(uint8_t);
  write_serial_buffer(servo_id, SERVO_OR_MOTOR_MODE_WRITE, parameters, length, true);
}

void servo_set_servo_mode(uint8_t servo_id)
{
  uint16_t parameters[1];
  parameters[0] = 1;
  size_t length = sizeof(parameters) / sizeof(uint8_t);
  write_serial_buffer(servo_id, SERVO_OR_MOTOR_MODE_WRITE, parameters, length, true);
}

bool servo_is_servo_mode(uint8_t servo_id, uint8_t attempt_count){
  uint8_t return_bytes = 4;
  uint8_t read_buffer[SMALLEST_BUS_SIZE + return_bytes];
  ReadStatus status;
  for (uint8_t i = 0; i < attempt_count; i++){
    status = write_and_read_buffer(read_buffer, SMALLEST_BUS_SIZE + return_bytes, servo_id, SERVO_OR_MOTOR_MODE_READ, {}, 0);
    if (status == READ_OK){
      bool servo_mode = read_buffer[PARAM_START_INDEX];
      return servo_mode;
      break;
    }
  }
}

Queue uart_write_queue;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial1.begin(115200);
  while (!Serial1) { ; }
  Serial2.begin(115200);
  while (!Serial2) { ; }
  delay(500);
  
  init_queue(&uart_write_queue);

  bool no_connection = true;
  bool force_calibration = true;

  // setus up servos
  pinMode(EN_SERIAL_PIN, OUTPUT);
  pinMode(EN_SERVO_PLATE, OUTPUT);
  pinMode(EN_SERVO_CAMERA, OUTPUT);

  pinMode(PLATE_IMU_ADDR_PIN, OUTPUT);
  pinMode(CAMERA_IMU_ADDR_PIN, OUTPUT);

  digitalWrite(PLATE_IMU_ADDR_PIN, LOW);
  digitalWrite(CAMERA_IMU_ADDR_PIN, HIGH);

  digitalWrite(EN_SERVO_PLATE, LOW);
  digitalWrite(EN_SERVO_CAMERA, HIGH);
  // write to any servo listening their id
  servo_id_write(0xFE, PLATE_SERVO);

  digitalWrite(EN_SERVO_PLATE, HIGH);
  digitalWrite(EN_SERVO_CAMERA, LOW);
  servo_id_write(0xFE, CAMERA_SERVO);

  digitalWrite(EN_SERVO_CAMERA, HIGH);

  for (int i = 0; i < 2; i++){
    Wire.begin();
    // wake up MPU
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission(true);

    // turn off FIFO
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(USER_CTRL);
    Wire.write(0);
    Wire.endTransmission(true);

    // reset FIFO
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(USER_CTRL);
    Wire.write(1 << 2);
    Wire.endTransmission(true);

    // enable FIFO gyroscope Z
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(FIFO_EN);
    Wire.write(1 << 4);
    Wire.endTransmission(true);

    // change the digital low pass filter so the gyro bandwith is below 42 hz
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(CONFIG);
    Wire.write(0x05);
    Wire.endTransmission(true);

    // gyroscope output rate / (1 + SMPLRT_DIV), which is 10 in this case
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(SMPLRT_DIV);
    Wire.write(IMU_DIVIDER - 1);
    Wire.endTransmission(true);

    // set the gyro to have a smaller configured Coriolis force, so the full range of integer values is 250 degrees
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(GYRO_CONFIG); 
    Wire.write(0 << 4 | 1 << 3);
    Wire.endTransmission(true);

    // set the accellerrometers external force to correspond to 2 time the gravitational force
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(ACCEL_CONFIG); // ACCEL_CONFIG
    Wire.write(0 << 4 | 0 << 3);
    Wire.endTransmission(true);

    // enable the fifo
    Wire.beginTransmission(serial_addr[i]);
    Wire.write(USER_CTRL);
    Wire.write(1 << 6);
    Wire.endTransmission(true);
  }
  if (no_connection){
    uint8_t bias_check_0 = EEPROM.read(EEPROM_BIAS_WRITTEN_0_ADDR);
    uint8_t bias_check_1 = EEPROM.read(EEPROM_BIAS_WRITTEN_1_ADDR);
    if (bias_check_0 == EEPROM_BIAS_CHECK_0_VAL && bias_check_1 == EEPROM_BIAS_CHECK_1_VAL){
      calibration_mode = false;

      static_bias[PLATE_ID] = EEPROM.read(EEPROM_STATIC_BIAS_PLATE_ADDR);
      static_bias[CAMERA_ID] = EEPROM.read(EEPROM_STATIC_BIAS_PLATE_ADDR);
      servo_bias[PLATE_ID] = EEPROM.read(EEPROM_SERVO_BIAS_PLATE_ADDR);
      servo_bias[CAMERA_ID] = EEPROM.read(EEPROM_SERVO_BIAS_PLATE_ADDR);
      forward_bias[PLATE_ID] = EEPROM.read(EEPROM_FORWARD_BIAS_PLATE_ADDR);
      forward_bias[CAMERA_ID] = EEPROM.read(EEPROM_FORWARD_BIAS_PLATE_ADDR);
      negative_bias[PLATE_ID] = EEPROM.read(EEPROM_BACKWARD_BIAS_PLATE_ADDR);
      negative_bias[CAMERA_ID] = EEPROM.read(EEPROM_BACKWARD_BIAS_PLATE_ADDR);
    }
  }

  // delay to prevent bit error write for the servos.
  delay(1000);
  
}

void step_camera_servo(float upward){
  if (upward && 65535 - vertical_step_angle > vertical_angle) {
    vertical_angle = vertical_angle + vertical_step_angle;
  } else if (vertical_angle > vertical_step_angle) {
    vertical_angle = vertical_angle + vertical_step_angle;
  }
  return vertical_angle;
}

void begin_plate_servo_rotation(uint32_t const current_time){
  begin_state_rotation = true;
  started_plate_rotation = current_time;
  stopped_plate_rotation = 0;
  current_gyro_rotation = 0.0;
  plate_start_location = servo_get_pos(PLATE_SERVO, 5); 
}

void rotate_plate_servo(uint32_t const current_time, float new_position){
  int16_t const current_position = servo_get_pos(PLATE_SERVO, 5);
  int16_t const position_difference = current_position - plate_start_location;
  current_gyro_rotation = current_gyro_rotation + read_rotation_of_imu(PLATE_ID);
  float fused_angle;
  if (-110 < current_position && current_position < 1110){
    fused_angle = alpha * current_gyro_rotation + (1.0f - alpha) * TICK_TO_ANGLE(position_difference);
  } else {
    fused_angle = current_gyro_rotation;
  }
  current_plate_rotation = current_plate_rotation + fused_angle;

  if (new_position - current_plate_rotation > 0.3 || fused_angle > 0.3){
    // create lock so whenever the plate rotation is out of bounds the time resets
    stopped_plate_rotation = current_time;
  }

  if (current_time - stopped_plate_rotation < SERVO_WAIT && current_time - started_plate_rotation > SERVO_WAIT && servo_is_servo_mode(servo_addr[PLATE_ID], 5)){
    servo_set_motor_mode(PLATE_SERVO, (uint16_t)(new_position - current_plate_rotation));
  } else {
    servo_set_servo_mode(PLATE_SERVO);
  }
}

void rotate_camera_servo(uint32_t const current_time, float new_position){
  int16_t const current_position = servo_get_pos(CAMERA_SERVO, 5);
  int16_t const position_difference = current_position - plate_start_location;
  current_gyro_rotation = current_gyro_rotation + read_rotation_of_imu(PLATE_ID);
  float fused_angle;
  if (-110 < current_position && current_position < 1110){
    fused_angle = alpha * current_gyro_rotation + (1.0f - alpha) * TICK_TO_ANGLE(position_difference);
  } else {
    fused_angle = position_difference;
  }
  current_plate_rotation = current_plate_rotation + fused_angle;
  float locational_difference = new_position - current_plate_rotation;
  if (fused_angle > 0.3){
    close_to_location = current_time;
  }

  if (current_time - close_to_location > SERVO_WAIT){
    servo_move_time_write(CAMERA_SERVO, ANGLE_TO_TICK((int16_t)current_plate_rotation), 500);
  }

  if (current_time - started_plate_rotation > SERVO_WAIT && servo_is_servo_mode(servo_addr[PLATE_ID], 5)){
    servo_move_time_write(CAMERA_SERVO, ANGLE_TO_TICK((int16_t)new_position), (uint16_t)(1000 - (new_position - current_plate_rotation) * 5));
  }
}

float read_rotation_of_imu(uint8_t part_id){
  uint16_t count = read_fifo_count(serial_addr[part_id]);
  int32_t total_gyro_z_velocity = 0;
  while (count >= 2) {
    int16_t gyro_z_velocity = read_fifo(serial_addr[part_id]);
    total_gyro_z_velocity = total_gyro_z_velocity + (gyro_z_velocity - static_bias[part_id]);
    count -= 2;
  }

  if (total_gyro_z_velocity > 0) {
    total_gyro_z_velocity = ((float)(total_gyro_z_velocity) * FSR * IMU_DIVIDED * forward_bias[part_id]);
  } else if (total_gyro_z_velocity < 0) {
    total_gyro_z_velocity = ((float)(total_gyro_z_velocity) * FSR * IMU_DIVIDED * negative_bias[part_id]);
  }

  return total_gyro_z_velocity;
}

int16_t read_int_16_bits(){
  uint8_t read_attempts = 0;
  while (Wire.available() < 2) {
    if (read_attempts > 20){
      WARN("ATTEMPTED 16 BIT READ BUT FAILED");
      return NO_VAL;
    }
  }
  return Wire.read() << 8 | Wire.read();
}

int16_t read_fifo(uint8_t serial_address){
  Wire.beginTransmission(serial_address);
  Wire.write(FIFO_R_W);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)serial_address, (uint8_t)2);
  return read_int_16_bits();
}

void acquire_current_rotation(
  uint8_t servo_address,
  uint8_t serial_address, 
  uint8_t part_id,
  CalibrationItems *calib_items
){
  uint16_t count = read_fifo_count(serial_address);
  
  while (count >= 2) {
    count -= 2;
    int16_t gyro_z_velocity = read_fifo(serial_address);
    gyro_z_velocity = gyro_z_velocity - static_bias[part_id];
    calib_items->current_rotation = calib_items->current_rotation + ((float)(gyro_z_velocity) * FSR * IMU_DIVIDED);
    // Serial.println(gyro_z_velocity);
  }
}

uint16_t read_fifo_count(uint8_t serial_address){
  Wire.beginTransmission(serial_address);
  Wire.write(FIFO_COUNTH);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)serial_address, (uint8_t)2);

  uint16_t count = (Wire.read() << 8) | Wire.read();

  if (count >= 1024) {
    reset_fifo(serial_address);
    Wire.beginTransmission(serial_address);
    Wire.write(FIFO_COUNTH);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)serial_address, (uint8_t)2);

    count = (Wire.read() << 8) | Wire.read();
  }
  return count;
}

void reset_fifo(uint8_t serial_address){
  Serial.println("Resetting Fifo");
  Wire.beginTransmission(serial_address);
  Wire.write(USER_CTRL);
  Wire.write(0 << 6);
  Wire.endTransmission(false);

  Wire.beginTransmission(serial_address);
  Wire.write(USER_CTRL);
  Wire.write(1 << 2);
  Wire.endTransmission(false);

  uint8_t fifo_reset = 0b00000100;
  while ((fifo_reset & 0b00000100) == 0b00000100){
    Wire.beginTransmission(serial_address);
    Wire.write(USER_CTRL);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)serial_address, (uint8_t)1);
    fifo_reset = Wire.read();
  }
  Serial.println("Reset Fifo");
  Wire.beginTransmission(serial_address);
  Wire.write(USER_CTRL);
  Wire.write(1 << 6);
  Wire.endTransmission(false);
}

void check_at_origin(uint8_t servo_address, CalibrationItems *calib_items, uint32_t const current_time){
  int16_t const current_position = servo_get_pos(servo_address, 5);

  if (calib_items->finished_reset_time == NO_VAL && -4 < current_position && current_position < 4) {
    Serial.println("At Zero.");
    calib_items->finished_reset_time = current_time;
  }

  // check if we have finished the reset, and 1000 milliseconds has passed since then
  if (calib_items->finished_reset_time != NO_VAL && current_time - calib_items->finished_reset_time > GYRO_SERVO_WAIT){
    Serial.println("Delay Hit.");
    calib_items->finished_reset_time = NO_VAL;
    calib_items->started_reset = false;
    if (calib_items->started_dynamic){
      float const delta_position = (float)current_position - (float)calib_items->reset_position;
      calib_items->bbcalib_total = calib_items->bbcalib_total + (calib_items->current_rotation * 1000.0f) / (delta_position * 240.0f);
      calib_items->bcalib_runs = calib_items->bcalib_runs + 1;
      Serial.print("B Delta Pos: ");
      Serial.println(delta_position);
      Serial.print("Reset Pos: ");
      Serial.println(calib_items->reset_position);
    }
    calib_items->reset_position = current_position;
    calib_items->current_rotation = 0;
  }
}

int16_t acquire_static_bias(uint8_t serial_address, CalibrationItems *calib_items){
  uint16_t count = read_fifo_count(serial_address);

  // read all values from fifo
  while (count >= 2) {
    Wire.beginTransmission(serial_address);
    Wire.write(FIFO_R_W);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)serial_address, (uint8_t)2);
    int16_t gyro_z_velocity = Wire.read() << 8 | Wire.read();
    calib_items->scalib_total = calib_items->scalib_total + gyro_z_velocity;
    calib_items->calib_runs = calib_items->calib_runs + 1;
    count -= 2;
  }

  // average all the readings from a still IMU
  if (calib_items->calib_runs >= 1024){
    int16_t const static_bias = calib_items->scalib_total / calib_items->calib_runs;
    calib_items->calib_runs = 0;
    calib_items->started_static = false;
    calib_items->started_dynamic = true;
    Serial.print("Static Bias:");
    Serial.println(static_bias);
    return static_bias;
  }
  return NO_VAL;
}

int16_t acquire_forward_bias(
  uint8_t part_id,
  uint32_t const current_time,
  CalibrationItems *calib_items
){
  int16_t const current_position = servo_get_pos(servo_addr[part_id], 5);

  if (calib_items->finished_dynamic_time == NO_VAL && JUMP_POS - 4 < current_position && current_position < JUMP_POS + 4) {
    calib_items->finished_dynamic_time = current_time;
  }

  if (calib_items->finished_dynamic_time != NO_VAL && current_time - calib_items->finished_dynamic_time > GYRO_SERVO_WAIT){
    float const delta_position = (float)current_position - (float)calib_items->reset_position;
    calib_items->fbcalib_total = calib_items->fbcalib_total + (calib_items->current_rotation * 1000.0f) / (delta_position * 240.0f);
    Serial.print("Current Rot: ");
    Serial.println(calib_items->current_rotation);

    Serial.print("Delta Pos: ");
    Serial.println(delta_position);

    Serial.print("Forward Bias Total: ");
    Serial.println(calib_items->fbcalib_total);

    calib_items->calib_runs = calib_items->calib_runs + 1;
    Serial.print("Forward Bias runs: ");
    Serial.println(calib_items->calib_runs);
    calib_items->finished_dynamic_time = NO_VAL;
    calib_items->reset_position = current_position;
    calib_items->current_rotation = 0;
    calib_items->started_reset = true;
    calib_items->moving = false;
    reset_fifo(serial_addr[part_id]);
    servo_move_time_write(servo_addr[part_id], 0, 300);
    Serial.println("Moving back to zero");
    if (calib_items->calib_runs > DYNAMIC_CALIB_RUNS){
      calib_items->started_dynamic = false;
      forward_bias[part_id] = calib_items->fbcalib_total / calib_items->calib_runs;
      negative_bias[part_id] = calib_items->bbcalib_total / calib_items->bcalib_runs;
      Serial.print("Forward Bias: ");
      Serial.println(forward_bias[part_id]);
      Serial.print("Backward Bias: ");
      Serial.println(negative_bias[part_id]);
    }
  }
  return NO_VAL;
}

bool run_gyro_calibrations(uint8_t part_id, CalibrationItems *calib_items, uint32_t current_time){

  if (calib_items->started_reset){
    acquire_current_rotation(servo_addr[part_id], serial_addr[part_id], part_id, calib_items);
    check_at_origin(servo_addr[part_id], calib_items, current_time);
  } else if (calib_items->started_static) {
    static_bias[part_id] = acquire_static_bias(serial_addr[part_id], calib_items);
  } else if (calib_items->started_dynamic){
    if (!calib_items->moving){
      Serial.println("Moving to angle");
      reset_fifo(serial_addr[part_id]);
      servo_move_time_write(servo_addr[part_id], JUMP_POS, 300);
      calib_items->moving = true;
    }
    acquire_current_rotation(servo_addr[part_id], serial_addr[part_id], part_id, calib_items);
    acquire_forward_bias(part_id, current_time, calib_items);
  } else if (forward_bias[part_id] != NO_VAL) {
    // Serial.println("Done.");
    return true;
  } 
  return false;
}

bool run_servo_calibrations(uint8_t part_id, uint32_t current_time, uint32_t *matched_time, int16_t *rotation, int16_t *servo_bias_total, uint8_t *servo_bias_runs){
  
  int16_t const current_position = servo_get_pos(servo_addr[part_id], 50);
  if (*rotation > 1000){
    servo_bias[part_id] = *servo_bias_total / *servo_bias_runs;
    Serial.println(*servo_bias_total);
    Serial.println(servo_bias[part_id]);
    servo_move_time_write(servo_addr[part_id], 0, 50);
    return true;
  }
  if (*matched_time == NO_VAL && current_position - 10 < *rotation && *rotation < current_position + 10) {
    *matched_time = current_time;
  }

  if (*matched_time != NO_VAL && current_time - *matched_time > GYRO_SERVO_WAIT){
    *matched_time = NO_VAL;
    *servo_bias_total = *servo_bias_total + (current_position - *rotation);
    *servo_bias_runs = *servo_bias_runs + 1;
    *rotation = *rotation + 30;
    servo_move_time_write(servo_addr[part_id], *rotation, 300);
  }
  return false;
}

void calibrate_gyroscopes(){
  CalibrationItems plate_calib_items;
  CalibrationItems camera_calib_items;

  servo_move_time_write(CAMERA_SERVO, 0, 50);
  servo_move_time_write(PLATE_SERVO, 0, 50);

  Serial.println("Move to 0.");

  bool acq_cam_cal = false;
  bool acq_plate_cal = false;
  bool acq_camera_servo = false;
  bool acq_plate_servo = false;

  uint32_t current_time;
  reset_fifo(PLATE_IMU_ADDR);
  reset_fifo(CAMERA_IMU_ADDR);

  int16_t plate_bias_rotation = 0;
  int16_t camera_bias_rotation = 0;

  int16_t plate_bias_total = 0;
  int16_t camera_bias_total = 0;

  uint8_t plate_bias_runs = 0;
  uint8_t camera_bias_runs = 0;

  uint32_t plate_matched_time = NO_VAL;
  uint32_t camera_matched_time = NO_VAL;

  while (!acq_cam_cal || !acq_plate_cal){
    current_time = millis();
    if (!acq_plate_servo){
      acq_plate_servo = run_servo_calibrations(PLATE_ID, current_time, &plate_matched_time, &plate_bias_rotation, &plate_bias_total, &plate_bias_runs);
    } else {
      acq_plate_cal = run_gyro_calibrations(PLATE_ID, &plate_calib_items, current_time);
    }
    if (!acq_camera_servo){
      acq_camera_servo = run_servo_calibrations(CAMERA_ID, current_time, &camera_matched_time, &camera_bias_rotation, &camera_bias_total, &camera_bias_runs);
    } else {
      acq_cam_cal = run_gyro_calibrations(CAMERA_ID, &camera_calib_items, current_time);
    }
  }

  EEPROM.update(EEPROM_BIAS_WRITTEN_0_ADDR, EEPROM_BIAS_CHECK_0_VAL);
  EEPROM.update(EEPROM_BIAS_WRITTEN_1_ADDR, EEPROM_BIAS_CHECK_1_VAL);
  EEPROM.put(EEPROM_STATIC_BIAS_PLATE_ADDR, static_bias[PLATE_ID]);
  EEPROM.put(EEPROM_SERVO_BIAS_PLATE_ADDR, servo_bias[PLATE_ID]);
  EEPROM.put(EEPROM_FORWARD_BIAS_PLATE_ADDR, forward_bias[PLATE_ID]);
  EEPROM.put(EEPROM_BACKWARD_BIAS_PLATE_ADDR, negative_bias[PLATE_ID]);
  EEPROM.put(EEPROM_STATIC_BIAS_CAMERA_ADDR, static_bias[CAMERA_ID]);
  EEPROM.put(EEPROM_SERVO_BIAS_CAMERA_ADDR, servo_bias[CAMERA_ID]);
  EEPROM.put(EEPROM_FORWARD_BIAS_CAMERA_ADDR, forward_bias[CAMERA_ID]);
  EEPROM.put(EEPROM_BACKWARD_BIAS_CAMERA_ADDR, negative_bias[CAMERA_ID]);
  
}

uint32_t test_rotate_time = 0;
uint32_t pin_low = 0;
uint8_t packet_index = 0;
uint8_t data_length = 0;
uint16_t read_buffer[32];
uint16_t data_buffer[64];
bool type_flags[64];
bool bus_held = false;



uint8_t read_packet(){
  while (Serial2.available() >= 2){
      uint8_t first_byte = Serial2.read();
      uint8_t second_byte = Serial2.read();
      uint16_t current_word = (second_byte << 8) | first_byte;
      
      read_buffer[packet_index] = current_word;
      if (current_word == HEADER_VALUE){
        packet_found = true;
      } else if (packet_found) {
        if (packet_index == 1) {
          data_length = ((read_buffer[packet_index] >> 10) & 0b111111);
        } if (packet_index - 1 > data_length){
          packet_index = 0;
          Serial.println("finished_packet");
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

void write_to_array(uint16_t data[], size_t data_length, uint8_t index, bool type_flag, uint8_t packet_size){
  while (index < data_length){
    if (packet_size == 0){
      uint8_t offset = (data[index] >> 12) & 0b1111;
      uint16_t byte_data = (data[index]) & 0xFF;
      data_buffer[index] |= (byte_data << offset);
      type_flags[index] = 0;
      index = index + 1;
    } else {
      for (uint8_t i = 0; i < packet_size; i++) {
        data_buffer[index] = data[i];
        type_flags[index] = type_flag;
        index = index + 1;
      }
    }
  }
}

void read_from_array(){
  printf("a");
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
        Serial.println("passed checksum");
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
          read_from_array();          
        } else {
          bus_held = false;
          write_to_array(&read_buffer[2], packet_length, index, type_flag, words_per_val / 2);
        }
      }
      
    }
}


void send_write_queue(){
  if (!is_empty(&uart_write_queue)){
    Serial.println("Queue not empty");
    // first word will be the length of the packet
    uint16_t packet_length[1];
    Serial.println(packet_length[0]);
    dequeue(&uart_write_queue, packet_length, 1);
    // write the actual info
    uint16_t packet[packet_length[0]];
    dequeue(&uart_write_queue, packet, packet_length[0]);
    for (uint8_t i = 0; i < packet_length[0]; i++){
      Serial2.write(packet[i] >> 8);
      Serial2.write(packet[i] & 0xFF);
    }
  }
}

void write_packet(uint16_t data[], size_t list_length, InfoPacket info){
  uint16_t to_queue_packet[list_length + 4];
  to_queue_packet[0] = list_length + 3;
  to_queue_packet[1] = HEADER_VALUE;
  uint16_t checksum = 0;
  uint16_t info_packet = 0;
  info_packet |= info.read_flag << READ_FLAG;
  info_packet |= (info.words_per_val << WORDS_PER_VAL);
  info_packet |= info.type_flag << TYPE_FLAG;
  info_packet |= (info.index_location << INDEX_START);
  info_packet |= (list_length << SIZE_START);
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

void write_dangerous_temperature(){
    InfoPacket info_packet = {
      .read_flag = false,
      .words_per_val = 0,
      .type_flag = false,
      .index_location = FLAG_INDEX,
    };
    uint8_t offset = 1;
    uint8_t length = 0;
    uint8_t data = 1;
    uint16_t packet[1] = {data | (length << 8) | (offset << 11)};
    write_packet(packet, 1, info_packet);
}

void loop() {
  uint32_t current_time = millis();
  // uint16_t packet = read_packet();
  while (true){
    write_dangerous_temperature();
    delayMicroseconds(1000);
    send_write_queue();

  }
  if (0 == START_CALIBRATION){

    calibrate_gyroscopes(); 
    calibration_mode = false;
  }
}