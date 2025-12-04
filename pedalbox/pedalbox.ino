#include <bdrcanlib.h>

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

// Modifed Hezheng Code for PedalBox Node
#define AVG(a, b) ((a + b) / 2)


// #define DRIVE_ENABLE_ID 0x0C1F
// #define DRIVE_ENABLE_ON 0x01
// #define DRIVE_ENABLE_LEN 1

// #define SET_CURRENT_ID 0x011F
// #define SET_CURRENT_LEN 2
#define mlen 8

#define CURRENT_MAX 2000
#define ANALOG_READ_MIN_ONE 2810
#define ANALOG_READ_MAX_ONE 2980

#define ANALOG_READ_MIN_TWO 3720 
#define ANALOG_READ_MAX_TWO 3890



#define ACCEPTABLE_DIFF 400 // diff
#define ITERATION_TIME 10 // send msg every 10 ms


// define pins
#define POT_PIN_ONE 19
#define POT_PIN_TWO 23
#define READY_MSG 0x07FE

void enable_drive();
bool ready = false;


void setup(void) {
  Serial.begin(115200); delay(400);
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.mailboxStatus();
  Can0.onReceive(ready_to_drive);



  analogReadResolution(12); // set analog read to 12 bits of precision
  pinMode(POT_PIN_ONE, INPUT);
  pinMode(POT_PIN_TWO, INPUT);

  while (!ready) Can0.events();
  Can0.disableFIFOInterrupt();
  Can0.onReceive(NULL);
  enable_drive();
}

void ready_to_drive(const CAN_message_t &msg) {
  if (msg.id == READY_MSG)
    ready = true;
}




void enable_drive(){
    CAN_message_t msg;
    msg.id = Drive_Enable.id;
    msg.len = Drive_Enable.length; 
    for (uint8_t j = 0; j < msg.len; j++ ) msg.buf[j] = Drive_Enable.id; // set the payload to the length
    Can0.write(msg);
}

uint8_t curr_val[mlen];


inline int getAnalogMap(int pin_num, int lo, int hi){
  return map(constrain(analogRead(pin_num), lo, hi), lo, hi, 0, CURRENT_MAX);
}

void mapResistanceToVal() {
    //Serial.println(analogRead(potPin_1));
    //Serial.println(analogRead(potPin_2));
    int mappedValue_1 = getAnalogMap(POT_PIN_ONE, ANALOG_READ_MIN_ONE, ANALOG_READ_MAX_ONE);
    int mappedValue_2 = getAnalogMap(POT_PIN_TWO, ANALOG_READ_MIN_TWO, ANALOG_READ_MAX_TWO);
    //Serial.println(mappedValue_1);
    //Serial.println(mappedValue_2);
    if (abs(mappedValue_1 - mappedValue_2) > ACCEPTABLE_DIFF) {
        curr_val[0] = 0;
        curr_val[1] = 0;
        return;
    }

    uint16_t average = AVG(mappedValue_1, mappedValue_2); 

    curr_val[0] = (average >> 8) & 0xFF; // Extract the high byte
    curr_val[1] = average & 0xFF; // extract the low byte
}



void loop() {
    long long start_time = millis();

    CAN_message_t msg;
    msg.id = Set_AC_Current.id;
    msg.len = Set_AC_Current.length;

    mapResistanceToVal(); 

    for (uint8_t i = 0; i < msg.len; i++ ) {
      Serial.println(curr_val[i]);
      msg.buf[i] = curr_val[i];
    }
    Can0.write(msg);

    long long end_time = millis();
    if ((end_time - start_time) < ITERATION_TIME) 
      delay(ITERATION_TIME - (end_time - start_time));  // stall until iteratin time is hit
}