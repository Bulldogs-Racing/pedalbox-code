
#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
#endif

//-----------------------------------------------------------------

#include <ACAN_T4.h> // ACAN_T4 library

#include <bdrcanlib.h> // can id libary (custom for BDR)

//-----------------------------------------------------------------

// This sketch is a conversion of the provided FlexCAN_T4 code to use the ACAN_T4 library.

// --- ACAN_T4 Object Instantiation ---
// We select CAN2, and the constructor will use the default pins for CAN2.
// ACAN_T4 Can2(CAN2_BASE, ACAN_T4::kCan2RxPin, ACAN_T4::kCan2TxPin);

// --- Constant Definitions ---
#define AVG(a, b) ((a + b) / 2)

#define DRIVE_ENABLE_ID Drive_Enable.id
#define DRIVE_ENABLE_ON 0x01
#define DRIVE_ENABLE_LEN Drive_Enable.length

#define SET_CURRENT_ID Set_AC_Current.id
#define SET_CURRENT_LEN Set_AC_Current.length

#define CURRENT_MAX 2000
#define ANALOG_READ_MIN_ONE 2810
#define ANALOG_READ_MAX_ONE 2980

#define ANALOG_READ_MIN_TWO 3720
#define ANALOG_READ_MAX_TWO 3890

#define ACCEPTABLE_DIFF 400 // diff
#define ITERATION_TIME 10 // send msg every 10 ms

// --- Pin Definitions ---
#define POT_PIN_ONE 19
#define POT_PIN_TWO 23
#define READY_MSG 0x07FE

// --- Global Variables ---
bool ready = false;
uint8_t curr_val[SET_CURRENT_LEN];

// --- Function Prototypes ---
void drive_enable();

void setup(void) {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("CAN1-CAN2-CAN3 test") ;
  ACAN_T4_Settings settings (1000 * 1000) ;
  settings.mTxPinIsOpenCollector = true ;
  settings.mRxPinConfiguration = ACAN_T4_Settings::PULLUP_22k ;
  Serial.print ("CAN Root Clock frequency: ") ;
  Serial.print (getCANRootClockFrequency ()) ;
  Serial.println (" Hz") ;
  Serial.print ("CAN Root Clock divisor: ") ;
  Serial.println (getCANRootClockDivisor ()) ;
  Serial.print ("Bitrate prescaler: ") ;
  Serial.println (settings.mBitRatePrescaler) ;
  Serial.print ("Propagation Segment: ") ;
  Serial.println (settings.mPropagationSegment) ;
  Serial.print ("Phase segment 1: ") ;
  Serial.println (settings.mPhaseSegment1) ;
  Serial.print ("Phase segment 2: ") ;
  Serial.println (settings.mPhaseSegment2) ;
  Serial.print ("RJW: ") ;
  Serial.println (settings.mRJW) ;
  Serial.print ("Triple Sampling: ") ;
  Serial.println (settings.mTripleSampling ? "yes" : "no") ;
  Serial.print ("Actual bitrate: ") ;
  Serial.print (settings.actualBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Exact bitrate ? ") ;
  Serial.println (settings.exactBitRate () ? "yes" : "no") ;
  Serial.print ("Distance from wished bitrate: ") ;
  Serial.print (settings.ppmFromWishedBitRate ()) ;
  Serial.println (" ppm") ;
  Serial.print ("Sample point: ") ;
  Serial.print (settings.samplePointFromBitStart ()) ;
  Serial.println ("%") ;

  const uint32_t errorCode = ACAN_T4::can2.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("can2 ok") ;
  }else{
    Serial.print ("Error can2: 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // Configure analog and digital pins
  analogReadResolution(12); // set analog read to 12 bits of precision
  pinMode(POT_PIN_ONE, INPUT);
  pinMode(POT_PIN_TWO, INPUT);

  // Wait for the "ready to drive" message from the bus
  Serial.println("Waiting for READY_MSG...");
  CANMessage msg;
  while (!ready) {
    if (ACAN_T4::can2.receive (msg)) { // Poll for a message
      if (msg.id == READY_MSG) {
        ready = true;
        Serial.println("READY_MSG received. Starting drive enable.");
      }
    }
  }

  // Enable the drive system
  drive_enable();
}

void drive_enable() {
  CANMessage msg;
  msg.id = DRIVE_ENABLE_ID;
  msg.len = DRIVE_ENABLE_LEN;
  for (uint8_t j = 0; j < msg.len; j++) {
    msg.data[j] = DRIVE_ENABLE_ON; // Set the payload
  }
  ACAN_T4::can2.tryToSend (msg); // Use tryToSend for non-blocking send
}

inline int getAnalogMap(int pin_num, int lo, int hi) {
  return map(constrain(analogRead(pin_num), lo, hi), lo, hi, 0, CURRENT_MAX);
}

void mapResistanceToVal() {
  int mappedValue_1 = getAnalogMap(POT_PIN_ONE, ANALOG_READ_MIN_ONE, ANALOG_READ_MAX_ONE);
  int mappedValue_2 = getAnalogMap(POT_PIN_TWO, ANALOG_READ_MIN_TWO, ANALOG_READ_MAX_TWO);

  // If the difference between the two sensor readings is too large, send a zero torque command
  if (abs(mappedValue_1 - mappedValue_2) > ACCEPTABLE_DIFF) {
    curr_val[0] = 0;
    curr_val[1] = 0;
    return;
  }

  // Average the two values and prepare them for CAN transmission (big-endian)
  uint16_t average = AVG(mappedValue_1, mappedValue_2);
  curr_val[0] = (average >> 8) & 0xFF; // Extract the high byte
  curr_val[1] = average & 0xFF;       // Extract the low byte
}



//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gSentCount2 = 0 ;
static uint32_t gReceivedCount2 = 0 ;

static const uint32_t SEND_COUNT = 50 * 1000 ;

//-----------------------------------------------------------------

void loop() {
   if (gBlinkDate <= millis ()) {
    gBlinkDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print (", CAN2: ") ;
    Serial.print (gSentCount2) ;
    Serial.print (" / ") ;
    Serial.print (gReceivedCount2) ;
    Serial.print (" / 0x") ;
    Serial.print (ACAN_T4::can2.globalStatus (), HEX) ;
    Serial.print (" / ") ;
    Serial.print (ACAN_T4::can2.receiveBufferPeakCount ()) ;
   }
  long long start_time = millis();

  // Prepare the CAN message
  CANMessage msg;
  msg.id = SET_CURRENT_ID;
  msg.len = SET_CURRENT_LEN;

  // Get sensor values and map them to the message payload
  mapResistanceToVal();
  for (uint8_t i = 0; i < msg.len; i++) {
    msg.data[i] = curr_val[i];
  }

  // Send the message
  const bool ok = ACAN_T4::can2.tryToSend (msg) ;
  if (ok) {
    gSentCount2 += 1 ;  // Optional: keep for debugging
  }

  // Enforce a consistent loop time
  long long end_time = millis();
  if ((end_time - start_time) < ITERATION_TIME) {
    delay(ITERATION_TIME - (end_time - start_time)); // Stall until iteration time is hit
  }
}