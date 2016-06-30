/*
 BetterSpeedBagCounter - A Better Speed Bag Counter
 6/25/2016

 License: This code is public domain you can use it all you want... Juse don't
          ask questions.... ;)

 BetterSpeedBagCounter is a modification to SparkFun's speed bag counter that
 uses an accelerometer mounted to the top of the speed bag platform to count
 the number of time the bag moves back and forth.  It's optimized to process
 acceleromerter data when the speed bag is installed ontop of speed
 bag platform only needing an accelerometer attached to the top of platform. 
 You don't have to alter the hitting surface or change out the swivel.

 I combine X/Y/Z into one vector using the sum of the absolute delta between
 the current and previous sample.  Next, two low pass rolling average filters
 are run with different lengths to generate the "magnitude" (i.e. short
 average) and "initial threshold" (i.e. long average) values.  The intial
 threshold is then offset by a "threshold scale" and limited to ensure it is
 at least a "min threshold" value.
 
 Finally, a hit is counted if the "magnitude" value is greater than the final 
 computed threshold.  The only other part of the process is a "min hit spacing"
 value that prevent hits being counter too close together.


 Hardware setup:

 An Arduino Leonardo was used to test this slgorithm connected to a MS Windows
 PC running the BetterSpeedBag Test Driver application which sends the captured
 accelerometer data to it via the serial interface and displays the results.

 This algorithm should easily run on any Arduino microcontroller.  An oscope
 was used to time the processAccelerometerData() routine running on the 
 Leonardo board and all times were less than 130us.  Thus, there is still
 plenty of processing time left for improvements (i.e. It's called
 BetterSpeedBagCounter NOT BestSpeedBagCounter)!

 */

//--------------------------------------------------------
// Structure / Type Declarations
//--------------------------------------------------------

typedef struct
{
  short x;
  short y;
  short z;
} AccelDataSample;
// Defines the structure of the accelerometer data.


//--------------------------------------------------------
// Constant Declarations
//--------------------------------------------------------

const int  LED = 13;                           // Status LED pin
const byte NUM_SHORT_SAMPLES_TO_AVG = 80;      // Number of samples to avg for the short filter.
const byte NUM_LONG_SAMPLES_TO_AVG  = 220;     // Number of samples to avg for the long filter.
const byte MIN_THRESHOLD            = 100;     // The minimum threshold to use.
const byte MAG_THRESHOLD_ADJUST     = 50;      // Value to raise the low pass value by for the threshold.
const byte MIN_HIT_SPACING          = 90;      // Minimum number of accelerometer readings between hits.
const byte SHORT_SAMPLE_DELAY       =          // Number of samples to delay the start of the short filter.
              (byte)(NUM_LONG_SAMPLES_TO_AVG - NUM_SHORT_SAMPLES_TO_AVG);


//--------------------------------------------------------
// Constant Declarations
//--------------------------------------------------------

unsigned short hitCounter = 0;         // Keeps track of the number of hits
byte initSampleCount = 0;              // The number of samples received since the last reset.
                                       //    This value is limited to 255.
byte samplesSinceLastHit = 255;        // The number of samples since the last hit was detected.
                                       //    This value is limited to 255.
AccelDataSample currentAccelData;      // The most recently received accelerometer dataset.
AccelDataSample prevAccelData;         // The previous accelerometer dataset.

byte absDeltaSumsIndex = 0;            // The location to store the next sample in the absDeltaSums
                                       //    array.
unsigned short absDeltaSums[256];      // The previous 256 calculated sum of absolute differences
                                       //    of the x, y, and z accelerometer readings.
unsigned short currentAbsDeltaSum = 0; // The sum of the absolute delta values of the x, y, and z
                                       //    axis data for the current and previous sample.
                                               
byte shortAvgStartIndex;               // The short avg. starting index in the absDeltaSums array
byte shortAvgEndIndex;                 // The short avg. ending index in the absDeltaSums array
unsigned short shortAvgAccum;          // Short avg. accumulator (16-bit)

byte longAvgStartIndex;                // The long avg. starting index in the absDeltaSums array
byte longAvgEndIndex;                  // The long avg. ending index in the absDeltaSums array
unsigned long longAvgAccum;            // Long avg. accumulator (32-bit)


//---------------------------------------------------------------
// Serial interface structures and variables for test driver.
//---------------------------------------------------------------

typedef enum
{
  MSG_RESET_CMD,
  MSG_TEST_COMPLETE_CMD,
  MSG_ACCEL_DATA,
  MSG_TEST_RESULTS_DATA,
  MSG_ACCEL_DATA_COMPLETE
} SerialMessageType;
// Defines the types of messages sent across the serial interface between the
// BetterSpeedBagCounter application and the test driver running on the PC.

#define  SERIAL_PORT  Serial1  // I could not receive data on the PC using the Serial
                               // interface on my Leonardo board, so I connected a
                               // FTDI breakout board to Serial1 (i.e. the standard
                               // DIO pins 0 and 1.

const byte SOM_BYTE = 0xA5;    // Value used to indicate the start of a serial message.

int  incomingByte;             // The byte that was read from the serial interface
                               //    or -1 if nothing available.
byte bytesReceived = 0;        // The number of bytes that have been received for the
                               //    current message.
byte* pAccelData;              // The address at which to store the next byte of 
                               //    accelerometer data received over the serial interface.


//-------------------------------------------------------------
// Function: ResetSpeedBagStateVariable()
//-------------------------------------------------------------
// This routine resets all of the Better Speed Bag Counter
// state values.
//-------------------------------------------------------------
void ResetSpeedBagStateVariable()
{
  int i;     // Loop Counter
                    
  // Set the hit counter, initial sample count, and index into
  // the absDeltaSums array to 0.
  hitCounter = 0;
  absDeltaSumsIndex = 0;
  initSampleCount = 0;
  
  // Set the samplesSinceLastHit to 255 to enable hits to be
  // immediately detected.
  samplesSinceLastHit = 255;

  // Clear the absDeltaSums array values.
  for (i = 0; i < 256; i++)
    absDeltaSums[i] = 0;

  // Clear the short average accumulator and setup the start and stop
  // indexes.  These indexes are setup to sync the short and long
  // average filters to be centered on the same sample, but allow both
  // to be a simple rolling average.
  shortAvgAccum = 0;
  shortAvgStartIndex = (byte)((NUM_LONG_SAMPLES_TO_AVG / 2) - 
                                (NUM_SHORT_SAMPLES_TO_AVG / 2));
  shortAvgEndIndex = shortAvgStartIndex;

  // Clear the long average accumulator and both the start and stop
  // indexes.  The long average process immediately!
  longAvgAccum = 0;
  longAvgStartIndex = 0;
  longAvgEndIndex = longAvgStartIndex;
}
                  
//-------------------------------------------------------------
// Function: setup()
//-------------------------------------------------------------
// This routine runs at power on and should initialize all
// pins and interfaces.
//-------------------------------------------------------------
void setup()
{
  // Set the LED pin up to be an output.
  pinMode(LED, OUTPUT);
  
  // Reset the state of the Better SpeedBag Counter.
  ResetSpeedBagStateVariable();

  // Initialize the serial interface for communication with
  // the test driver.
  SERIAL_PORT.begin(57600);
}


//-------------------------------------------------------------
// Function: loop()
//-------------------------------------------------------------
// This routine runs repetitively after the setup() function
// runs at power on.  Currently, this routine handles the
// interface with the BetterSpeedBagCounter Test Driver
// application running on a PC and calls the
// processAccelerometerData() function when a new sample has
// been received.
//
// Note: When porting the BetterSpeedBagCounter algorithm to
//       a different application, all you need is the
//       processAccelerometerData() function and the global
//       variable defined above.
//-------------------------------------------------------------
void loop()
{

  //-----------------------------------------------------
  // Serial Message Processing
  //-----------------------------------------------------

  // Grab the next byte of data from the serial interface.
  incomingByte = SERIAL_PORT.read();
  
  // Loop while data is available from the serial interface.
  while (incomingByte != -1)
  {
    bytesReceived++;

    // If this is the first byte received for the message make sure
    // it's value is SOM_BYTE otherwise throw it away!
    if ((bytesReceived == 1) &&
        (incomingByte != SOM_BYTE))
    {
      bytesReceived = 0;
    }

    // Determine if we have just received the second byte
    // of the message.  If so, determine what message we
    // received.
    if (bytesReceived == 2)
    {   
      // Determine if we have received a MSG_RESET_CMD or 
      // MSG_TEST_COMPLETE_CMD.
      if ((incomingByte == MSG_TEST_COMPLETE_CMD) ||
          (incomingByte == MSG_RESET_CMD))
      {
        // Only send the MSG_TEST_RESULTS_DATA message if we just received
        // a MSG_TEST_COMPLETE_CMD message.
        if (incomingByte == MSG_TEST_COMPLETE_CMD)
        {          
          // Send the MSG_TEST_RESULTS_DATA message with the total number
          // of hits that were detected.
          SERIAL_PORT.write(0xA5);
          SERIAL_PORT.write(MSG_TEST_RESULTS_DATA);
          SERIAL_PORT.write((hitCounter >> 8) & 0xFF);
          SERIAL_PORT.write(hitCounter & 0xFF);
        }
         
        // Reset the state of the Better SpeedBag Counter.
        ResetSpeedBagStateVariable();

        // There is no data associated with either of these messages so
        // clear the bytesReceived variable to prepare for the next message.
        bytesReceived = 0;
      }

      // Determine if this message id is for a new accelerometer sample.
      if (incomingByte == MSG_ACCEL_DATA)
      {
         // Set the pAccelData pointer to the start of the currentAccelData
         // variable, so the accelerometer data can get written into it when
         // received.
         pAccelData = (byte*)&currentAccelData;
      }
    }

    // Determine if we are in the middle of receiving an accelerometer data
    // message.
    if (bytesReceived >= 3)
    {
      // Store the received byte of accelerometer data.
      *pAccelData++ = (byte)incomingByte;

      // Determine if we have received the complete MSG_ACCEL_DATA message.
      // There is the SOM byte, Message ID, and 6 bytes of accelerometer data.
      if (bytesReceived == 8)
      {
        // Set the LED pin HIGH so that we can time the duration of the
        // processAccelerometerData() function with a oscope.
        digitalWrite(LED, HIGH);
        
        // Process the accelerometer sample and increment the hitCounter
        // if a hit was detected.
        if (processAccelerometerData())
          hitCounter++;
          
        // Set the LED pin LOW so that we can time the duration of the
        // processAccelerometerData() function with a oscope.
        digitalWrite(LED, LOW);
        
        // Send the MSG_ACCEL_DATA_COMPLETE message back to the Better
        // Speed Bag Counter Test Driver.
        SERIAL_PORT.write(0xA5);
        SERIAL_PORT.write(MSG_ACCEL_DATA_COMPLETE);
        SERIAL_PORT.write((currentAbsDeltaSum >> 8) & 0xFF);
        SERIAL_PORT.write(currentAbsDeltaSum & 0xFF);
        
        // Clear the bytesReceived variable to start processing the next
        // message.
        bytesReceived = 0;
      }
    }

    // Grab the next byte of data from the serial interface.
    incomingByte = Serial.read(); 
  }
}


//-------------------------------------------------------------
// Function: processAccelerometerData()
//-------------------------------------------------------------
// This routine processes the newly received accelerometer
// data (i.e. x, y, and z axis) held in the currentAccelData
// variable.
//-------------------------------------------------------------
boolean processAccelerometerData()
{
  int i;                            // Loop counter
  byte avgIndex;                    // Index into the absDeltaSums Array
  short xDelta, yDelta, zDelta;     // Deltas from prev reading
  unsigned short shortAvg;          // short avg. value
  unsigned short longAvg;           // long avg. value
  unsigned short threshold;         // Calculated hit threshold
  boolean hit = false;              // Hit indicator

  // Deterine if this is the very first accelerometer sample since
  // ResetSpeedBagStateVariable was called.
  if (initSampleCount == 0)
  {
    // Set the currentAbsDeltaSum for the first sample to 0 as
    // we do not have a previous sample to compare too.
    currentAbsDeltaSum = (unsigned short)0;
    absDeltaSums[absDeltaSumsIndex++] = currentAbsDeltaSum;
  }
  else
  {
    // Calculate the sum of the absolute difference between the
    // x, y, and z axis values for the current and previous
    // accelerometer samples.
    xDelta = (short)(currentAccelData.x - prevAccelData.x);
    yDelta = (short)(currentAccelData.y - prevAccelData.y);
    zDelta = (short)(currentAccelData.z - prevAccelData.z);
    currentAbsDeltaSum = (unsigned short)(abs(xDelta) + abs(yDelta) + abs(zDelta));
    absDeltaSums[absDeltaSumsIndex++] = currentAbsDeltaSum;
  }

  // Save the current accelerometer data off for use when the
  // next reading is received.
  prevAccelData = currentAccelData;

  //----------------------------------------------------------------
  // Calculate the rolling average of the absDeltaSums for the long
  // average.  We start updating the long average filter
  // immediately...  The start of the short filter is delay so that
  // both filters are centered on the same sample.  The 8-bit
  // indexes in the absDeltaSums array wrap around at every 256
  // elements!
  //----------------------------------------------------------------
  if (initSampleCount < NUM_LONG_SAMPLES_TO_AVG)
  {
    longAvgAccum += absDeltaSums[longAvgEndIndex++];
  }
  else
  {
    longAvgAccum -= absDeltaSums[longAvgStartIndex++];
    longAvgAccum += absDeltaSums[longAvgEndIndex++];
  }

  //----------------------------------------------------------------
  // Calculate the rolling average of the absDeltaSums for the
  // short average.  The start of the short filter is delay so that
  // both filters are centered on the same sample.  The 8-bit
  // indexes in the absDeltaSums array wrap around at every 256
  // elements!
  //----------------------------------------------------------------
  if (initSampleCount >= SHORT_SAMPLE_DELAY)
  {
    if (initSampleCount < (SHORT_SAMPLE_DELAY + NUM_SHORT_SAMPLES_TO_AVG))
    {
      shortAvgAccum += absDeltaSums[shortAvgEndIndex++];
    }
    else
    {
      shortAvgAccum -= absDeltaSums[shortAvgStartIndex++];
      shortAvgAccum += absDeltaSums[shortAvgEndIndex++];
    }
  }

  // Increment the initSampleCount value until it reaches 255.
  if (initSampleCount < 255)
    initSampleCount++;

  // Make sure we have initialized the absDeltaSums array with
  // enough sample to calculate the large average before we start
  // processing the hit detection.
  if (initSampleCount >= NUM_LONG_SAMPLES_TO_AVG)
  {
    shortAvg = (unsigned short)((float)shortAvgAccum / (float)NUM_SHORT_SAMPLES_TO_AVG + 0.5);
    longAvg = (unsigned short)((float)longAvgAccum / (float)NUM_LONG_SAMPLES_TO_AVG + 0.5);

    // Calculate the threshold...
    // 1) Initially set threshold to the long average.
    // 2) If the shortAvg (i.e. magnitude) is less than the
    //       threshold + thresholdOffset change to threshold
    //       to the shortAvg.  This guarentees it does not count
    //       as a hit.
    // 3) If the threshold is less than the minThreshold (i.e. 100)
    //       change it to the minThreshold.
    threshold = (unsigned short)longAvg;
    if (shortAvg < (threshold + MAG_THRESHOLD_ADJUST))
      threshold = (unsigned short)shortAvg;
    if (threshold < MIN_THRESHOLD)
      threshold = (unsigned short)MIN_THRESHOLD;

    // Increment the samplesSinceLastHit counter.  This is used to
    // prevent the same hit from being counted multiple times.
    // Prevent the 8-bit counter from wrapping around.
    if (samplesSinceLastHit < 255)
      samplesSinceLastHit++;

    // Determine if this sample exceeds its threshold
    // (i.e. potential hit).
    if (shortAvg > threshold)
    {
      // Determine if enough time has elapsed since that last
      // hit was detected.
      if (samplesSinceLastHit > MIN_HIT_SPACING)
      {
        // Set the hit indicator flag and clear the
        // samplesSinceLastHit counter.
        hit = true;
        samplesSinceLastHit = 0;
      }
    }
  }

  return hit;
}


