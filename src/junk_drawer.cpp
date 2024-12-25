#include "Microphone_PDM.h"
#include "MicWavWriter.h"
#include <Wire.h>

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

// How long to record in milliseconds
const unsigned long RECORDING_LENGTH_MS = 50;
const int ENERGY_THRESHOLD = 750; // Adjust as needed

// Define LED pins
const int LED1_PIN = D7; // Use onboard D7 LED
const int LED2_PIN = D6; // Use an external LED connected to D6

Servo myservo;                                      // create servo object to control a servo

int servo_pos = 0;                                  // variable to store the servo position


bool startRecording = false;

// NEW: Track if a 1-second sampling is currently in progress
bool isSamplingInProgress = false;



void santa2()  {
  int speed = 10;
  int wait = 1;

  for(servo_pos = 0; servo_pos < 90; servo_pos += speed)   // goes from 0 degrees to xxx degrees
  {                                                     // in steps of 1 degree
    myservo.write(servo_pos);                           // tell servo to go to position in variable 'pos'
    delay(wait);                                           // waits for the servo to reach the position
  }

  delay(wait);

  for(servo_pos = 90; servo_pos>=1; servo_pos -= speed)      // goes from xxx degrees to 0 degrees
  {                                                     // in steps of 1 degree
    myservo.write(servo_pos);                           // tell servo to go to position in variable 'pos'
    delay(wait);                                           // waits for the servo to reach the position
  }
}



void santa()  {
    int speed = 10;
    int wait = 10;

    myservo.write(130);                           // tell servo to go to position in variable 'pos'
    delay(wait);
    myservo.write(-130);                           // tell servo to go to position in variable 'pos'


}



// --------------------------------------------------------------------------
// Simulate servo actions with LED1 and LED2
// --------------------------------------------------------------------------
void simulateServoActions() {
    // Simulate servo movements using LEDs
    digitalWrite(LED1_PIN, HIGH); // Turn on LED1
    delay(40);                    // Simulate movement time
    digitalWrite(LED1_PIN, LOW);  // Turn off LED1
    
    santa();

    digitalWrite(LED2_PIN, HIGH); // Turn on LED2
    delay(50);                    // Simulate movement time
    digitalWrite(LED2_PIN, LOW);  // Turn off LED2
}


// --------------------------------------------------------------------------
// Analyze raw audio buffer to detect energy and optionally trigger servo
// --------------------------------------------------------------------------
void analyzeBuffer(uint8_t *buf, size_t bufSize) {
    // Analyze raw audio data
    int16_t *audioData = (int16_t *)buf; // Assuming SIGNED_16 format
    size_t samples = bufSize / sizeof(int16_t);

    int energy = 0;

    for (size_t i = 0; i < samples; i++) {
        energy += abs(audioData[i]);
    }

    energy /= samples; // Average energy

    // Log the average energy level
    Log.info("Average Energy Level: %d", energy);

    // Trigger "servo" (LED) actions if energy exceeds threshold
    if (energy > ENERGY_THRESHOLD) {
        santa();
    }
}


// --------------------------------------------------------------------------
// loop()
// Continuously samples 1-second chunks if startRecording == true
// --------------------------------------------------------------------------
void loop() {



    // Keep PDM driver alive
    Microphone_PDM::instance().loop();

    // If we're supposed to be recording, start a new sample if not already in progress
    if (startRecording) {
        if (!isSamplingInProgress) {
            isSamplingInProgress = true;

            // Create a new sampling buffer for WAV data (1 second)
            Microphone_PDM_BufferSampling *samplingBuffer = new Microphone_PDM_BufferSampling_wav();
            samplingBuffer->withCompletionCallback([](uint8_t *buf, size_t bufSize) {
                // Process the audio buffer to detect beats / energy
                analyzeBuffer(buf, bufSize);

                // Mark that we're free to start another 1-second sample
                isSamplingInProgress = false;
            });
            samplingBuffer->withDurationMs(RECORDING_LENGTH_MS);

            // Start PDM sampling for the next 1 second
            Microphone_PDM::instance().bufferSamplingStart(samplingBuffer);
        }
    }
    else {
        // If not recording, ensure we're not stuck in 'progress' state
        isSamplingInProgress = false;
    }
}

// --------------------------------------------------------------------------
// Button handler for SETUP button, toggles recording on/off
// --------------------------------------------------------------------------
void buttonHandler(system_event_t event, int data) {
    startRecording = !startRecording; // Toggle recording state
    if (!startRecording) {
        digitalWrite(LED1_PIN, LOW); // Turn off recording LED when stopping
        // (No direct stop method for PDM sampling, so no call here)
    } else {
        digitalWrite(LED1_PIN, HIGH); // Turn on recording LED when starting
    }
}

// --------------------------------------------------------------------------
// setup()
// --------------------------------------------------------------------------
void setup() {

    myservo.attach(D1);                                 // attaches the servo on the D2 pin to the servo object
    //Wire.begin();                                       // Initialize I2C

    santa();
    //Serial.begin(9600);                                 // Start Serial communication
    
    Particle.connect();


    // Register handler for the SETUP (Mode) button (single click)
    System.on(button_click, buttonHandler);


    // Initialize LEDs
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);

    // Turn LEDs off initially
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);

    // Initialize and start the PDM microphone
    int err = Microphone_PDM::instance()
        .withOutputSize(Microphone_PDM::OutputSize::SIGNED_16)
        .withRange(Microphone_PDM::Range::RANGE_2048)
        .withSampleRate(16000)
        .init();

    if (err) {
        Log.error("PDM decoder init err=%d", err);
    }

    err = Microphone_PDM::instance().start();
    if (err) {
        Log.error("PDM decoder start err=%d", err);
    }
}
