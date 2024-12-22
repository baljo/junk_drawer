#include "Microphone_PDM.h"
#include "MicWavWriter.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

// How long to record in milliseconds
const unsigned long RECORDING_LENGTH_MS = 20000;
const int ENERGY_THRESHOLD = 25000; // Adjust as needed

// Define LED pins
const int LED1_PIN = D7; // Use onboard D7 LED
const int LED2_PIN = D6; // Use an external LED connected to D6

// Forward declarations
void buttonHandler(system_event_t event, int data);

bool startRecording = false;


void simulateServoActions() {
    // Simulate servo movements using LEDs
    digitalWrite(LED1_PIN, HIGH); // Turn on LED1
    delay(40);                   // Simulate movement time
    digitalWrite(LED1_PIN, LOW);  // Turn off LED1

    digitalWrite(LED2_PIN, HIGH); // Turn on LED2
    delay(50);                   // Simulate movement time
    digitalWrite(LED2_PIN, LOW);  // Turn off LED2
}



void analyzeBuffer(uint8_t *buf, size_t bufSize) {
    // Analyze raw audio data
    int16_t *audioData = (int16_t *)buf; // Assuming SIGNED_16 format
    size_t samples = bufSize / sizeof(int16_t);

    int energy = 0;

    for (size_t i = 0; i < samples; i++) {
        energy += abs(audioData[i]);
        if (i % 1000 == 0) {
            Log.info("Sample %d: %d %d", i, audioData[i], energy);
        }
    }

    energy /= samples; // Average energy

    Log.info("Energy: %d", energy);

    if (energy > ENERGY_THRESHOLD) {
        simulateServoActions();
    }
}



void setup() {
    Particle.connect();

    // Register handler for the SETUP button
    System.on(button_click, buttonHandler);

    // Initialize LEDs
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);

    // Turn LEDs off initially
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);

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

void loop() {
    Microphone_PDM::instance().loop();

    if (startRecording) {
        startRecording = false;
        //digitalWrite(LED1_PIN, HIGH); // Indicate recording with LED1

        Microphone_PDM_BufferSampling *samplingBuffer = new Microphone_PDM_BufferSampling_wav();
        samplingBuffer->withCompletionCallback([](uint8_t *buf, size_t bufSize) {
            digitalWrite(LED1_PIN, LOW); // Turn off recording LED

            Log.info("done! buf=%x size=%d", (int)buf, (int)bufSize);

            // Process the audio buffer to detect beats
            analyzeBuffer(buf, bufSize);
        });
        samplingBuffer->withDurationMs(RECORDING_LENGTH_MS);

        Microphone_PDM::instance().bufferSamplingStart(samplingBuffer);
    }
}


// Button handler for the SETUP button, used to toggle recording on and off
void buttonHandler(system_event_t event, int data) {
    startRecording = true;
}