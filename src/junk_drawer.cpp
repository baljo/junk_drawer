#include "Microphone_PDM.h"
#include "MicWavWriter.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

SerialLogHandler logHandler;

const int ENERGY_THRESHOLD = 5000; // Adjust as needed

// Define LED pins
const int LED1_PIN = D7; // Use onboard D7 LED
const int LED2_PIN = D6; // Use an external LED connected to D6

unsigned long lastBlinkTime = 0; // For blinking LED
const unsigned long BLINK_INTERVAL = 500; // Blink every 500ms

void simulateServoActions() {
    // Simulate servo movements using LEDs
    digitalWrite(LED2_PIN, HIGH); // Turn on LED2
    delay(200);                   // Simulate movement time
    digitalWrite(LED2_PIN, LOW);  // Turn off LED2
}

void analyzeBuffer(uint8_t *buf, size_t bufSize) {
    // Analyze raw audio data
    int16_t *audioData = (int16_t *)buf; // Assuming SIGNED_16 format
    size_t samples = bufSize / sizeof(int16_t);

    int energy = 0;

    for (size_t i = 0; i < samples; i++) {
        energy += abs(audioData[i]);
    }

    energy /= samples; // Average energy

    // Print energy level to the terminal
    Log.info("Energy Level: %d", energy);

    // Blink onboard LED to indicate activity
    digitalWrite(LED1_PIN, HIGH);
    delay(50);
    digitalWrite(LED1_PIN, LOW);

    if (energy > ENERGY_THRESHOLD) {
        simulateServoActions();
    }
}

void startSampling() {
    Microphone_PDM_BufferSampling *samplingBuffer = new Microphone_PDM_BufferSampling_wav();
    samplingBuffer->withCompletionCallback([](uint8_t *buf, size_t bufSize) {
        // Process the audio buffer to detect beats
        analyzeBuffer(buf, bufSize);

        // Start the next sampling automatically
        startSampling();
    });

    Microphone_PDM::instance().bufferSamplingStart(samplingBuffer);
}

void setup() {

	Serial.begin(115200); // Initialize Serial at 9600 baud rate
    while (!Serial) { // Wait for the serial connection
        delay(10);
    }
    Particle.connect();

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

    // Start continuous sampling
    startSampling();
}

void loop() {
    // Blink the onboard LED at regular intervals to indicate the program is running
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
        lastBlinkTime = currentTime;
        digitalWrite(LED1_PIN, !digitalRead(LED1_PIN)); // Toggle LED state
    }

    // Let Microphone_PDM handle its internal operations
    Microphone_PDM::instance().loop();
}
