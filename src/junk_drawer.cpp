/*
 * This program analyzes audio data from a microphone, performs FFT to extract frequency components,
 * and visualizes the audio spectrum using NeoPixel LEDs. It also activates a servo motor when certain
 * frequency bins have the largest magnitude.
 * 
 * Author: Thomas Vikström
 * Date: 30.12.2024 19:44
 */

#include "Microphone_PDM.h"
#include "MicWavWriter.h"
#include <Wire.h>
#include "PlainFFT.h"
#include "neopixel.h"

// Enable system threading and set system mode to semi-automatic
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);


/* ======================= prototypes =============================== */

void colorAll(uint32_t c, uint16_t wait);
void colorWipe(uint32_t c, uint8_t wait);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);
uint32_t Wheel(byte WheelPos);


// IMPORTANT: Set pixel COUNT, PIN and TYPE
#if (PLATFORM_ID == 32)
#define PIXEL_PIN SPI1
#else 
#define PIXEL_PIN D3
#endif

// Define the number of pixels and the type of NeoPixel
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B

// Create an instance of Adafruit_NeoPixel with the defined parameters
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// Initialize a serial log handler for logging
SerialLogHandler logHandler;

// How long to record in milliseconds
const unsigned long RECORDING_LENGTH_MS = 20;
const int ENERGY_THRESHOLD = 270; // Adjust as needed

const double SAMPLE_RATE = 16000.0; // Sample rate of the audio signal

// Define LED pins
const int LED1_PIN = D7;        // Use onboard D7 LED
const int LED2_PIN = D6;        // Use an external LED connected to D6

Servo myservo;                  // create servo object to control a servo

int servo_pos = 0;              // variable to store the servo position

bool startRecording = false;    // Track if recording is active

// Track if a sampling is currently in progress
bool isSamplingInProgress = false;

PlainFFT FFT;                // Initialize PlainFFT object

const size_t FFT_SIZE = 128; // Increase the number of samples for FFT
double vReal[FFT_SIZE];      // Create an array for the real part of the FFT
double vImag[FFT_SIZE];      // Create an array for the imaginary part of the FFT


// --------------------------------------------------------------------------
// Hamming window function, reduces spectral leakage
// --------------------------------------------------------------------------
void applyHammingWindow(double *data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        data[i] *= 0.54 - 0.46 * cos(2 * M_PI * i / (size - 1));
    }
}


// --------------------------------------------------------------------------
// Move the servo motor to simulate drumming
// --------------------------------------------------------------------------
void drum2() {      

    int speed = 15;
    int wait = 5;

    for (servo_pos = 0; servo_pos < 90; servo_pos += speed) {   
        myservo.write(servo_pos);
        delay(wait);
    }

    delay(wait);

    for (servo_pos = 90; servo_pos >= 1; servo_pos -= speed) {
        myservo.write(servo_pos);
        delay(wait);
    }
}


// --------------------------------------------------------------------------
// Move the servo motor to simulate drumming
// --------------------------------------------------------------------------
void drum1() {
    int speed = 10;
    int wait = 30;

    myservo.write(130); // tell servo to go to position
    delay(wait);
    myservo.write(-130); // tell servo to go to position
}


// --------------------------------------------------------------------------
// Simulate servo actions with LED1 and LED2
// --------------------------------------------------------------------------
void simulateServoActions() {
//    Log.info("Simulating servo actions");
    // Simulate servo movements using LEDs
    digitalWrite(LED1_PIN, HIGH); // Turn on LED1
    delay(40);                    // Simulate movement time
    digitalWrite(LED1_PIN, LOW);  // Turn off LED1
    
    drum1();

    digitalWrite(LED2_PIN, HIGH); // Turn on LED2
    delay(50);                    // Simulate movement time
    digitalWrite(LED2_PIN, LOW);  // Turn off LED2
}


// --------------------------------------------------------------------------
// Interpolate between two colors based on a fraction
//--------------------------------------------------------------------------
uint32_t interpolateColor(uint32_t color1, uint32_t color2, float fraction) {
    uint8_t r1 = (color1 >> 16) & 0xFF;
    uint8_t g1 = (color1 >> 8) & 0xFF;
    uint8_t b1 = color1 & 0xFF;

    uint8_t r2 = (color2 >> 16) & 0xFF;
    uint8_t g2 = (color2 >> 8) & 0xFF;
    uint8_t b2 = color2 & 0xFF;

    uint8_t r = r1 + fraction * (r2 - r1);
    uint8_t g = g1 + fraction * (g2 - g1);
    uint8_t b = b1 + fraction * (b2 - b1);

    return (r << 16) | (g << 8) | b;
}


// --------------------------------------------------------------------------
// Analyze raw audio buffer to detect energy and optionally trigger servo
// --------------------------------------------------------------------------
void analyzeBuffer(uint8_t *buf, size_t bufSize) {
    //Log.info("Analyzing buffer...");
    //Log.info("\033[2J\033[H");                // Clear the terminal screen

    // Analyze raw audio data
    int16_t *audioData = (int16_t *)buf;        // Assuming SIGNED_16 format
    size_t samples = bufSize / sizeof(int16_t); // Calculate the number of samples

    if (samples > FFT_SIZE) {
        samples = FFT_SIZE;                     // Limit the number of samples to FFT_SIZE
    }

    int energy = 0;

    // Prepare data for FFT
    for (size_t i = 0; i < samples; i++) {
        vReal[i] = audioData[i];                // Copy audio data to the real part
        vImag[i] = 0;                           // Clear the imaginary part
        energy += abs(audioData[i]);            // Calculate the energy
    }

    energy /= samples;                          // Average energy

    // Apply Hamming window to reduce spectral leakage
    applyHammingWindow(vReal, samples);

    // Perform FFT
    //Log.info("Performing FFT...");
    FFT.compute(vReal, vImag, samples, FFT_FORWARD);    // Perform FFT
    FFT.complexToMagnitude(vReal, vImag, samples);      // Convert complex to magnitude

    // Log the average energy level
    //Log.info("Average Energy Level: %d", energy);

    size_t nr_of_bins = 12;                     // Number of bins to display

    // Select bins to cover a wide range of frequencies in pop music
    size_t selectedBins[nr_of_bins] = {3, 5, 7, 9, 11, 14, 17, 20, 23, 26, 29, 32}; // Adjust as needed

    double maxMagnitude = 0;                    // Maximum magnitude
    size_t maxBinIndex = 0;                     // Index of the maximum magnitude
    const double noiseFloorThreshold = 180.0;   // Threshold to ignore noise

    // Find the maximum magnitude for scaling, ignoring noise
    for (size_t i = 0; i < nr_of_bins; i++) {
        size_t bin = selectedBins[i];
        if (vReal[bin] > noiseFloorThreshold && vReal[bin] > maxMagnitude) {    // Ignore noise
            maxMagnitude = vReal[bin];                                          // Update the maximum magnitude
            maxBinIndex = i;                                                    // Update the index of the maximum magnitude
        }
    }

    // Check if the bin magnitudes are over the threshold, then drum
    if (vReal[2] > 300) {
        drum1();
    }

   if (vReal[10] > 300) {
        drum2();
    }

    //Log.info("Max Magnitude: %f", maxMagnitude);

    // Define a dynamic threshold as a fraction of the maximum magnitude
    const double dynamicThreshold = maxMagnitude * 0.1; // 10% of the maximum magnitude


    // Normalize the magnitudes, ignoring values below the dynamic threshold
    if (maxMagnitude > 0) {
        for (size_t i = 0; i < nr_of_bins; i++) {
            size_t bin = selectedBins[i];
            if (vReal[bin] > dynamicThreshold) {
                vReal[bin] /= maxMagnitude;             // Normalize the magnitude
            } else {
                vReal[bin] = 0;                         // Set to zero if below threshold
            }
        }


        // Apply a power function to enhance brightness variation
        const double gamma = 8.0; // Adjust gamma value to control contrast
        for (size_t i = 0; i < nr_of_bins; i++) {
            size_t bin = selectedBins[i];           // Get the bin index
            vReal[bin] = pow(vReal[bin], gamma);    // Apply gamma correction
        }


        // Define base colors for the gradient
        uint32_t colorStart = strip.Color(255, 0, 0);   // Red
        uint32_t colorEnd = strip.Color(0, 0, 255);     // Blue

         // Minimum brightness threshold
        const float minBrightness = 0.01;               // 1% brightness

        // Update NeoPixel LEDs based on the magnitudes
        for (size_t i = 0; i < nr_of_bins; i++) {
            size_t bin = selectedBins[i];               // Get the bin index
            float fraction = vReal[bin];                // Use the normalized magnitude as the fraction
            uint32_t color = interpolateColor(colorStart, colorEnd, (float)i / (nr_of_bins - 1)); // Fixed color for each LED

            uint8_t r = (color >> 16) & 0xFF;           // Extract the red component
            uint8_t g = (color >> 8) & 0xFF;            // Extract the green component
            uint8_t b = color & 0xFF;                   // Extract the blue component

            double_t maxBrightness = 0.95; // Maximum brightness is 95%
            // Adjust brightness based on the magnitude with a minimum threshold and scale to 95%
            r = (uint8_t)(r * (fraction * (1.0 - minBrightness) + minBrightness) * maxBrightness);  // Scale the red component
            g = (uint8_t)(g * (fraction * (1.0 - minBrightness) + minBrightness) * maxBrightness);  // Scale the green component
            b = (uint8_t)(b * (fraction * (1.0 - minBrightness) + minBrightness) * maxBrightness);  // Scale the blue component

            strip.setPixelColor(i, strip.Color(r, g, b));   // Set the color of the LED
        }

        strip.show();                                       // Update the LEDs

        // Print the vertical chart
        const int chartHeight = 10;                         // Height of the chart in characters
        for (int row = chartHeight; row > 0; row--) {
            std::string rowStr;
            for (size_t i = 0; i < nr_of_bins; i++) {
                size_t bin = selectedBins[i];
                int barHeight = (int)((vReal[bin]) * chartHeight);  // Calculate the bar height
                if (barHeight >= row) {
                    rowStr += "| ";                                 // Add a bar to the row
                } else {
                    rowStr += "  ";                                 // Add an empty space to the row
                }
            }
            //Log.info(rowStr.c_str());                     // Print the entire row at once, uncomment to print
        }

        // Print the frequency labels
        std::string labelStr;                               // Create a string for the frequency labels
        for (size_t i = 0; i < nr_of_bins; i++) {
            size_t bin = selectedBins[i];
            double frequency = bin * (SAMPLE_RATE / FFT_SIZE);  // Calculate the frequency for each bin
            labelStr += String::format("%.1f Hz ", frequency);  // Add the frequency to the string
        }
        //Log.info(labelStr.c_str()); // Print the frequency labels
    // } else {
    //     Log.warn("Max Magnitude is zero, skipping normalization and LED update.");
    }

}



// --------------------------------------------------------------------------
// Button handler for SETUP button, toggles recording on/off
// --------------------------------------------------------------------------
void buttonHandler(system_event_t event, int data) {
    Log.info("Button pressed");
    startRecording = !startRecording;           // Toggle recording state
    if (!startRecording) {
        digitalWrite(LED1_PIN, LOW);            // Turn off recording LED when stopping
        // (No direct stop method for PDM sampling, so no call here)
    } else {
        digitalWrite(LED1_PIN, HIGH);           // Turn on recording LED when starting
    }
}


// --------------------------------------------------------------------------
// setup()
// --------------------------------------------------------------------------
void setup() {
    Log.info("Setup started");

    // Initialize NeoPixel strip
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Initialize the servo motor
    myservo.attach(D1); // attaches the servo on the D1 pin to the servo object

    // Connect to the Particle cloud
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
        .withOutputSize(Microphone_PDM::OutputSize::SIGNED_16)      // 16-bit signed output
        .withRange(Microphone_PDM::Range::RANGE_2048)               // 12-bit range
        .withSampleRate(SAMPLE_RATE)                                // 16 kHz sample rate
        .init();

    if (err) {
        Log.error("PDM decoder init err=%d", err);
    }

    err = Microphone_PDM::instance().start();
    if (err) {
        Log.error("PDM decoder start err=%d", err);
    }

    Log.info("Setup completed");
}




// --------------------------------------------------------------------------
// loop()
// Continuously samples chunks if startRecording == true
// --------------------------------------------------------------------------
void loop() {
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
            samplingBuffer->withDurationMs(RECORDING_LENGTH_MS);            // Set the recording length

            // Start PDM sampling for the next x second
            Microphone_PDM::instance().bufferSamplingStart(samplingBuffer);
        }
    } else {
        // If not recording, ensure we're not stuck in 'progress' state
        isSamplingInProgress = false;
    }
}

// --------------------------------------------------------------------------
// Below functions are not used in the program, but can be used to create visual effects
// --------------------------------------------------------------------------

// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint16_t wait) {
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

// Fill the dots one after the other with a color, wait (ms) after each one
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout, then wait (ms)
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout, then wait (ms)
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) { // 1 cycle of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
