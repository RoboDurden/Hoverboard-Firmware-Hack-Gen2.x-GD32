#include <stdint.h>

#define REMOTE_BAUD 115200		// increase default baud rate from 19200 to higher speed

#define SEND_IMU_DATA // send the IMU data with RemoteUart or RemoteUartBus
//#define MPU_6050		// active this if you do not SEND_IMU_DATA but only need imu data for your pilot code



// Schmitt clip thresholds in raw ADC/LSB units (tune these)
#define CLIP_THR_HI      3      // enter +1 if (x - baseline) >= +10
#define CLIP_THR_LO     -3      // enter -1 if (x - baseline) <= -10

//void PilotInit();

void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave);



/* Gemini 2.5 pro:

prompt:
I want to detect pedaling on a bicycle with a mpu6050 mounted to the frame on a GD32F130 mcu.
I have reduced an accelleration value from the mpu6050 to a +-1 signal with a high-pass filter and clamping.
I think that the Goertzel algorithm could do this detection.
If you agree, please write the code that detects a frequency in the range from f0 = 1 Hz to f1 = 1.5 Hz.
Sample frequency is FS = 25 Hz and sample buffer should be N = 64.
It should run on a cyclic buffer to speed up computation.
There should be an init function for me to enter the parameters like f0, f1 as float values, threshold, etc.
And a AddSample function that returns -1 until the buffer is full
and then 1 if a frequency in the range f0 - f1 is detected or 0 otherwise.
So once the buffer has been filled, i get a detection result on every new sample.
The runtime computation should be integer only.
*/

// --- Helper Functions ---

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// The maximum number of samples in the analysis window (N).
// This is used to statically allocate the buffer in the state struct.
#define MAX_SAMPLES 64

// The maximum number of frequency bins to check.
// The range f0 to f1 is converted into a set of discrete frequency bins.
#define MAX_BINS 5

// --- Fixed-Point Arithmetic Configuration ---

// Q-format for fixed-point calculations. Q14 means 14 fractional bits.
// This provides a good balance between precision and avoiding overflow.
#define FIXED_POINT_Q 14
#define FIXED_POINT_SCALE (1 << FIXED_POINT_Q)

// --- Data Structures ---

/**
 * @brief Holds the state and configuration for a Goertzel detector instance.
 */
typedef struct {
    // --- Configuration (set by init) ---
    int16_t num_bins;                      // Number of frequency bins to test
    int64_t threshold;                     // Detection threshold for magnitude squared
    int32_t coeffs[MAX_BINS];              // Pre-calculated coefficients for each bin
    uint16_t n;                            // Sample window size (must be <= MAX_SAMPLES)

    // --- State (updated by AddSample) ---
    int16_t sample_buffer[MAX_SAMPLES];    // Cyclic buffer for incoming samples
    uint16_t buffer_index;                 // Current write position in the buffer
    uint16_t samples_added;                // Counter for initial buffer fill
} GoertzelDetector;

// --- Function Prototypes ---

/**
 * @brief Initializes the Goertzel detector.
 *
 * This function calculates the necessary frequency bins and their corresponding
 * coefficients for the given frequency range. It must be called before using
 * Goertzel_AddSample.
 *
 * @param detector Pointer to the GoertzelDetector instance to initialize.
 * @param f0 The lower bound of the frequency range to detect (in Hz).
 * @param f1 The upper bound of the frequency range to detect (in Hz).
 * @param fs The sampling frequency of the input signal (in Hz).
 * @param n The number of samples to use in the analysis window (the buffer size).
 * @param threshold The threshold for the squared magnitude to trigger a detection.
 * This value needs to be tuned experimentally.
 * @return 0 on success, -1 on failure (e.g., too many bins required).
 */
int8_t Goertzel_Init(GoertzelDetector* detector, float f0, float f1, float fs, uint16_t n, int64_t threshold);

/**
 * @brief Processes a new sample and returns the detection result.
 *
 * Adds a sample to the cyclic buffer. Once the buffer is full, it runs the
 * Goertzel algorithm for each configured frequency bin on every new sample.
 *
 * @param detector Pointer to the initialized GoertzelDetector instance.
 * @param sample The new input sample. Your spec mentioned a +-1 signal, so this
 * can be a small integer value.
 * @return
 * -  1: A frequency within the target range was detected.
 * -  0: No frequency was detected (or buffer is full but signal is out of range).
 * - -1: The sample buffer is not yet full.
 */
int8_t Goertzel_AddSample(GoertzelDetector* detector, int16_t sample);

/*
// --- Example Usage ---

#include <stdio.h>

void main() {
    // --- Parameters from your request ---
    const float F0 = 1.0f;        // 1 Hz
    const float F1 = 1.5f;        // 1.5 Hz
    const float FS = 25.0f;       // 25 Hz sampling frequency
    const uint16_t N = 64;        // 64 samples buffer size

    // The threshold must be tuned based on the actual signal strength.
    // Start with a value and adjust it up or down.
    // A value of (N/2)^2 * (input_amplitude)^2 is a reasonable starting point.
    // For input of +-1, this is (32*32) = 1024. Let's try something higher.
    const int64_t THRESHOLD = 50000;

    GoertzelDetector pedaling_detector;

    // Initialize the detector
    if (Goertzel_Init(&pedaling_detector, F0, F1, FS, N, THRESHOLD) != 0) {
        printf("Failed to initialize detector.\n");
        return;
    }
    printf("Detector initialized.\n");
    printf("Checking %d frequency bins.\n", pedaling_detector.num_bins);

    // --- Simulate a signal ---
    // Let's generate a 1.2 Hz sine wave with an amplitude of 1
    float test_freq = 1.2f;
    int16_t test_signal[200];
    for (int i = 0; i < 200; i++) {
        test_signal[i] = (int16_t)(sin(2.0 * M_PI * test_freq * i / FS));
    }

    // Process the simulated signal
    for (int i = 0; i < 200; i++) {
        int8_t result = Goertzel_AddSample(&pedaling_detector, test_signal[i]);
        printf("Sample %3d, Input: %2d, Result: %2d\n", i, test_signal[i], result);
    }
}
*/