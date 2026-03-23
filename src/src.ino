// ================= Libraries =================
#include <Arduino.h>
#include "driver/adc.h"
#include "fir_new_2_1000hz.h"
#include <math.h>

// ================= ΡΥΘΜΙΣΕΙΣ =================
#define SAMPLE_RATE         1000                // Sampling Frequency
#define ADC_CH              ADC1_CHANNEL_6      // GPIO34
#define DC_SHIFT            5                   // k = 1/32, for dc remover
#define GOERTZEL_N          2048                // Number of samples proccessed by Goertzel algorithm
#define GOERTZEL_STEP       256                 // Number of samlples for decision algorithm (overlap = GOERTZEL_N - GOERTZEL_STEP)
#define TH                  0.5f                // fixed threshold for decision

// --- Goertzel parameters ---
const int NUM_FREQ = 4; //number of computed frequencies
float target_freq[NUM_FREQ] = {10.0, 12.0, 20.0, 24.0}; 

// ================= TIMER ISR =================
hw_timer_t *timer = NULL;
volatile int16_t adcSample = 0;
volatile bool sampleReady = false;

void IRAM_ATTR onTimer()
{
    if (!sampleReady) {
        adcSample = (int16_t)adc1_get_raw(ADC_CH); //Channel 1 for ADC
        sampleReady = true; //Flag for ready sample
    }
}

void initTimer(){
    adc1_config_width(ADC_WIDTH_BIT_12); 
    adc1_config_channel_atten(ADC_CH, ADC_ATTEN_DB_11); //Input signal is in range (0-3V), so the attenuation is 11 dB

    timer = timerBegin(1000000);          // set the prescaler to 1MHz
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, SAMPLE_RATE, true, 0); //set the interrupt timer to 1ms
}

// ================= DC REMOVER =================
int32_t dc_est = 0;   // init the dc the algorithm detects

inline int16_t removeDC(int16_t x)
{
    // x is 12-bit ADC (0-4095)
    // trnasfer the data to 32 bit data for better resolution on the next computes
    int32_t temp = x;

    // dc_est = dc_est + k*(x - dc_est)
    dc_est +=  (temp - dc_est) >> DC_SHIFT ; //accumulator

    // AC output
    int32_t y = (temp - dc_est);

    // saturation
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;

    return (int16_t)y; //12 bit information to int16
}

// ================= FIR FILTER =================
static int16_t delayLine[coeffLen] = {0}; //delayLine buffer to store some previous data, so the fir filter has memory and doesnt reset every new porcess
static size_t firWriteIdx = 0;

inline int16_t FIR_processSample(int16_t x)
{

    delayLine[firWriteIdx] = x;

    int32_t acc = 0;
    size_t idx = firWriteIdx;

    for (size_t k = 0; k < coeffLen; k++) {
        acc += (int32_t)delayLine[idx] * (int32_t)coeff[k];
        idx = (idx == 0) ? coeffLen - 1 : idx - 1;
    }

    firWriteIdx++;
    if (firWriteIdx >= coeffLen)
        firWriteIdx = 0;

    // Q28 → Q12
    acc >>= (28 - 12);

    // Saturation (Q12)
    if (acc >  32768) acc =  32768;
    if (acc < -32768) acc = -32768;

    return (int16_t)acc;
}

// ================= GOERTZEL ALGORITHM =================
int32_t coeffGoertzel[NUM_FREQ];
//Goertzel algorithm is a 2 taps IIR filter, s1, s2 is the memory of filter
int32_t s1[NUM_FREQ] = {0};
int32_t s2[NUM_FREQ] = {0};

int goertzelCount = 0; // cycle data to compute the new result
int bufferIdx = 0;  //counter of goertzel algorithm resolution

int16_t goertzelBuffer[GOERTZEL_N] = {0};

// ================= INIT GOERTZEL =================
void initGoertzel() {
    for (int i = 0; i < NUM_FREQ; i++) {
        float omega = 2.0f * M_PI * target_freq[i] / SAMPLE_RATE;
        coeffGoertzel[i] = roundf(32767 * (2.0f * cosf(omega)));
        s1[i] = 0;
        s2[i] = 0;
    }
    goertzelCount = 0;
    bufferIdx = 0;
}

// ================= GOERTZEL ALGORITHM =================
void processGoertzelSampleStep(int16_t sample, int32_t mags[NUM_FREQ], bool &newResult) {
    newResult = false;

    goertzelBuffer[bufferIdx++] = sample;
    goertzelCount++;

    if (bufferIdx >= GOERTZEL_N)
        bufferIdx = 0;

    if (goertzelCount >= GOERTZEL_STEP) {

        for (int f = 0; f < NUM_FREQ; f++) {
            int32_t s1_local = 0;
            int32_t s2_local = 0;
            int32_t coeff = coeffGoertzel[f];

            for (int n = 0; n < GOERTZEL_N; n++) {
                int16_t x = goertzelBuffer[(bufferIdx + n) % GOERTZEL_N];
                int64_t prod = ((int64_t)coeff * s1_local) >> 15;
                int32_t s0 = x + (int32_t)prod - s2_local;
                s2_local = s1_local;
                s1_local = s0;
            }

            //use floating point numbers for better resolution
            float omega = 2.0f * M_PI * target_freq[f] / SAMPLE_RATE;
            float real = s1_local - cosf(omega) * s2_local;
            float imag = sinf(omega) * s2_local;
            mags[f] = (int32_t)sqrtf(real*real + imag*imag);
        }

        goertzelCount = 0;
        newResult = true; //flag for new result
    }
}

// ================= MAD DECISION =================
#define MAD_WIN 25 //Counter to compute the new MAD 
#define K_MAD   0.7f //Threashold for MAD classifier

float scoreBuffer[MAD_WIN]; 
int scoreIdx = 0;
int scoreCount = 0;

// helper for median
float computeMedian(float *arr, int n) {
    float temp[MAD_WIN];
    memcpy(temp, arr, n * sizeof(float));

    // simple insertion sort (n μικρό)
    for (int i = 1; i < n; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j+1] = temp[j];
            j--;
        }
        temp[j+1] = key;
    }

    if (n % 2 == 0)
        return 0.5f * (temp[n/2 - 1] + temp[n/2]);
    else
        return temp[n/2];
}

float computeMAD(float *arr, int n, float median) {
    float dev[MAD_WIN];
    for (int i = 0; i < n; i++)
        dev[i] = fabs(arr[i] - median); //Meadian = |x_i-Meadian(x)|

    return computeMedian(dev, n);
}


// ================= SETUP =====================
void setup()
{
    Serial.begin(921600);
    initTimer();
    //initHann();
    initGoertzel();
    //pinMode(22, OUTPUT);
    //pinMode(23, OUTPUT);
    
}


// ================= LOOP =================
int32_t mags[NUM_FREQ];
static int decisionArgmax = 0;
static int decisionMAD = 0;

void loop()
{
    if (sampleReady) {

        noInterrupts();
        int16_t x = adcSample;
        sampleReady = false;
        interrupts();

        int16_t x_ac = removeDC(x);
        int16_t y = FIR_processSample(x_ac);

        bool newResult = false;
        processGoertzelSampleStep(y, mags, newResult);

        if (newResult) {
            float power0 = (float)mags[0]*mags[0] + (float)mags[2]*mags[2];
            float power1 = (float)mags[1]*mags[1] + (float)mags[3]*mags[3];

            float score = logf(power1 + 1.0f) - logf(power0 + 1.0f);

            // update scoreBuffer
            scoreBuffer[scoreIdx++] = score;
            if (scoreIdx >= MAD_WIN) scoreIdx = 0;
            if (scoreCount < MAD_WIN) scoreCount++;

            // argmax
            decisionArgmax = (power1 > power0) ? 1 : 0;

            // MAD (computes the MAD for MAD_WIN=25 samples, but wait for the first)
            if (scoreCount >= 5) { // Wait for buffer to collect the first 5 samples, for more statistical stability 
                float med = computeMedian(scoreBuffer, scoreCount);
                float mad = computeMAD(scoreBuffer, scoreCount, med);

                if (score > med + K_MAD * mad)
                    decisionMAD = (score > 0) ? 1 : 0;
                else if (score < med - K_MAD * mad)
                    decisionMAD = (score > 0) ? 1 : 0;
                // αλλιώς κράτα το lastDecisionMAD
            }

        }

        // Output κάθε sample για να βλέπεις το σήμα στο Serial Plotter
        float vout = y * (3.3f / 4095.0f);
        Serial.print(vout, 3);
        Serial.print(",");
        Serial.print(decisionArgmax);
        Serial.print(",");
        Serial.println(decisionMAD);
    }
}