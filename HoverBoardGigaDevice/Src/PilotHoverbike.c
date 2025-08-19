#include "../Inc/defines.h"

#ifdef PILOT_HOVERBIKE


#include "../Inc/it.h"
#include "../Inc/mpu6050.h"

extern uint32_t iBug;
extern uint8_t iBug8;
extern uint8_t bPilotTimeout;
extern MPU_Data mpuData;	
extern float currentDC; 									// global variable for current dc
extern int32_t speed;

int16_t iGoert1=0, iGoert2=0, iGoert3=0;


// -------------- Deepseek begin
#include <stdint.h>

typedef struct {
    int16_t prev_x;    // Previous input (x[n-1])
    int16_t prev_y;    // Previous output (y[n-1])
    int16_t b0;  // Feedforward coefficient (Q15)
    int16_t b1;  // Feedforward coefficient (Q15)
    int16_t a1;  // Feedback coefficient (Q15)
} IIR_HPF;

// Initialize filter structure with coefficients
void init_iir_hpf(IIR_HPF *filter) {
    filter->prev_x = 0;
    filter->prev_y = 0;
    filter->b0 = 29476;   // (1+alpha)/2 = 0.8995 in Q15
    filter->b1 = -29476;  // -0.8995 in Q15
    filter->a1 = 26183;   // alpha = 0.799 in Q15
}

// Process one sample through the high-pass filter
int16_t iir_hpf_update(IIR_HPF *filter, int16_t input) {
    // Calculate accumulator (32-bit to prevent overflow)
    int32_t acc = (int32_t)filter->b0 * (int32_t)input;
    acc += (int32_t)filter->b1 * (int32_t)filter->prev_x;
    acc += (int32_t)filter->a1 * (int32_t)filter->prev_y;
    
    // Round to nearest and convert back to Q0 (add 0.5 in Q15)
    acc += (1 << 14);         // Add rounding constant
    int16_t output = (int16_t)(acc >> 15);  // Convert Q15 to integer
    
    // Update filter state
    filter->prev_x = input;  // Store current input for next iteration
    filter->prev_y = output; // Store current output for feedback
    
    return output;
}

//----------- Deepseek end


#ifdef GOERTZEL

#include <stdint.h>
#include <math.h>

/*====================== Configuration ======================*/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_N      256      // max window length you allow
#define DEFAULT_N  64       // initial window length (can change in goertzel_init)
#define DEFAULT_FS 25.0f    // Hz

// Optional: right-shift input samples by S bits to avoid overflow when N is large.
// For N <= 128 you can often leave this at 0. For N near 256, consider 1 or 2.
#ifndef INPUT_SHIFT
#define INPUT_SHIFT 0
#endif

/*====================== State ==============================*/
static int16_t  gBuf[MAX_N];
static int      gN         = DEFAULT_N;
static int      gBufIdx    = 0;
static int      gBufFull   = 0;
static float    gFs        = DEFAULT_FS;

static int32_t  gCoeff1_q15 = 0;   // Q15 for f1 (e.g., 1.0 Hz)
static int32_t  gCoeff2_q15 = 0;   // Q15 for f2 (e.g., 1.5 Hz)
static uint32_t gThreshold  = 50000; // tune this for your setup
                                     // Note: power roughly scales with N^2

/*====================== Helper: Q15 Coeff ==================*/
/*
  Make the Q15 coefficient for runtime Goertzel:
  coeff = 2*cos(2*pi*f/Fs) scaled by 32768 (Q15), stored in int32_t.
  This is called ONLY at init (float allowed here), runtime is integer-only.
*/
static int32_t goertzel_coeff(float fFreq) {
    float omega  = 2.0f * (float)M_PI * (fFreq / gFs);
    float c      = 2.0f * cosf(omega);           // in [-2, 2]
    // Convert to Q15 (1.0 -> 32768). 2.0 -> 65536 fits in int32_t.
    float scaled = c * 32768.0f;
    // Round-to-nearest:
    int32_t q15  = (int32_t)lrintf(scaled);
    // Clamp for safety (range a tad beyond Q15 since we allow 2.0):
    if (q15 >  65536) q15 =  65536;
    if (q15 < -65536) q15 = -65536;
    return q15;
}

/*====================== Initialization =====================*/
void goertzel_init(float fs_hz, int N, float f1_hz, float f2_hz, uint32_t threshold)
{
    if (N < 8)   N = 8;
    if (N > MAX_N) N = MAX_N;
    gN       = N;
    gFs      = (fs_hz <= 0.0f) ? DEFAULT_FS : fs_hz;
    gCoeff1_q15 = goertzel_coeff(f1_hz);
    gCoeff2_q15 = goertzel_coeff(f2_hz);
    gThreshold  = threshold;

    gBufIdx   = 0;
    gBufFull  = 0;
    for (int i = 0; i < gN; i++) gBuf[i] = 0;
}

/*====================== Ring buffer I/O ====================*/
static inline void add_sample(int16_t sample)
{
    // Optional downscale to make room when N gets large
    if (INPUT_SHIFT > 0) sample >>= INPUT_SHIFT;

    gBuf[gBufIdx] = sample;
    gBufIdx++;
    if (gBufIdx >= gN) { gBufIdx = 0; gBufFull = 1; }
}

/*====================== Core Goertzel (block over N) =======*/
/*
  Integer-only Goertzel over the current window (N samples), starting
  from 'start' index in the ring buffer. coeff is Q15 in int32_t.
  Returns a scaled power estimate (uint32_t).
*/
static uint32_t goertzel_block(const int16_t *buf, int start, int N, int32_t coeff_q15)
{
    int32_t q0, q1 = 0, q2 = 0;

    // Recurrence over N samples (true sliding window)
    for (int i = 0; i < N; i++) {
        int16_t x = buf[(start + i) % N];
        // q0 = coeff*q1 - q2 + x
        // Use 64-bit for the multiply to avoid overflow, then >>15 for Q15.
        q0 = (int32_t)(((int64_t)coeff_q15 * (int64_t)q1) >> 15) - q2 + (int32_t)x;
        q2 = q1;
        q1 = q0;
    }

    // Power = q1^2 + q2^2 - coeff*q1*q2  (still valid for arbitrary ?)
    int64_t p = (int64_t)q1 * (int64_t)q1
              + (int64_t)q2 * (int64_t)q2
              - (((int64_t)coeff_q15 * (int64_t)q1 * (int64_t)q2) >> 15);

    // Scale down a bit to keep numbers in a comfy range
    if (p < 0) p = 0; // guard
    return (uint32_t)(p >> 10);
}

/*====================== Public API =========================*/
/*
  Feed one new sample. Returns:
    -1 while the buffer hasn't filled yet,
     0 or 1 once full (decision updates on every new sample).
*/
int goertzel_process_sample(int16_t sample,uint16_t* pP1,uint16_t* pP2)
{
    add_sample(sample);

    if (!gBufFull) return -1;

    // Start index is the current write pointer (oldest sample)
    int start = gBufIdx;

    // Evaluate both band edges (or two probes inside your 1–1.5 Hz band).
    uint32_t p1 = goertzel_block(gBuf, start, gN, gCoeff1_q15);
    uint32_t p2 = goertzel_block(gBuf, start, gN, gCoeff2_q15);

    uint32_t band_power = (p1 > p2) ? p1 : p2;

		*pP1 = p1>>2;
		*pP2 = p2>>2;
	
    // Decide
    return (band_power > gThreshold) ? 1 : 0;
}

#define GOERTZEL_FS 25	// sampling frequency in Hz
#define GOERTZEL_N 128		// buffer size

uint8_t bPilotInit = 1;
void PilotInit()
{
	goertzel_init(GOERTZEL_FS, GOERTZEL_N, 1.0, 1.5, 20000);
	bPilotInit = 0;
}

#define SAMPLETIME (1000/GOERTZEL_FS)	// every 40 ms = 25 Hz

uint32_t iTimeLastSample,iTimeNextSample = 0;
uint16_t iMpuRetries = 0;
void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)
{
	*pPwmMaster = -speed;
	uint32_t iNow = millis();
	if (iNow < iTimeNextSample)
		return;
	

	if (MPU_ReadAll() != SUCCESS) 
	{
		I2C_Init();	// try to re-init if i2c bus fails. Need when I2C_SPEED is 400000 instead of 200000
		//MPU_Init();		// re-init mpu6050 (not really needed)
		if (iMpuRetries < 100)
			iMpuRetries++;
		else
			bPilotTimeout = 1;
		
		return;
	}
	iMpuRetries = 0;

	iBug = iNow-iTimeLastSample;
	iTimeLastSample = iNow;
	iTimeNextSample = iNow + SAMPLETIME;
	
	if (bPilotInit)	PilotInit();
		
	int iResult = goertzel_process_sample(mpuData.gyro.x, &iGoert1,&iGoert2);
	//int iResult = goertzel_process_sample(mpuData.gyro.x - mpuData.gyro.y , &iGoert1,&iGoert2);
	//int iResult = goertzel_process_sample(mpuData.accel.z , &iGoert1,&iGoert2);
	if (iResult>=0)
	{
		currentDC = iResult == 0 ? 1 : 8;
	}

}
#else

#include <stdint.h>
#include <string.h>
#include <math.h>    // only for init (roundf). Not used at runtime.

/* =================== Configuration & Limits =================== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_FS_HZ        100     // just for sanity checks
#define MAX_LAGS         64      // max number of lag bins we track
#define MAX_BUF_POW2     256     // ring buffer size (power of two), adjust as needed

// Baseline (high-pass) time constant: baseline += (x - baseline) >> BASE_SHIFT
// With Fs=25 Hz, BASE_SHIFT=5 -> ~1.3 s time constant.
#define BASE_SHIFT       3


// Optional input downscale to grow headroom when buffers are long
#ifndef INPUT_SHIFT
#define INPUT_SHIFT 0
#endif

/* =================== State =================== */

typedef struct {
    // Runtime parameters
    uint32_t Fs_q16;           // sampling freq in Q16 (for optional uses)
    int      Lmin, Lmax;       // lag search range (samples) inclusive
    int      numLags;          // Lmax - Lmin + 1
    int      guard1, guard2;   // two guard lags outside band
    int      M;                // correlation window length (samples)
    int      Nbuf;             // ring buffer length (power of two)
    int      mask;             // Nbuf-1
    int      idx;              // current write index
    int      filled;           // number of valid samples written (caps at Nbuf)

    // Baseline for HP filter (Q0 integer)
    int32_t  baseline;

    // Clipped signal buffer (-1/0/+1)
    int8_t   s_buf[MAX_BUF_POW2];

    // Correlation accumulators for each lag in band
    int32_t  corr_band[MAX_LAGS];

    // Correlation accumulators for two guard lags
    int32_t  corr_guard1;
    int32_t  corr_guard2;

    // Last output state for Schmitt clip (holds state inside deadband)
    int8_t   clip_state;

    // Detection threshold as a fraction of M in Q15 (0..32767)
    // e.g. 0.55 -> 0.55*32768 ˜ 18022
    uint16_t thr_q15;
    // Guard margin: in-band correlation must exceed guards by this fraction
    // e.g. 0.15 -> 0.15*32768 ˜ 4915
    uint16_t guard_margin_q15;
} PedalDet;

static PedalDet PD;

/* =================== Utility =================== */

static int pow2_ceil(int x) {
    int p = 1;
    while (p < x && p < MAX_BUF_POW2) p <<= 1;
    if (p > MAX_BUF_POW2) p = MAX_BUF_POW2;
    return p;
}

static int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* =================== Init (floats OK here only) =================== */
/*
  fs_hz: sampling rate (e.g., 25.0f)
  fmin_hz, fmax_hz: band of interest (e.g., 1.0f..1.5f)
  window_periods: correlation window length ˜ window_periods * (1/fmin)
                  e.g. 2.0 -> ~2 cycles of the slowest cadence
  thr_frac: in-band corr threshold as fraction of M (e.g., 0.55)
  guard_margin_frac: required advantage over guards (e.g., 0.15)
*/
void pedal_init(float fs_hz,
                float fmin_hz,
                float fmax_hz,
                float window_periods,
                float thr_frac,
                float guard_margin_frac)
{
    if (fs_hz <= 0) fs_hz = 25.0f;
    if (fmin_hz <= 0) fmin_hz = 1.0f;
    if (fmax_hz < fmin_hz) { float t=fmin_hz; fmin_hz=fmax_hz; fmax_hz=t; }

    // Convert band to lag range in samples: L = round(Fs / f)
    int Lmin = (int)lroundf(fs_hz / fmax_hz);  // smaller lag = higher freq
    int Lmax = (int)lroundf(fs_hz / fmin_hz);  // larger lag = lower freq
    Lmin = clampi(Lmin, 2, 200);
    Lmax = clampi(Lmax, Lmin, 200);

    int numLags = Lmax - Lmin + 1;
    if (numLags > MAX_LAGS) {
        // shrink evenly around center if user picked a huge range
        int center = (Lmin + Lmax) / 2;
        int half   = MAX_LAGS / 2;
        Lmin = center - half;
        Lmax = center + (MAX_LAGS - 1 - half);
        numLags = MAX_LAGS;
    }

    // Choose correlation window M ˜ window_periods * (Fs/fmin)
    float M_f = window_periods * (fs_hz / fmin_hz);
    int   M   = (int)lroundf(M_f);
    M = clampi(M, Lmax + 4, MAX_BUF_POW2 - 4);  // at least one full slow period

    // Ring buffer must cover M + Lmax (we need s[n - Lmax] for the oldest i)
    int needed = M + Lmax + 2;
    int Nbuf   = pow2_ceil(needed);
    int mask   = Nbuf - 1;

    // Pick two guard lags outside the band (roughly 0.8 Hz and 1.7 Hz if available)
    int guard1 = (int)lroundf(fs_hz / 1.7f);
    int guard2 = (int)lroundf(fs_hz / 0.8f);
    // Ensure they are not inside the band; if they are, push further out
    if (guard1 >= Lmin && guard1 <= Lmax) guard1 = clampi(Lmin - 2, 2, 200);
    if (guard2 >= Lmin && guard2 <= Lmax) guard2 = clampi(Lmax + 2, 2, 200);

    // Thresholds in Q15
    if (thr_frac < 0.1f) thr_frac = 0.1f;
    if (thr_frac > 0.95f) thr_frac = 0.95f;
    if (guard_margin_frac < 0.0f) guard_margin_frac = 0.0f;
    if (guard_margin_frac > 0.9f) guard_margin_frac = 0.9f;

    memset(&PD, 0, sizeof(PD));
    PD.Fs_q16          = (uint32_t)lroundf(fs_hz * 65536.0f);
    PD.Lmin            = Lmin;
    PD.Lmax            = Lmax;
    PD.numLags         = numLags;
    PD.guard1          = guard1;
    PD.guard2          = guard2;
    PD.M               = M;
    PD.Nbuf            = Nbuf;
    PD.mask            = mask;
    PD.idx             = 0;
    PD.filled          = 0;
    PD.baseline        = 0;
    PD.clip_state      = 0;
    PD.thr_q15         = (uint16_t)lroundf(thr_frac * 32768.0f);
    PD.guard_margin_q15= (uint16_t)lroundf(guard_margin_frac * 32768.0f);

    memset(PD.s_buf, 0, sizeof(PD.s_buf));
    memset(PD.corr_band, 0, sizeof(PD.corr_band));
    PD.corr_guard1 = 0;
    PD.corr_guard2 = 0;
}

/* =================== Schmitt clip around baseline =================== */
static inline int8_t clip_sample(int16_t x)
{
	/*
    // Update baseline (EMA)
    PD.baseline += (((int32_t)x - PD.baseline) >> BASE_SHIFT);

    // High-pass residual (optionally downscale)
    int32_t d = ((int32_t)x - PD.baseline);
#if INPUT_SHIFT > 0
    d >>= INPUT_SHIFT;
#endif

		iGoert2 = d;
*/
		int32_t d = x;
	
    // Schmitt: hold previous state inside deadband
    if (d >= CLIP_THR_HI)       PD.clip_state = +1;
    else if (d <= CLIP_THR_LO)  PD.clip_state = -1;
    // else retain previous PD.clip_state

    return PD.clip_state;
}

/* =========== Update one correlation accumulator (one lag) =========== */
static inline void corr_update_one(int lag, int8_t s_new, int8_t s_old_out)
{
    // Indices for the "other side" of the product
    int idx_new_del   = (PD.idx - 1 - lag) & PD.mask;  // index matching s_new
    int idx_old_del   = (PD.idx - 1 - PD.M - lag) & PD.mask; // leaving partner

    int8_t s_new_del  = PD.s_buf[idx_new_del];
    int8_t s_old_del  = PD.s_buf[idx_old_del];

    // Product of ±1/0 fits in int8, accumulate into int32
    int32_t add = (int32_t)s_new * (int32_t)s_new_del;
    int32_t sub = (int32_t)s_old_out * (int32_t)s_old_del;

    // Map lag to band index if inside band, else to guards
    if (lag >= PD.Lmin && lag <= PD.Lmax) {
        int bandIdx = lag - PD.Lmin;
        PD.corr_band[bandIdx] += add - sub;
    } else if (lag == PD.guard1) {
        PD.corr_guard1 += add - sub;
    } else if (lag == PD.guard2) {
        PD.corr_guard2 += add - sub;
    }
}

/* =================== Public: process one sample =================== */
/*
  Returns:
    -1 : not enough data yet
     0 : no pedaling detected
     1 : pedaling (1..1.5 Hz) detected
*/
int32_t gmax;
int32_t best_band;
int pedal_process_sample(int16_t x_raw)
{
    // 1) Clip to -1/0/+1 around a running baseline (amplitude-robust)
    int8_t s_new = clip_sample(x_raw);

    // 2) Push into ring buffer; grab the sample that leaves the M-window
    int old_idx = (PD.idx - PD.M) & PD.mask;
    int8_t s_old_out = PD.s_buf[old_idx];

    PD.s_buf[PD.idx] = s_new;
    PD.idx = (PD.idx + 1) & PD.mask;
    if (PD.filled < PD.Nbuf) PD.filled++;

    if (PD.filled < (PD.M + PD.Lmax + 1)) {
        // Not enough history to form full correlations yet
        return -1;
    }

    // 3) Update correlations for each in-band lag
    int lag = PD.Lmin;
    for (int i = 0; i < PD.numLags; i++, lag++) {
        corr_update_one(lag, s_new, s_old_out);
    }
    // Update the two guards
    corr_update_one(PD.guard1, s_new, s_old_out);
    corr_update_one(PD.guard2, s_new, s_old_out);

    // 4) Decision (per-sample)
    // Normalize by M by comparing against thresholds scaled with M
    int32_t best_band1 = 0;
    for (int i = 0; i < PD.numLags; i++) {
        int32_t v = PD.corr_band[i];
        if (v > best_band1) best_band1 = v;
    }
		best_band = best_band1;
    // Convert thresholds: (thr_q15/32768)*M  => (thr_q15 * M) >> 15
    int32_t thr_abs        = ((int32_t)PD.thr_q15 * (int32_t)PD.M) >> 15;
    int32_t guard_margin   = ((int32_t)PD.guard_margin_q15 * (int32_t)PD.M) >> 15;

    // Guard criterion: best_band should clearly beat guards
    int32_t g1 = PD.corr_guard1;
    int32_t g2 = PD.corr_guard2;
    gmax = (g1 > g2) ? g1 : g2;

    int detected = (best_band >= thr_abs) && (best_band >= gmax + guard_margin);
    return detected ? 1 : 0;
}


#define SAMPLING_FREQ 25	// sampling frequency in Hz
uint16_t iBrake = 0;
// CLIP_THR_HI
uint8_t bPilotInit = 1;
IIR_HPF hpf;

void PilotInit()
{
		//=================== Example suggested init ===================
   // For Fs=25 Hz, band 1..1.5 Hz, ~2 cycles window, 55% threshold, 15% guard margin:
		// pedal_init(fs_hz, fmin_hz, fmax_hz, window_periods, thr_frac, guard_margin_frac)
   // pedal_init(25.0f, 1.0f, 1.5f, 2.0f, 0.55f, 0.15f);
	//pedal_init(SAMPLING_FREQ, 1.0f, 1.5f, 2.0f, 0.55f, 0.15f);
	pedal_init(SAMPLING_FREQ, 1.0f, 1.5f, 2.0f, 0.25f, 0.05f);
	bPilotInit = 0;
	
	init_iir_hpf(&hpf);
}

#define SAMPLETIME (1000/SAMPLING_FREQ)	// every 40 ms = 25 Hz

int8_t iClipState=0;
uint32_t iTimeLastSample,iTimeNextSample = 0;
uint16_t iMpuRetries = 0;

int16_t iSample;
int16_t iSampleHP;


void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)
{
	*pPwmMaster = -speed;
	
	uint32_t iNow = millis();
	if (iNow < iTimeNextSample)
		return;
	
	if (MPU_ReadAll() != SUCCESS) 
	{
		I2C_Init();	// try to re-init if i2c bus fails. Need when I2C_SPEED is 400000 instead of 200000
		//MPU_Init();		// re-init mpu6050 (not really needed)
		if (iMpuRetries < 100)
			iMpuRetries++;
		else
			bPilotTimeout = 1;
		
		return;
	}
	iMpuRetries = 0;

	iBug = iNow-iTimeLastSample;
	iTimeLastSample = iNow;
	iTimeNextSample = iNow + SAMPLETIME;
	
	if (mpuData.gyro.z < -200)	iBrake = 50;	// this needs a high-pass
	else if (iBrake>0) iBrake--;
	
	if (bPilotInit)	PilotInit();
	
	//iSample = mpuData.gyro.x - mpuData.gyro.y;	
	iSample = mpuData.gyro.z;		// it is the bumping because of a fully supension bike
  iSampleHP = iir_hpf_update(&hpf, iSample);		// high pass filter for 25 Hz and cutoff below 1 Hz
	
	int iResult = pedal_process_sample(iSample);
	
	iGoert3 = iClipState = PD.clip_state;
	iGoert1 = iSample;
	
	//int iResult = goertzel_process_sample(mpuData.gyro.x - mpuData.gyro.y , &iGoert1,&iGoert2);
	//int iResult = goertzel_process_sample(mpuData.accel.z , &iGoert1,&iGoert2);
	if (iResult>=0)
	{
		currentDC = (iResult == 0 ? 1 : 8) + iBrake;
	}

}


#endif

#endif	// #ifdef PILOT_HOVERBIKE