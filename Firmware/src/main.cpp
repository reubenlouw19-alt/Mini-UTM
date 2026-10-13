// Miniature tension / compression rig
// Mechanical Project 478 - Reuben Louw 26896850 - Stellenbosch University
//
// Hardware: ESP32, Adafruit TMC2209 breakout + 57BYGH420 stepper, SFU1204
// ball screw (4 mm lead), linear pot (Opkon SLPT-75) on an ADS1115, load
// cell (PA6110) on an HX711.
//
// How the program works. loop() calls these over and over:
// readSerial() - commands from the dashboard
// serviceSafety() - E-stop and end stops
// serviceJog() - notices when a jog has finished
// servicePot() - linear pot -> crosshead extension
// serviceClosedLoop() - closed-loop test: sets the motor speed
// serviceCell() - load cell -> force
// printTelemetry() - one "T ..." line every 100 ms for the dashboard
// The motor is stepped by a timer interrupt (onStepTimer), so it runs
// smoothly no matter what loop() is busy with.
//
// Directions are absolute: TENSION = crosshead UP (opening),
// COMPRESSION = crosshead DOWN (closing)
//
// Gains, filters and calibrations are constants below: change one and
// reflash. Nothing is stored in flash. Type HELP for the command list.

#include <Arduino.h>
#include <TMCStepper.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <HX711.h>
#include <soc/gpio_struct.h> // GPIO registers, used in step interrupt
#include <esp_rom_sys.h> // esp_rom_delay_us()
#include <esp_timer.h> // esp_timer_get_time(): microseconds, 64-bit, never wraps
#include <math.h>
#include <algorithm> // std::sort

// Pins

#define STEP_PIN 32
#define DIR_PIN 33
#define EN_PIN 25
#define TMC_TX 16
#define TMC_RX 17
#define ESTOP_PIN 34
#define ENDSTOP_TOP_PIN 36
#define ENDSTOP_BTM_PIN 39
#define HX711_DOUT 35
#define HX711_SCK 26
#define I2C_SDA 21
#define I2C_SCL 22

#define TMC_SERIAL Serial2

// Mechanics and direction

constexpr float LEAD_MM = 4.0f; // SFU1204
constexpr uint16_t FULL_STEPS = 200; // 1.8 deg motor
constexpr uint16_t MICROSTEPS = 256;
constexpr float STEPS_PER_MM = FULL_STEPS * MICROSTEPS / LEAD_MM; // 12800
constexpr int32_t STEPS_PER_REV = (int32_t)FULL_STEPS * MICROSTEPS; // 51200
constexpr float MAX_MM_MIN = 40.0f; // user defined limit

constexpr uint8_t DIR_UP_LEVEL = LOW;
constexpr float FORCE_SIGN = -1.0f;

constexpr uint32_t STEP_TICK_US = 50;
constexpr double STEP_TICK_HZ = 1e6 / STEP_TICK_US;

// Tuning: closed-loop displacement control

constexpr float CL_LAMBDA_MM = 0.050f; // loop length scale [mm]
constexpr float CL_PERIOD_S = 0.020f; // control update interval [s]
constexpr float MACHINE_K_NMM = 1107.0f; // machine stiffness [N/mm]

// Tuning: TMC2209

constexpr float R_SENSE = 0.05f;
constexpr uint8_t DRV_ADDR = 0b00;
constexpr uint16_t RMS_CURRENT_MA = 1300;
constexpr bool SPREADCYCLE = false; // StealthChop on boot

// Tuning: sensors

constexpr uint8_t POT_CH_WIPER = 0;
constexpr uint8_t POT_CH_EXC = 1;
constexpr uint8_t POT_WIN_N = 16; // pot filter window, samples (~0.18 s)
constexpr uint32_t POT_CONV_US = 4000; // wait per conversion, after the start command returns
constexpr uint16_t POT_NOISE_N = 400; // samples for POT NOISE (~4.5 s)

constexpr float GRAVITY = 9.807f; // [m/s^2]
constexpr uint8_t CELL_AVG = 3; // HX711 runs at 10 SPS
constexpr uint8_t CELL_TARE_N = 20; // readings averaged for a cell tare

constexpr uint32_t TELE_MS = 100; // telemetry interval [ms]

// Load cell calibration:
// N = CELL_ZERO_COUNTS - CELL_COUNTS_PER_G * m (m in grams)
// so m = (CELL_ZERO_COUNTS - N) / CELL_COUNTS_PER_G
constexpr long CELL_ZERO_COUNTS = 830219L; // fitted intercept, counts
constexpr double CELL_COUNTS_PER_G = 132.304; // fitted slope magnitude [counts/g]

// Linear pot calibration - pooled least-squares fit of three runs
// x = x0 + POT_MM_PER_RATIO * p (x in mm, p = wiper / excitation)
constexpr float POT_MM_PER_RATIO = 75.383f; // fitted slope, mm per unit ratio
constexpr float POT_ZERO_MM = 0.0f; // absolute offset [mm]

// Global variables

TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRV_ADDR);
Adafruit_ADS1115 ads;
HX711 scale;
hw_timer_t *stepTimer = nullptr;

// --- motion ---
volatile bool moving = false; // move, jog or test is running (interrupt only steps while true)
volatile bool dirUp = true; // current direction
volatile uint32_t stepInc = 0; // added to stepAcc every tick: sets the speed
volatile uint32_t stepAcc = 0; // step counter: step made each time it overflows
volatile int32_t stepsLeft = 0; // jog: microsteps still to go (0 = run until STOP)
volatile bool jogDone = false; // set by the interrupt when a jog finishes
volatile int32_t stepPos = 0; // motor position, microsteps (+ = up)
volatile bool estopLatched = false;

bool testing = false; // the move is a TEST (tension if up, compression if down)
float speedMmMin = 0.0f; // set speed (test / move / jog)
float motorMmMin = 0.0f; // speed the motor is actually running at
int32_t stepPosDatum = 0; // motor position at the last TARE

// --- closed loop ---
// clEnabled is the one loop setting serial can change: the dashboard sends
// LOOP ON / LOOP OFF from its checkbox before every test.
// The set speed, direction and gains are not copied here: they are
// speedMmMin, dirUp and loopKp() / loopKi().
bool clEnabled = false; // LOOP ON: TEST commands run closed loop
bool clActive = false; // a closed-loop test is running now
float clX0 = 0.0f; // extension when the test started, mm
int32_t clMot0 = 0; // motor position when the test started, microsteps
float clF0 = 0.0f; // force when the test started, N
float clRef0 = 0.0f; // where the reference ramp starts, mm
int64_t clRefStartUs = 0; // when the reference ramp started
float clRef = 0.0f; // reference now, mm
float clErr = 0.0f; // reference - measured in the test direction, mm (+ = behind)
float clInt = 0.0f; // integral of the error, mm s
float clLost = 0.0f; // lost motion since the start, mm
int64_t clLastUs = 0; // time of the last control update

// --- driver ---
bool spreadCycle = SPREADCYCLE; // MODE SPREAD / MODE STEALTH
uint8_t drvTemp = 0; // 0 OK, 1 overtemp warning, 2 overtemp shutdown (reported only)

// --- end stops ---
bool endTop = false, endBtm = false; // true = pressed

// --- linear pot ---
enum PotPhase { POT_IDLE, POT_WIPER, POT_EXC }; // which conversion is running

bool potPresent = false; // ADS1115 found at boot
PotPhase potPhase = POT_IDLE;
uint32_t potStartUs = 0; // when the running conversion was started
float potW = 0.0f; // wiper voltage of the sample being read
float potVExc = 0.0f; // last excitation voltage

float potWin[POT_WIN_N]; // filter window: the last potWinFill ratios
uint8_t potWinHead = 0, potWinFill = 0;
float potRatio = 0.0f; // filtered ratio (wiper / excitation)

bool potDatumSet = false; // TARE has been done
float potDatumMm = 0.0f; // extension = position - datum (0 until TARE)

float potBlock[POT_NOISE_N]; // POT NOISE: raw samples
float potBlockTmp[POT_NOISE_N]; // POT NOISE: filter outputs

// --- load cell ---
bool cellPresent = false; // HX711 found at boot
long cellZero = CELL_ZERO_COUNTS; // zero in use (TARE changes it, RAM only)
long cellBuf[CELL_AVG]; // last CELL_AVG raw readings, averaged
uint8_t cellBufN = 0, cellBufHead = 0;
float cellForceN = 0.0f;

// --- telemetry and serial ---
uint32_t lastTele = 0, lastDrvPoll = 0;
String rxLine;

// Step generation (timer interrupt)
//
// The timer calls onStepTimer() every 50 us. Each call adds stepInc to the
// 32-bit counter stepAcc, and every time the counter overflows one step is
// made. So the step rate is stepInc / 2^32 * 20 kHz, and the speed can be
// changed at any moment just by changing stepInc - no timer restart, no
// dropped steps.
//
// The interrupt reads and writes the GPIO registers directly: it is much
// faster than digitalRead / digitalWrite. GPIO32-39 are in the second
// register bank (in1 / out1), bit = pin - 32.

static_assert(STEP_PIN >= 32 && ENDSTOP_TOP_PIN >= 32 && ENDSTOP_BTM_PIN >= 32,
        "the step interrupt assumes STEP and the end stops are on GPIO32-39");
constexpr uint32_t STEP_BIT = 1UL << (STEP_PIN - 32);
constexpr uint32_t TOP_BIT = 1UL << (ENDSTOP_TOP_PIN - 32);
constexpr uint32_t BTM_BIT = 1UL << (ENDSTOP_BTM_PIN - 32);

void IRAM_ATTR onStepTimer()
{
    if (!moving || estopLatched) return;

    uint32_t before = stepAcc;
    uint32_t after = before + stepInc; // wraps round past 2^32
    stepAcc = after;
    if (after >= before) return; // no overflow: no step this tick

    // End stops (LOW = pressed) are also checked here, so a switch stops the
    // very next step. Only the end you are moving TOWARDS blocks; you can
    // always drive away from a pressed switch.
    uint32_t pins = GPIO.in1.val;
    if (dirUp && !(pins & TOP_BIT)) return;
    if (!dirUp && !(pins & BTM_BIT)) return;

    GPIO.out1_w1ts.val = STEP_BIT; // STEP high
    esp_rom_delay_us(4); // TMC2209 needs > 100 ns
    GPIO.out1_w1tc.val = STEP_BIT; // STEP low

    stepPos += dirUp ? 1 : -1;

    if (stepsLeft > 0) // jogging
    {
        stepsLeft--;
        if (stepsLeft == 0)
        {
            moving = false;
            jogDone = true;
        }
    }
}

// Start the 20 kHz tick. Called once from setup(); the timer never stops.
void stepTimerStart()
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    stepTimer = timerBegin(1000000); // 1 us ticks
    timerAttachInterrupt(stepTimer, &onStepTimer);
    timerAlarm(stepTimer, STEP_TICK_US, true, 0);
#else
    stepTimer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 us ticks
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, STEP_TICK_US, true);
    timerAlarmEnable(stepTimer);
#endif
}

// Set the motor speed, kept in 0..MAX_MM_MIN. Safe to call while moving.
void setMotorSpeed(float mmMin)
{
    if (!(mmMin > 0.0f)) mmMin = 0.0f; // written this way so NaN also gives 0
    if (mmMin > MAX_MM_MIN) mmMin = MAX_MM_MIN;
    double stepsPerS = (double)mmMin / 60.0 * STEPS_PER_MM;
    stepInc = (uint32_t)(stepsPerS / STEP_TICK_HZ * 4294967296.0); // 2^32
    motorMmMin = mmMin;
}

// Motion

const char *moveName()
{
    if (!moving) return "IDLE";
    return dirUp ? "UP" : "DOWN";
}

const char *testName()
{
    if (!testing) return "NONE";
    return dirUp ? "TENSION" : "COMPRESSION"; // directions are absolute
}

// Stop the motor and end any move, jog or test. The driver keeps holding.
void stopMotion(const char *why)
{
    bool wasMoving = moving;
    moving = false; // the step interrupt stops here
    stepsLeft = 0;
    setMotorSpeed(0.0f);
    if (wasMoving) Serial.printf("STOPPED: %s\n", why);
    testing = false;
    speedMmMin = 0.0f;
    clActive = false;
}

// Start moving. steps = 0 means run until STOP.
void startMotion(bool up, float mmMin, int32_t steps, bool test)
{
    moving = false; // hold the interrupt while things change
    clActive = false;

    dirUp = up;
    digitalWrite(DIR_PIN, up ? DIR_UP_LEVEL : !DIR_UP_LEVEL);
    delayMicroseconds(50); // let the driver see the new direction

    speedMmMin = mmMin;
    testing = test;
    stepsLeft = steps;
    jogDone = false;
    stepAcc = 0;
    setMotorSpeed(mmMin);
    moving = true;
}

// The step interrupt sets jogDone (and clears moving) when a jog has made
// all its steps. stopMotion() prints nothing here, as moving is already false.
void serviceJog()
{
    if (!jogDone) return;
    jogDone = false;
    stopMotion("jog done");
    Serial.println("JOG DONE");
}

// E-stop and end stops
//
// The E-stop button clamps STEP in hardware. The firmware latch makes the
// stop stick: while it is latched nothing moves, until RESUME clears it (if
// the button is still pressed it latches again straight away).
// An end stop only stops motion TOWARDS it; you can always drive away from a
// pressed switch.

bool estopPressed() { return digitalRead(ESTOP_PIN) == HIGH; }

void serviceSafety()
{
    if (estopPressed() && !estopLatched)
    {
        estopLatched = true; // the step interrupt stops immediately
        Serial.println("E-STOP LATCHED. Driver still holding. Release the button, then send RESUME.");
    }
    if (estopLatched && moving) stopMotion("E-STOP");

    endTop = digitalRead(ENDSTOP_TOP_PIN) == LOW; // LOW = pressed
    endBtm = digitalRead(ENDSTOP_BTM_PIN) == LOW;
    if (moving && (dirUp ? endTop : endBtm)) stopMotion(dirUp ? "TOP end stop" : "BOTTOM end stop");
}

// TMC2209 driver

void driverConfigure()
{
    driver.begin();
    driver.pdn_disable(true); // UART controls the driver
    driver.mstep_reg_select(true); // microsteps from UART, not MS1/MS2
    driver.I_scale_analog(false); // current from UART, not the VREF pot
    driver.toff(4);
    driver.blank_time(24);
    driver.rms_current(RMS_CURRENT_MA, 1.0); // hold = run: the e-stop relies on holding
    driver.microsteps(MICROSTEPS);
    driver.dedge(false);
    driver.intpol(false);
    driver.en_spreadCycle(spreadCycle);
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.TPOWERDOWN(255);
    driver.iholddelay(0);
    delay(50);

    Serial.printf("TMC2209: UART %s, microsteps %u, %u mA RMS (driver reports %u), %s\n",
            driver.test_connection() == 0 ? "OK" : "FAILED", driver.microsteps(),
            RMS_CURRENT_MA, driver.rms_current(), spreadCycle ? "SpreadCycle" : "StealthChop");
}

// Driver temperature flags, for the ot= telemetry field only
void pollDriverTemp()
{
    uint32_t status = driver.DRV_STATUS();
    drvTemp = (status & 0x02) ? 2 : (status & 0x01) ? 1 : 0; // ot, otpw
}

// DRV: driver diagnostics (dashboard button)
void printDrv()
{
    uint32_t s = driver.DRV_STATUS();
    uint8_t g = driver.GSTAT();
    Serial.println("---- TMC2209 ----");
    Serial.printf("Current      %u mA set, %u mA reported, CS_ACTUAL %u/31\n",
            RMS_CURRENT_MA, driver.rms_current(), driver.cs_actual());
    Serial.printf("Mode         %s\n", driver.stealth() ? "StealthChop" : "SpreadCycle");
    Serial.printf("Overtemp     warn(>120C) %s   shutdown %s   >143C %s\n",
            driver.otpw() ? "YES" : "no", driver.ot() ? "YES" : "no", driver.t143() ? "YES" : "no");
    Serial.printf("Shorts       GND A/B %s/%s   VS A/B %s/%s\n",
            driver.s2ga() ? "YES" : "no", driver.s2gb() ? "YES" : "no",
            driver.s2vsa() ? "YES" : "no", driver.s2vsb() ? "YES" : "no");
    Serial.printf("Open load    A/B %s/%s (only valid while moving)\n",
            driver.ola() ? "YES" : "no", driver.olb() ? "YES" : "no");
    Serial.printf("GSTAT        0x%02X (reset/drv_err/uv_cp)   DRV_STATUS 0x%08lX\n",
            g, (unsigned long)s);
    driver.GSTAT(0x07); // clear latched flags
}

// Linear pot - ADS1115
//
// One sample = two single-shot conversions, wiper then excitation. Using the
// ratio wiper / excitation cancels drift in the 3V3 supply.
//
// Each conversion is simply given POT_CONV_US to finish and then read; the
// ALERT/RDY pin and the conversion-ready bit are not used. A 250 SPS
// conversion takes 4 ms (up to 4.4 ms on the ADS1115's +/-10 % clock), and
// the library's start command spends another ~0.7 ms writing the threshold
// registers after the conversion has begun, so the result is there 4 ms
// after it returns. One sample takes ~11 ms (~90 samples/s).
//
// Filter: the last POT_WIN_N ratios go through a trimmed mean, which ignores
// single-sample spikes without adding much lag.
//
// Note: the ~45 um "stair steps" seen 11-16 Sep were a counterfeit 12-bit
// ADC sold as an ADS1115 (every code a multiple of 16). Replaced 17 Sep.

// Sample standard deviation (divides by n - 1)
float standardDeviation(const float *values, uint16_t n)
{
    double sum = 0;
    for (uint16_t i = 0; i < n; i++) sum += values[i];
    double mean = sum / n;
    double sumSq = 0;
    for (uint16_t i = 0; i < n; i++) sumSq += (values[i] - mean) * (values[i] - mean);
    return (float)sqrt(sumSq / (n - 1));
}

// Mean of the middle half: sort a copy, drop the lowest and highest quarter,
// average the rest. Ignores single-sample spikes.
float trimmedMean(const float *values, uint8_t n)
{
    float sorted[POT_WIN_N];
    memcpy(sorted, values, n * sizeof(float));
    std::sort(sorted, sorted + n);
    uint8_t cut = n / 4;
    float sum = 0;
    for (uint8_t i = cut; i < n - cut; i++) sum += sorted[i];
    return sum / (n - 2 * cut);
}

float potAbsMm() { return POT_ZERO_MM + POT_MM_PER_RATIO * potRatio; }

float potExtMm() { return potAbsMm() - potDatumMm; }

void potBegin()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(10);
    potPresent = ads.begin(0x48, &Wire);
    ads.setGain(GAIN_ONE); // +/-4.096 V, 125 uV per count
    ads.setDataRate(RATE_ADS1115_250SPS);
}

// Start a single-shot conversion on one channel
void potStartConversion(uint8_t ch)
{
    ads.startADCReading(MUX_BY_CHANNEL[ch], false);
    potStartUs = micros();
}

// Result of the last conversion, volts
float potLastVolts() { return ads.computeVolts(ads.getLastConversionResults()); }

// Add a new ratio to the filter window and update potRatio
void potAddSample(float ratio)
{
    potWin[potWinHead] = ratio;
    potWinHead = (potWinHead + 1) % POT_WIN_N;
    if (potWinFill < POT_WIN_N) potWinFill++;
    potRatio = trimmedMean(potWin, potWinFill);
}

// Live reading, called from loop(). Never waits: it starts a conversion,
// and on later calls reads it once POT_CONV_US has passed.
void servicePot()
{
    if (!potPresent) return;
    if (potPhase != POT_IDLE && micros() - potStartUs < POT_CONV_US) return; // still converting

    if (potPhase == POT_IDLE) // start the wiper conversion
    {
        potStartConversion(POT_CH_WIPER);
        potPhase = POT_WIPER;
    }
    else if (potPhase == POT_WIPER) // wiper done: keep it, start the excitation conversion
    {
        potW = potLastVolts();
        potStartConversion(POT_CH_EXC);
        potPhase = POT_EXC;
    }
    else // excitation done: a full sample
    {
        potVExc = potLastVolts();
        potPhase = POT_IDLE;
        potAddSample(potW / potVExc);
    }
}

// One sample, waiting for both conversions (POT NOISE only)
float potReadBlocking()
{
    potStartConversion(POT_CH_WIPER);
    delayMicroseconds(POT_CONV_US);
    float w = potLastVolts();
    potStartConversion(POT_CH_EXC);
    delayMicroseconds(POT_CONV_US);
    return w / potLastVolts();
}

// POT NOISE: measure the pot, don't change it. Reports the noise of single
// samples and of the live filter. The filter figure is measured, not derived:
// the filter (trimmed mean of POT_WIN_N) is slid along the block one sample at
// a time and the spread of its output is reported.
void potNoise()
{
    potReadBlocking(); // the live reading may still be converting: discard one
    for (uint16_t i = 0; i < POT_NOISE_N; i++) potBlock[i] = potReadBlocking();
    potPhase = POT_IDLE; // the live reading starts again from scratch

    uint16_t windows = 0;
    for (uint16_t i = 0; i + POT_WIN_N <= POT_NOISE_N; i++)
        potBlockTmp[windows++] = trimmedMean(&potBlock[i], POT_WIN_N);

    float umPerRatio = POT_MM_PER_RATIO * 1000.0f;
    Serial.printf("POT raw single-sample noise: %.2f um RMS (%u samples). "
            "Filtered (window %u, trimmed) %.2f um RMS, measured over %u windows.\n",
            standardDeviation(potBlock, POT_NOISE_N) * umPerRatio, POT_NOISE_N,
            POT_WIN_N, standardDeviation(potBlockTmp, windows) * umPerRatio, windows);
}

// Zero the extension (and the motor position) at the current position
void potTare()
{
    potDatumMm = potAbsMm();
    potDatumSet = true;
    stepPosDatum = stepPos;
    Serial.printf("POT datum set at %.4f mm\n", potDatumMm);
}

// Load cell - HX711 (10 readings per second)

// Live reading, called from loop(). Only does something when the HX711 has a new reading.
void serviceCell()
{
    if (!cellPresent || !scale.is_ready()) return;

    // Force = average of the last CELL_AVG readings, through the calibration
    cellBuf[cellBufHead] = scale.read();
    cellBufHead = (cellBufHead + 1) % CELL_AVG;
    if (cellBufN < CELL_AVG) cellBufN++;
    double sum = 0;
    for (uint8_t i = 0; i < cellBufN; i++) sum += cellBuf[i];
    double grams = ((double)cellZero - sum / cellBufN) / CELL_COUNTS_PER_G;
    cellForceN = (float)(FORCE_SIGN * grams * GRAVITY / 1000.0);
}

// Zero the force: average of CELL_TARE_N raw readings (~2 s)
void cellTare()
{
    if (!cellPresent) return;
    cellZero = scale.read_average(CELL_TARE_N);
    cellBufN = 0; // restart the running average
    cellBufHead = 0;
    Serial.printf("CELL zeroed (%ld counts, %+ld from calibration zero).\n",
            cellZero, cellZero - CELL_ZERO_COUNTS);
}

// Closed-loop displacement control
//
// Setpoint: ref(t) = ext at start + sign * v * t (a ramp at the set speed)
// Feedback: pot extension (trimmed-mean filtered)
// Output: motor speed u = v + 60 (Kp e + Ki integral(e dt))
//
// The loop runs until it is told to stop: STOP (the dashboard sends it on
// fracture and from its Stop button), an end stop or the E-stop.

// +1 in tension (UP = extension increasing), -1 in compression
float testSign() { return dirUp ? 1.0f : -1.0f; }

// Gains scale with the set speed v: Kp = (v / 60) / lambda, Ki = Kp^2 / 4
float loopKp() { return (speedMmMin / 60.0f) / CL_LAMBDA_MM; }
float loopKi() { return loopKp() * loopKp() / 4.0f; }

// Reference ramp at time nowUs, mm
float refAt(int64_t nowUs)
{
    float t = (nowUs - clRefStartUs) * 1e-6; // seconds since the ramp started
    return clRef0 + testSign() * speedMmMin / 60.0f * t;
}

// Close the loop on the TEST that startMotion() has just started
void startClosedLoop()
{
    clX0 = potExtMm();
    clMot0 = stepPos;
    clF0 = cellForceN;
    clRef0 = clX0; // the ramp starts from here, now
    clRef = clX0;
    clErr = 0.0f;
    clInt = 0.0f;
    clLost = 0.0f;
    clRefStartUs = esp_timer_get_time();
    clLastUs = clRefStartUs;
    clActive = true;
}

// SPEED <v>: change the speed of a running test (the post-yield speed-up).
// Open loop: just the motor speed. Closed loop: the reference ramp carries on
// from where it is now at the new slope, the gains are reset for the new
// speed, and the integral is rescaled so its output does not jump.
// SPEED 0 pauses the test in both modes: the motor stops (in closed loop the
// ramp holds and the gains are 0) and a later SPEED <v> resumes it. On
// resuming, the closed-loop integral starts again from 0.
void changeTestSpeed(float v)
{
    if (!(v > 0.0f)) v = 0.0f; // negative or NaN also pauses, as in setMotorSpeed()
    Serial.printf("SPEED: test speed %.4f -> %.4f mm/min\n", speedMmMin, v);
    if (!clActive) { speedMmMin = v; setMotorSpeed(v); return; }

    int64_t now = esp_timer_get_time();
    clRef0 = refAt(now); // the end of the old ramp is the start of the new one
    clRefStartUs = now;

    float kiOld = loopKi();
    speedMmMin = v; // new slope and new gains
    if (v > 0.0f) clInt *= kiOld / loopKi(); // keep the integral term's output the same (0 after a pause)
}

// Lost motion = motor travel - crosshead travel - machine stretch, all in the
// test direction. Telemetry only (`lost`): a stall, skipped steps or coupling
// slip show up as it growing.
void updateLostMotion(float x, float force)
{
    float motorTravel = (stepPos - clMot0) / STEPS_PER_MM;
    float crossheadTravel = x - clX0;
    float stretch = (force - clF0) / MACHINE_K_NMM;
    clLost = testSign() * (motorTravel - crossheadTravel - stretch);
}

// Called from loop(). One PI update every CL_PERIOD_S.
void serviceClosedLoop()
{
    if (!clActive) return;

    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - clLastUs) * 1e-6f;
    if (dt < CL_PERIOD_S) return;
    clLastUs = nowUs;

    float x = potExtMm();
    clRef = refAt(nowUs);
    clErr = testSign() * (clRef - x); // + = crosshead behind the reference
    clInt += clErr * dt;

    setMotorSpeed(speedMmMin + 60.0f * (loopKp() * clErr + loopKi() * clInt)); // PI

    updateLostMotion(x, cellForceN);
}

// Telemetry: one line of key=value pairs, read by the dashboard
//
// pos = pot position, mm
// ext = extension since TARE, mm
// f = force, N (tension +)
// v = set speed, mm/min
// cmd = reference ramp in closed loop, motor position since TARE otherwise
// mot = motor position since TARE, mm (always from the step count)
// u = speed the motor is running at, mm/min
// err = closed-loop error in um (+ = behind the reference), 0 in open loop
// ang = screw angle 0-360 deg from the step count (for once-per-rev analysis)
// lost = lost drive motion since the start of the test, mm
// es, top, btm = 1 when the E-stop is latched / an end stop is pressed
// pot, cell = 1 when the ADS1115 / HX711 was found at boot
// ot = driver temperature: 0 OK, 1 warning, 2 shutdown (reported only)

void printTelemetry()
{
    int32_t pos = stepPos;
    float motMm = (pos - stepPosDatum) / STEPS_PER_MM;
    int32_t revPos = pos % STEPS_PER_REV; // microsteps into the current screw turn
    if (revPos < 0) revPos += STEPS_PER_REV;
    float angDeg = revPos * 360.0f / STEPS_PER_REV;

    // the flags are bools, which printf %d prints as 0 / 1
    Serial.printf("T t=%lu pos=%.4f ext=%.4f f=%.2f cmd=%.4f mv=%s test=%s v=%.4f "
            "es=%d top=%d btm=%d pot=%d cell=%d ot=%d "
            "cl=%d ref=%.4f err=%.1f u=%.4f mot=%.4f ang=%.1f lost=%.4f\n",
            (unsigned long)millis(),
            potAbsMm(), potExtMm(), cellForceN,
            clActive ? clRef : motMm,
            moveName(), testName(), speedMmMin,
            estopLatched, endTop, endBtm, potPresent, cellPresent, drvTemp,
            clActive,
            clActive ? clRef : 0.0f,
            clActive ? clErr * 1000.0f : 0.0f,
            motorMmMin, motMm, angDeg,
            clActive ? clLost : 0.0f);
}

// Status printouts (dashboard buttons)

void printStatus()
{
    const char *overtemp = drvTemp == 2 ? "SHUTDOWN" : drvTemp == 1 ? "WARNING" : "no";

    Serial.println("---- STATUS ----");
    Serial.printf("Motion       %s%s at %.4f mm/min (motor %.4f), test %s\n", moveName(),
            stepsLeft > 0 ? " (jog)" : "", speedMmMin, motorMmMin, testName());
    Serial.printf("Commanded    %.4f mm since tare (%ld microsteps)\n",
            (stepPos - stepPosDatum) / STEPS_PER_MM, (long)(stepPos - stepPosDatum));
    Serial.printf("Closed loop  %s%s\n", clEnabled ? "ON" : "OFF", clActive ? " (running)" : "");
    Serial.printf("E-stop       %s (button %s)\n", estopLatched ? "LATCHED" : "clear",
            estopPressed() ? "PRESSED" : "released");
    Serial.printf("End stops    TOP %s   BOTTOM %s\n", endTop ? "HIT" : "clear", endBtm ? "HIT" : "clear");
    Serial.printf("Driver       %u mA, %s, overtemp %s\n", RMS_CURRENT_MA,
            spreadCycle ? "SpreadCycle" : "StealthChop", overtemp);
    Serial.printf("Pot          %s, abs %.4f mm, ext %.4f mm, exc %.3f V (%s)\n",
            potPresent ? "OK" : "NO ADS1115", potAbsMm(), potExtMm(), potVExc,
            potDatumSet ? "tared" : "no datum");
    Serial.printf("Load cell    %s, %.2f N (zero %+ld counts from calibration)\n",
            cellPresent ? "OK" : "NO HX711", cellForceN, cellZero - CELL_ZERO_COUNTS);
}

void printLoop()
{
    Serial.printf("---- CLOSED LOOP ----\nMode         %s%s\n",
            clEnabled ? "ON (TEST runs closed loop on the pot)" : "OFF (TEST runs open loop)",
            clActive ? "  - running now" : "");
    Serial.printf("Compiled in  lambda %.0f um, update every %.0f ms, pot filter %u samples\n",
            CL_LAMBDA_MM * 1000.0f, CL_PERIOD_S * 1000.0f, POT_WIN_N);
    Serial.printf("             k_machine %.0f N/mm (lost telemetry only)\n", MACHINE_K_NMM);
    Serial.println("             Plain PI: u = v + 60 (Kp e + Ki int(e dt)), Kp = (v/60)/lambda, Ki = Kp^2/4.");
    if (clActive)
        Serial.printf("Running      v %.4f mm/min, Kp %.4f 1/s, Ki %.5f 1/s^2, ref %.4f mm, err %+.1f um\n"
                "             u %.4f mm/min, lost %.3f mm\n",
                speedMmMin, loopKp(), loopKi(), clRef, clErr * 1000.0f, motorMmMin, clLost);
    else
        Serial.println("Gains are set when a test starts.");
}

void printHelp()
{
    Serial.println("---- COMMANDS (speeds in mm/min, distances in mm) ----");
    Serial.println("UP <v>                    move up until STOP");
    Serial.println("DOWN <v>                  move down until STOP");
    Serial.println("JOG UP <mm> <v>           move up a set distance");
    Serial.println("JOG DOWN <mm> <v>         move down a set distance");
    Serial.println("TEST TENSION <v>          tension test   = crosshead UP");
    Serial.println("TEST COMPRESSION <v>      compression    = crosshead DOWN");
    Serial.println("SPEED <v>                 change the speed of a running TEST (post-yield speed-up)");
    Serial.println("LOOP ON | OFF             TEST runs closed loop on the pot / open loop");
    Serial.println("STOP                      stop, driver keeps holding");
    Serial.println("RESUME                    clear the E-stop latch, rig stays stopped");
    Serial.println("TARE                      zero position and force");
    Serial.println("POT TARE                  zero position only (force unchanged)");
    Serial.println("MODE SPREAD | STEALTH     chopper mode (not saved, StealthChop at boot)");
    Serial.println("STATUS | LOOP | DRV       rig status / closed-loop state / driver diagnostics");
    Serial.println("POT NOISE                 measure pot noise, raw and filtered (~4.5 s, crosshead still)");
    Serial.println("HELP                      this list");
    Serial.println("Gains, filters, limits and calibrations are compiled in - reflash to change them.");
}

// Commands

void handleCommand(String line)
{
    line.trim();
    line.toUpperCase();
    if (line.length() == 0) return;

    // Split the line into words: "JOG UP 0.5 2" -> words[0] = "JOG", words[1] = "UP", ...
    // Missing words are empty strings.
    String words[5];
    uint8_t n = 0;
    char buf[128];
    line.toCharArray(buf, sizeof(buf));
    for (char *w = strtok(buf, " "); w != nullptr && n < 5; w = strtok(nullptr, " "))
        words[n++] = w;
    const String &cmd = words[0];

    if (cmd == "STOP")
    {
        stopMotion("STOP command");
        Serial.println("STOPPED (holding).");
    }
    else if ((cmd == "UP" || cmd == "DOWN") && n >= 2)
    {
        bool up = (cmd == "UP");
        float v = words[1].toFloat();
        startMotion(up, v, 0, false);
        Serial.printf("MOVING %s at %.4f mm/min until STOP\n", up ? "UP" : "DOWN", v);
    }
    else if (cmd == "JOG" && n >= 4 && (words[1] == "UP" || words[1] == "DOWN"))
    {
        bool up = (words[1] == "UP");
        float mm = words[2].toFloat();
        float v = words[3].toFloat();
        int32_t steps = lroundf(mm * STEPS_PER_MM);
        if (steps < 1) { Serial.println("JOG ignored: shorter than one microstep."); return; } // 0 steps means "until STOP"
        startMotion(up, v, steps, false);
        Serial.printf("JOG %s %.4f mm at %.4f mm/min (%.1f s)\n", up ? "UP" : "DOWN", mm, v, mm / v * 60.0f);
    }
    else if (cmd == "TEST" && n >= 3 && (words[1] == "TENSION" || words[1] == "COMPRESSION"))
    {
        bool tension = (words[1] == "TENSION");
        float v = words[2].toFloat();
        startMotion(tension, v, 0, true);
        if (clEnabled) startClosedLoop();

        Serial.printf("TEST %s STARTED: crosshead moving %s (%s) at %.4f mm/min, %s\n",
                tension ? "TENSION" : "COMPRESSION", tension ? "UP" : "DOWN",
                tension ? "opening" : "closing", v, clEnabled ? "CLOSED LOOP" : "open loop");
        if (clEnabled)
            Serial.printf("CLOSED LOOP: ref starts at %.4f mm, Kp %.4f 1/s, Ki %.5f 1/s^2\n",
                    clRef0, loopKp(), loopKi());
    }
    else if (cmd == "SPEED" && n >= 2) changeTestSpeed(words[1].toFloat());
    else if (cmd == "LOOP")
    {
        if (words[1] == "ON" || words[1] == "OFF")
        {
            clEnabled = (words[1] == "ON");
            Serial.printf("LOOP %s: TEST commands now run %s.\n", clEnabled ? "ON" : "OFF",
                    clEnabled ? "closed loop on the pot" : "open loop (step count)");
        }
        else printLoop();
    }
    else if (cmd == "RESUME")
    {
        estopLatched = false;
        Serial.println("RESUMED: E-stop latch cleared. Rig is stopped.");
    }
    else if (cmd == "TARE")
    {
        potTare();
        cellTare();
    }
    else if (cmd == "POT")
    {
        if (words[1] == "TARE") potTare();
        else if (words[1] == "NOISE") potNoise();
        else Serial.println("POT: TARE | NOISE");
    }
    else if (cmd == "MODE" && (words[1] == "SPREAD" || words[1] == "STEALTH"))
    {
        spreadCycle = (words[1] == "SPREAD");
        driver.en_spreadCycle(spreadCycle);
        Serial.printf("MODE %s\n", spreadCycle ? "SpreadCycle" : "StealthChop (wait 1 s before moving)");
    }
    else if (cmd == "STATUS") printStatus();
    else if (cmd == "DRV") printDrv();
    else if (cmd == "HELP") printHelp();
    else Serial.printf("Unknown command '%s'. Type HELP.\n", line.c_str());
}

// Collect characters into a line; run it when Enter (LF or CR) arrives
void readSerial()
{
    while (Serial.available())
    {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r')
        {
            if (rxLine.length()) handleCommand(rxLine);
            rxLine = "";
        }
        else if (rxLine.length() < 120) rxLine += ch;
    }
}

// Setup and loop

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("=== Mini UTM firmware (PI control) ===");

    pinMode(STEP_PIN, OUTPUT); digitalWrite(STEP_PIN, LOW);
    pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, DIR_UP_LEVEL);
    pinMode(EN_PIN, OUTPUT); digitalWrite(EN_PIN, LOW); // driver enabled = holding
    pinMode(ESTOP_PIN, INPUT);
    pinMode(ENDSTOP_TOP_PIN, INPUT);
    pinMode(ENDSTOP_BTM_PIN, INPUT);

    TMC_SERIAL.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
    delay(100);
    driverConfigure();

    stepTimerStart();

    potBegin();
    Serial.printf("Pot: ADS1115 %s, cal x = %.3f + %.3f p\n",
            potPresent ? "OK" : "NOT FOUND at 0x48", POT_ZERO_MM, POT_MM_PER_RATIO);

    scale.begin(HX711_DOUT, HX711_SCK); // gain 128
    cellPresent = scale.wait_ready_timeout(3000);
    Serial.printf("Load cell: HX711 %s, cal N = %ld - %.3f m\n",
            cellPresent ? "OK" : "NOT FOUND", CELL_ZERO_COUNTS, CELL_COUNTS_PER_G);

    Serial.printf("Direction: TENSION = UP, COMPRESSION = DOWN (DIR_UP_LEVEL = %s)\n",
            DIR_UP_LEVEL == HIGH ? "HIGH" : "LOW");
    Serial.println("Closed loop is OFF at boot. LOOP ON makes TEST commands closed loop.");
    Serial.println("Rig STOPPED and holding. TARE before each test. Type HELP.");
}

void loop()
{
    readSerial();
    serviceSafety();
    serviceJog();
    servicePot();
    serviceClosedLoop();
    serviceCell();

    uint32_t now = millis();
    if (now - lastDrvPoll >= 1000) // driver temperature, once a second (telemetry only)
    {
        lastDrvPoll = now;
        pollDriverTemp();
    }
    if (now - lastTele >= TELE_MS)
    {
        lastTele = now;
        printTelemetry();
    }
}
