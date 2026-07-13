/* =============================================================
 *  cp_analyzer.c — IEC 61851-1 CP Signal Analyzer  (STM32G4)
 *
 *  Two independent PWM-domain measurements, both via TIM Input
 *  Capture — no ADC is used anywhere in this file.
 *
 *  CHANNEL 1 — CP signal itself (frequency + duty → current limit)
 *    TIM1  → PA0 — PWM Input Mode (CH1=period, CH2=pulse)
 *    (unchanged from the original design)
 *
 *  CHANNEL 2 — CP amplitude, re-encoded as duty cycle
 *    A peak detector holds the CP signal's positive amplitude as a
 *    slowly-varying DC level. A Schmitt-trigger + integrator forms
 *    a free-running triangle-wave oscillator that ramps between
 *    TRIANGLE_LOW_MV and TRIANGLE_HIGH_MV. A comparator compares
 *    the held peak-detector DC level against this triangle, so the
 *    comparator's output duty cycle is a linear function of the CP
 *    amplitude. That comparator output is opto-isolated (PC817)
 *    and captured the same way as channel 1.
 *
 *    TIM3  → PB4/PB5 — PWM Input Mode (CH1=period, CH2=pulse)
 *    NOTE: pins/timer assumed — swap for whatever TIM3 (or another
 *    free 16-bit timer) pair you've actually wired in CubeMX.
 *
 *  This replaces the old ADC1/PA1 peak-detector-to-ADC path: the
 *  amplitude now crosses the isolation barrier as a PWM duty cycle
 *  instead of an analog voltage, matching the actual hardware.
 *
 *  OPTO_INV / AMP_OPTO_INV:
 *    0 = signal not inverted by its opto stage
 *    1 = PC817 stage inverts that channel, so duty = 100 - raw_duty
 * =============================================================*/

#include "cp_analyzer.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

/* ---- Build-time config ----------------------------------------*/
int OPTO_INV     = 1;         /* channel 1 (CP signal) opto inversion   */
int AMP_OPTO_INV  = 1;         /* channel 2 (amplitude PWM) opto inversion */
int TIM_CLK       = 10000000UL; /* TIM1 tick rate: 170MHz / (PSC+1) = 170MHz/17 */
int EDGE_TIMEOUT_MS = 5U;      /* no edge for 5 ms → treat channel as dead   */
int TASK_PERIOD_MS  = 500U;    /* decode/report interval                     */

/* ---- Triangle-wave reference (Schmitt trigger + integrator) ---
 * The oscillator ramps between these two rails. The comparator's
 * duty cycle vs. CP peak DC level is:
 *     duty(%) = (TRIANGLE - V_peak) / (TRIANGLE_HIGH - TRIANGLE_LOW) * 100
 * so decoding is just the inverse of that, in millivolts.        */
int TRIANGLE_LOW_MV  = -200;
int TRIANGLE_HIGH_MV = 3000;

/* ---- State confirmation debounce ------------------------------
 * next_state() must return the same new state CONFIRM_COUNT times
 * in a row before cp.state is updated.  This prevents a single
 * noisy reading from flipping the displayed state.
 * At 500 ms period: CONFIRM_COUNT=2 → 500 ms debounce delay.    */
int CONFIRM_COUNT = 2;

/* ---- Amplitude thresholds with hysteresis (mV, decoded from
 *      the channel-2 duty cycle via the triangle-wave mapping) --
 * Report boundaries:   A≥2600  B/C@1837  D@400  E/F<400
 * ~150 mV dead-band around each boundary stops bouncing when the
 * decoded level sits near a threshold.
 *
 *   ENTER state when level crosses the _HI threshold (rising)
 *   LEAVE  state when level falls below  the _LO threshold
 * ----------------------------------------------------------------*/
int THR_A_HI  = 2700;   /* enter A                      */
int THR_A_LO  = 2500;   /* leave A → D or E/F           */
int THR_BC_HI = 1900;   /* enter B  (from C or no-PWM)  */
int THR_BC_LO = 1770;   /* leave B → C                  */
int THR_D_HI  = 500;    /* enter D  (from E/F)          */
int THR_D_LO  = 350;    /* leave D → E/F                */

/* ---- IC capture state (written in ISR, read in task) ----------*/
typedef struct {
    uint32_t period;    /* CCR1: full period in timer ticks  */
    uint32_t pulse;     /* CCR2: high-phase in timer ticks   */
    uint8_t  valid;     /* 1 once first capture received     */
    uint32_t edge_ts;   /* HAL_GetTick() at last capture     */
} IC_t;

/* ---- CP decoder state -----------------------------------------*/
typedef enum {
    CP_UNKNOWN = 0, CP_A, CP_B, CP_C, CP_D, CP_E, CP_F
} CPState_t;

typedef struct {
    CPState_t state;    /* committed current state           */
    CPState_t prev;     /* previous state (for change print) */
    CPState_t pending;  /* candidate next state              */
    uint8_t   confirm;  /* consecutive reads matching pending */
    float     freq;     /* Hz  (channel 1 — CP signal)        */
    float     duty;     /* %   (channel 1, opto-corrected)   */
    float     i_max;    /* A — IEC 61851-1 current limit     */
    float     amp_duty; /* %   (channel 2 — amplitude PWM)   */
    int32_t   amp_mv;   /* mV — CP amplitude decoded from amp_duty */
    uint8_t   had_pwm;  /* 1 if channel-1 PWM was ever seen (E vs F) */
} CP_t;

static volatile IC_t ic     = {0};  /* channel 1 — CP signal (freq/duty)   */
static volatile IC_t ic_amp = {0};  /* channel 2 — amplitude-as-duty       */
static          CP_t cp     = {0};
static uint32_t      task_ts = 0;

/* =============================================================
 *  IEC 61851-1 Table A.2 — duty cycle → max charge current
 *    8 % to 85 % :  I = duty × 0.6
 *   85 % to 97 % :  I = (duty − 64) × 2.5
 * =============================================================*/
static float duty_to_amps(float d)
{
    if (d < 8.0f || d > 97.0f) return 0.0f;
    if (d <= 85.0f)             return d * 0.6f;
    return (d - 64.0f) * 2.5f;
}

static const char *state_name(CPState_t s)
{
    switch (s) {
        case CP_A: return "A  Standby (no EV)";
        case CP_B: return "B  EV Connected";
        case CP_C: return "C  Charging";
        case CP_D: return "D  Charging + Ventilation";
        case CP_E: return "E  FAULT — CP shorted/open";
        case CP_F: return "F  EVSE Internal Fault";
        default:   return "?  Unknown";
    }
}

/* =============================================================
 *  amplitude_decode_mv() — channel 2
 *
 *  Converts the channel-2 duty cycle back into the CP peak voltage
 *  (mV) it represents, using the known triangle-wave excursion.
 *  On a timeout (oscillator/opto channel dead) the last decoded
 *  value is held rather than snapping to 0, since a genuine fault
 *  condition is already handled by next_state()'s E/F logic.
 * =============================================================*/
static int32_t amplitude_decode_mv(uint8_t *ok)
{
    uint32_t now   = HAL_GetTick();
    uint8_t  valid = (ic_amp.valid
                      && (now - ic_amp.edge_ts) < EDGE_TIMEOUT_MS
                      && ic_amp.period > 0);
    *ok = valid;
    if (!valid) return cp.amp_mv;

    float duty2 = ((float)ic_amp.pulse / (float)ic_amp.period) * 100.0f;
#if AMP_OPTO_INV
    duty2 = 100.0f - duty2;
#endif
    if (duty2 < 0.0f)   duty2 = 0.0f;
    if (duty2 > 100.0f) duty2 = 100.0f;
    cp.amp_duty = duty2;

    float mv = TRIANGLE_HIGH_MV
             - (duty2 / 100.0f) * (float)(TRIANGLE_HIGH_MV - TRIANGLE_LOW_MV);
    return (int32_t)mv;
}

/* =============================================================
 *  next_state() — pure function, no side effects.
 *
 *  Returns the state the decoder WANTS to go to given the
 *  current amplitude-derived level and channel-1 PWM presence.
 *  The caller applies the CONFIRM_COUNT debounce before committing.
 *
 *  Hysteresis rules (prevents threshold bouncing):
 *    • We know which state we're currently IN, so we only cross
 *      a boundary when the level moves past the correct edge
 *      (_HI to enter, _LO to leave).
 *    • For states reached by a "hard-snap" (UNKNOWN / B / C
 *      losing PWM) hysteresis has not yet been established, so
 *      we snap once and let hysteresis take over next cycle.
 *      CONFIRM_COUNT absorbs any one-shot noise here.
 * =============================================================*/
static CPState_t next_state(uint8_t has_pwm, int32_t amp_mv)
{
    CPState_t cur = cp.state;

    /* ---- Channel-1 PWM present --------------------------------*/
    if (has_pwm)
    {
        /* Very low amplitude with PWM edges present = noise on a
         * floating/open channel. Treat as fault.                */
        if (amp_mv < THR_D_LO) return CP_E;

        /* B ↔ C with hysteresis */
        if (cur == CP_B) return (amp_mv < THR_BC_LO) ? CP_C : CP_B;
        if (cur == CP_C) return (amp_mv >= THR_BC_HI) ? CP_B : CP_C;

        /* Entering from a no-PWM state — hard snap once */
        return (amp_mv >= THR_BC_HI) ? CP_B : CP_C;
    }

    /* ---- No channel-1 PWM --------------------------------------*/

    /* Currently A: leave only when level drops below A_LO */
    if (cur == CP_A)
    {
        if (amp_mv < THR_A_LO)
            return (amp_mv >= THR_D_HI) ? CP_D : (cp.had_pwm ? CP_F : CP_E);
        return CP_A;
    }

    /* Currently D: leave upward to A, or downward to E/F */
    if (cur == CP_D)
    {
        if (amp_mv >= THR_A_HI) return CP_A;
        if (amp_mv <  THR_D_LO) return cp.had_pwm ? CP_F : CP_E;
        return CP_D;
    }

    /* Currently E or F: can only recover if level rises */
    if (cur == CP_E || cur == CP_F)
    {
        if (amp_mv >= THR_A_HI) return CP_A;
        if (amp_mv >= THR_D_HI) return CP_D;
        return cur;           /* stay in fault */
    }

    /* UNKNOWN / B / C with no PWM — hard snap.
     * Happens once at startup or immediately after PWM loss.
     * CONFIRM_COUNT debounce absorbs transient noise here.      */
    if (amp_mv >= THR_A_HI) return CP_A;
    if (amp_mv >= THR_D_HI) return CP_D;
    return cp.had_pwm ? CP_F : CP_E;
}

/* =============================================================
 *  decode() — called every TASK_PERIOD_MS ms
 *
 *  1. Decode channel-2 duty → amplitude (mV)
 *  2. Calculate channel-1 PWM metrics (freq/duty → current limit)
 *  3. Compute candidate next state (pure, no side effects)
 *  4. Apply CONFIRM_COUNT debounce before committing
 * =============================================================*/
static void decode(void)
{
    uint32_t now     = HAL_GetTick();
    uint8_t  has_pwm = (ic.valid
                        && (now - ic.edge_ts) < EDGE_TIMEOUT_MS
                        && ic.period > 0);

    uint8_t amp_ok = 0;
    cp.amp_mv = amplitude_decode_mv(&amp_ok);

    if (has_pwm)
    {
        cp.had_pwm = 1;
        cp.freq    = (float)TIM_CLK / (float)ic.period;

        float raw = ((float)ic.pulse / (float)ic.period) * 100.0f;
#if OPTO_INV
        raw = 100.0f - raw;
#endif
        cp.duty  = (raw < 0.0f) ? 0.0f : (raw > 100.0f) ? 100.0f : raw;
        cp.i_max = duty_to_amps(cp.duty);
    }
    else
    {
        cp.freq = cp.duty = cp.i_max = 0.0f;
    }

    /* ---- State confirmation debounce ------------------------*/
    CPState_t cand = next_state(has_pwm, cp.amp_mv);

    if (cand == cp.state)
    {
        cp.pending = cand;
        cp.confirm = 0;
    }
    else if (cand == cp.pending)
    {
        if (++cp.confirm >= CONFIRM_COUNT)
        {
            cp.state   = cand;
            cp.pending = cand;
            cp.confirm = 0;
        }
    }
    else
    {
        cp.pending = cand;
        cp.confirm = 1;
    }
}

/* =============================================================
 *  report() — print to SWV ITM console
 * =============================================================*/
static void report(void)
{
    if (cp.state != cp.prev)
    {
        printf("\r\n>>> STATE: %s  -->  %s\r\n",
               state_name(cp.prev), state_name(cp.state));
        cp.prev = cp.state;
    }

    if (cp.state == CP_B || cp.state == CP_C)
        printf("[CP] %-32s | %7.1f Hz | %5.1f%% | %5.1f A | Vamp %5ld mV (%5.1f%%)\r\n",
               state_name(cp.state),
               cp.freq, cp.duty, cp.i_max, (long)cp.amp_mv, cp.amp_duty);
    else
        printf("[CP] %-32s | Vamp %5ld mV (%5.1f%%)\r\n",
               state_name(cp.state), (long)cp.amp_mv, cp.amp_duty);
}

/* =============================================================
 *  TIM Input Capture ISR Callback — shared by both channels
 *
 *  PWM Input Mode fires TWO capture events per cycle:
 *    CH1 (rising edge)  → CCR1 = full period
 *    CH2 (falling edge) → CCR2 = pulse width
 *  We act only on CH1; CCR2 is already latched by hardware.
 * =============================================================*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        ic.period  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        ic.pulse   = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        ic.valid   = 1;
        ic.edge_ts = HAL_GetTick();
    }
    else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        ic_amp.period  = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        ic_amp.pulse   = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        ic_amp.valid   = 1;
        ic_amp.edge_ts = HAL_GetTick();
    }
}

/* =============================================================
 *  Public API
 * =============================================================*/
void CP_Init(void)
{
    memset((void *)&ic,     0, sizeof(ic));
    memset((void *)&ic_amp, 0, sizeof(ic_amp));
    memset(&cp, 0, sizeof(cp));
    cp.state = cp.prev = cp.pending = CP_UNKNOWN;

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);

    printf("\r\n========================================\r\n");
    printf("  STM32G4 IEC 61851-1 CP Analyzer\r\n");
    printf("  Ch1: CP PWM (freq/duty -> current limit)\r\n");
    printf("  Ch2: Amplitude-as-duty (triangle+comparator -> state)\r\n");
    printf("========================================\r\n\r\n");
}

void CP_Task(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - task_ts) >= TASK_PERIOD_MS)
    {
        task_ts = now;
        decode();
        report();
    }
}
