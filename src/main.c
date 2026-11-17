/*
 * NUCLEO-L432KC — One-shot Sign Capture (2 s window) + Orientation (LSM6D*) + ESP8266
 * ChibiOS 21.11
 *
 * Website output: ONLY "text:<LABEL>" when a rule matches.
 * Examples: "text:HELLO", "text:PLEASE", "text:THANK YOU"
 *
 * P = 2 -> steso
 *     0 -> chiuso
 *
 * I = 2 -> steso
 *     1 -> metà
 *     0 -> chiuso
 *
 * M = 2 -> steso
 *     1 -> metà
 *     0 -> chiuso
 * A = 2 -> steso
 *     1 -> metà
 *     0 -> chiuso
 * M = 2 -> steso
 *     0 -> chiuso
 *
 *
  { ORI_LANDSCAPE_RIGHT, 0b11111, "PLEASE" },

 */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include <math.h>
#include <string.h>

/* ================= Tunables ================= */
#define VREF                3.3f
#define ADC_RES             4095.0f

/* 2-second capture window, sample every 50 ms (end-of-window used for final state) */
#define CAPTURE_MS          1300
#define SAMPLE_MS           50

/* Button debounce */
#define DEBOUNCE_MS         30

/* Finger thresholds (Volts). If v >= THR -> finger considered STRAIGHT (bit=1). */
#define FINGER_STRAIGHT_THR 1.20f

/* Sequence detection stability & gap */
#define SEQ_HOLD_SAMPLES    2     /* ~100 ms (2 * 50 ms) stable for each step */
#define SEQ_MIN_GAP_MS      150   /* at least 150 ms between step1 and step2 */

/* ================= Serial ports ================= */
static const SerialConfig sd1cfg = {38400, 0, 0, 0};   /* USART1 -> ESP */
BaseSequentialStream *esp = (BaseSequentialStream *)&SD1;
BaseSequentialStream *chp = (BaseSequentialStream *)&SD2;

/* ================= I2C1 (for LSM6D* IMU) ================= */
static const I2CConfig i2ccfg = {
  .timingr = 0x10909CEC, /* ~100 kHz; adjust for your clock if needed */
  .cr1     = 0,
  .cr2     = 0
};
#define I2C_TMO   TIME_MS2I(50)

/* ===== LSM6D* registers & values ===== */
#define ADDR_6A   0x6A  /* SDO=GND */
#define ADDR_6B   0x6B  /* SDO=VDD */

#define REG_WHO_AM_I   0x0F  /* LSM6DSOX=0x6C, LSM6DSL=0x6A, LSM6DS3=0x69 */
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_OUTX_L_A   0x28

#define CTRL3_SW_RESET (1U<<0)
#define CTRL3_IF_INC   (1U<<2)
#define CTRL3_BDU      (1U<<6)

#define WHO_DSOX  0x6C
#define WHO_DSL   0x6A
#define WHO_DS3   0x69

static uint8_t g_addr = 0;
static uint8_t g_who  = 0;

/* Accelerometer sensitivity @±2g: 0.061 mg/LSB = 0.000061 g/LSB */
#define SENS_2G_G_PER_LSB  (0.000061f)

/* ================= Orientation ================= */
typedef enum {
  ORI_UNKNOWN = 0,
  ORI_FACE_UP,
  ORI_FACE_DOWN,
  ORI_LANDSCAPE_LEFT,   /* +X */
  ORI_LANDSCAPE_RIGHT,  /* -X */
  ORI_PORTRAIT_UP,      /* +Y */
  ORI_PORTRAIT_DOWN     /* -Y */
} orientation_t;

#define THR_DOM_G  0.75f
#define THR_FLAT_G 0.60f

/* ================= Threads ================= */
static THD_WORKING_AREA(waCapture, 1024);

/* ================= Helpers ================= */

/* === Calibrazione flex (percentuali) === */
static const float delta[5] = {0.6523f, 0.7293f, 0.7856f, 0.7731f, 0.7699f};
static const float u0[5]    = {1.6792f, 1.8281f, 1.8629f, 1.7720f, 1.6321f};
static const float u100[5]  = {2.3316f, 2.5574f, 2.6484f, 2.5450f, 2.4020f};

/* I2C helpers */
static void i2c_print_status(const char *tag, msg_t st) {
  i2cflags_t e = i2cGetErrors(&I2CD1);
  chprintf(chp, "%s: st=%d, errs=0x%04X\r\n", tag, st, (unsigned)e);
}
static msg_t i2c_wr1(uint8_t dev7, uint8_t reg, uint8_t val) {
  uint8_t tx[2] = {reg, val};
  msg_t st = i2cMasterTransmitTimeout(&I2CD1, dev7, tx, 2, NULL, 0, I2C_TMO);
  if (st != MSG_OK) i2c_print_status("WR1", st);
  return st;
}
static msg_t i2c_rd(uint8_t dev7, uint8_t reg, uint8_t *rx, size_t n) {
  msg_t st = i2cMasterTransmitTimeout(&I2CD1, dev7, &reg, 1, rx, n, I2C_TMO);
  if (st != MSG_OK) i2c_print_status("RD", st);
  return st;
}

/* Probe + configure IMU */
static bool lsm6_detect(uint8_t addr, uint8_t *who) {
  uint8_t id = 0;
  if (i2c_rd(addr, REG_WHO_AM_I, &id, 1) != MSG_OK) return false;
  *who = id;
  return (id == WHO_DSOX) || (id == WHO_DSL) || (id == WHO_DS3);
}
static bool lsm6_configure(uint8_t addr) {
  if (i2c_wr1(addr, REG_CTRL3_C, CTRL3_SW_RESET) != MSG_OK) return false;
  chThdSleepMilliseconds(15);
  if (i2c_wr1(addr, REG_CTRL3_C, CTRL3_IF_INC | CTRL3_BDU) != MSG_OK) return false;

  /* Accelerometer: ODR=104 Hz, FS=±2g */
  uint8_t ctrl1_xl = (0b0100 << 4) | (0b00 << 2) | 0x00;
  if (i2c_wr1(addr, REG_CTRL1_XL, ctrl1_xl) != MSG_OK) return false;

  /* Gyro OFF */
  if (i2c_wr1(addr, REG_CTRL2_G, 0x00) != MSG_OK) return false;

  return true;
}
static bool lsm6_init_auto(void) {
  uint8_t who = 0;
  if (lsm6_detect(ADDR_6A, &who)) { g_addr = ADDR_6A; g_who = who; return lsm6_configure(g_addr); }
  if (lsm6_detect(ADDR_6B, &who)) { g_addr = ADDR_6B; g_who = who; return lsm6_configure(g_addr); }
  return false;
}
static bool lsm6_read_accel_g(float *ax, float *ay, float *az) {
  uint8_t raw[6];
  if (i2c_rd(g_addr, REG_OUTX_L_A, raw, 6) != MSG_OK) return false;
  int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
  int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
  int16_t z = (int16_t)((raw[5] << 8) | raw[4]);
  *ax = x * SENS_2G_G_PER_LSB;
  *ay = y * SENS_2G_G_PER_LSB;
  *az = z * SENS_2G_G_PER_LSB;
  return true;
}

/* Orientation classify */
static orientation_t classify_orientation(float ax, float ay, float az) {
  float aX=fabsf(ax), aY=fabsf(ay), aZ=fabsf(az);
  if (aZ > THR_FLAT_G && aZ > aX && aZ > aY)
    return (az >= 0.0f) ? ORI_FACE_UP : ORI_FACE_DOWN;
  if (aX > THR_DOM_G && aX > aY && aX > aZ)
    return (ax >= 0.0f) ? ORI_LANDSCAPE_LEFT : ORI_LANDSCAPE_RIGHT;
  if (aY > THR_DOM_G && aY > aX && aY > aZ)
    return (ay >= 0.0f) ? ORI_PORTRAIT_UP : ORI_PORTRAIT_DOWN;
  return ORI_UNKNOWN;
}

/* You asked to swap the printed labels for left/right (debug text only) */
static const char* ori_to_text(orientation_t o) {
  switch (o) {
    case ORI_FACE_UP:         return "face_up";
    case ORI_FACE_DOWN:       return "face_down";
    case ORI_LANDSCAPE_LEFT:  return "landscape_left";
    case ORI_LANDSCAPE_RIGHT: return "landscape_right";
    case ORI_PORTRAIT_UP:     return "portrait_up";
    case ORI_PORTRAIT_DOWN:   return "portrait_down";
    default:                  return "unknown";
  }
}

/* Read fingers once -> output percentages (0..100) and STRAIGHT mask.
 * STRAIGHT mask still uses the original voltage threshold so rules remain identical. */
static void read_fingers_percent(int pcent[5], uint8_t *mask_out) {
  static const ADCConversionGroup adcgrpcfg = {
    .circular     = false,
    .num_channels = 5,
    .end_cb       = NULL,
    .error_cb     = NULL,
    .cfgr         = 0,
    .cfgr2        = 0U,
    .tr1          = ADC_TR_DISABLED,
    .tr2          = ADC_TR_DISABLED,
    .tr3          = ADC_TR_DISABLED,
    .awd2cr       = 0U,
    .awd3cr       = 0U,
    .smpr = {
      ADC_SMPR1_SMP_AN5(ADC_SMPR_SMP_247P5) |
      ADC_SMPR1_SMP_AN6(ADC_SMPR_SMP_247P5) |
      ADC_SMPR1_SMP_AN9(ADC_SMPR_SMP_247P5),
      ADC_SMPR2_SMP_AN15(ADC_SMPR_SMP_247P5)|
      ADC_SMPR2_SMP_AN16(ADC_SMPR_SMP_247P5),
    },
    .sqr = {
      ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5) |
      ADC_SQR1_SQ2_N(ADC_CHANNEL_IN6) |
      ADC_SQR1_SQ3_N(ADC_CHANNEL_IN9) |
      ADC_SQR1_SQ4_N(ADC_CHANNEL_IN15),
      ADC_SQR2_SQ5_N(ADC_CHANNEL_IN16),
      0U, 0U
    }
  };

  adcsample_t samples[5];
  float v[5];
  adcConvert(&ADCD1, &adcgrpcfg, samples, 1);

  /* Convert to volts (for straight-mask threshold) + compute percentages */
  uint8_t m = 0;
  for (int i = 0; i < 5; ++i) {
    v[i] = ((float)samples[i] * VREF) / ADC_RES;

    /* Straightness mask preserved using original voltage threshold */
    if (v[i] >= FINGER_STRAIGHT_THR) m |= (1U << i);

    /* Percentage with clamping 0..100 */
    int p = (int)(100.0f * ((v[i] - u0[i]) / delta[i]));
    if (v[i] > u100[i]) p = 100;
    if (v[i] < u0[i])   p = 0;
    pcent[i] = p;

  }
  *mask_out = m;
}

/* ================= Sign rules ================= */
typedef struct {
  orientation_t ori;
  uint8_t straight_mask;  /* bit i = 1 means finger i is STRAIGHT */
  const char* label;      /* sign name to output */
} SignRule;

/* Single-pose rules (bit 0 = F0 (PA0) ... bit 4 = F4 (PA6)) */
static const SignRule RULES[] = {
  { ORI_PORTRAIT_UP,     0b11111, "HELLO"  },
  { ORI_LANDSCAPE_RIGHT, 0b11111, "PLEASE" },
};
static const size_t RULES_N = sizeof(RULES)/sizeof(RULES[0]);

static const char* match_sign(orientation_t o, uint8_t straight_mask) {
  for (size_t i=0; i<RULES_N; ++i) {
    if (RULES[i].ori == o && RULES[i].straight_mask == straight_mask) return RULES[i].label;
  }
  return NULL;
}

/* ====== Sequence rule support ====== */
typedef struct {
  orientation_t s1_ori;
  uint8_t       s1_mask;    /* straight mask for step 1 */
  orientation_t s2_ori;
  uint8_t       s2_mask;    /* straight mask for step 2 */
  const char*   label;
} SeqRule;

/* THANK YOU: portrait_up + 11111 -> face_down + 11111 */
static const SeqRule SEQ_RULES[] = {
  { ORI_PORTRAIT_UP, 0b11111, ORI_FACE_DOWN, 0b11111, "THANK YOU" },
};
static const size_t SEQ_RULES_N = sizeof(SEQ_RULES)/sizeof(SEQ_RULES[0]);

/* Send "text:<msg>\n" to ESP */
static void esp_send_text(const char* msg) {
  if (!msg) return;
  chprintf(esp, "text:%s\n", msg);
}

/* ================= Pin mux ================= */
static void pins_init(void) {
  /* USART2 (PC log) */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7)); // USART2_TX
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7)); // USART2_RX

  /* ADC inputs */
  palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);  // F0
  palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);  // F1
  palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);  // F2
  palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);  // F3
  palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);  // F4

  /* Button (pull-up, active LOW) */
  palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_PULLUP);

  /* USART1 (ESP) */
  palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));

  /* I2C1: PB6=SCL, PB7=SDA (AF4, open-drain, pull-up, high speed) */
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4) |
                        PAL_STM32_OTYPE_OPENDRAIN |
                        PAL_STM32_PUPDR_PULLUP |
                        PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4) |
                        PAL_STM32_OTYPE_OPENDRAIN |
                        PAL_STM32_PUPDR_PULLUP |
                        PAL_STM32_OSPEED_HIGHEST);
}

/* =============== Capture Thread =============== */
static THD_FUNCTION(CaptureThread, arg) {
  (void)arg;
  chRegSetThreadName("sign_capture");

  /* IMU init */
  if (!lsm6_init_auto()) {
    chprintf(chp, "LSM6D* init FAIL (no recognized WHO_AM_I)\r\n");
    while (true) chThdSleepMilliseconds(1000);
  }
  const char *chip =
      (g_who==WHO_DSOX) ? "LSM6DSOX" :
      (g_who==WHO_DSL)  ? "LSM6DSL"  :
      (g_who==WHO_DS3)  ? "LSM6DS3"  : "LSM6D*";
  chprintf(chp, "%s init OK @0x%02X (WHO=0x%02X)\r\n", chip, g_addr, g_who);

  /* Button state for edge detect + debounce */
  bool btn_prev = (palReadPad(GPIOA, 7) == PAL_LOW);
  systime_t last_change = chVTGetSystemTime();

  /* Work buffers (redeclared per capture) */
  chprintf(chp, "Ready. Press button to capture (%d ms window).\r\n", (int)CAPTURE_MS);

  while (true) {
    bool btn_now = (palReadPad(GPIOA, 7) == PAL_LOW);

    /* Debounce edge (HIGH->LOW means pressed) */
    if (btn_now != btn_prev) {
      systime_t now = chVTGetSystemTime();
      if (TIME_I2MS(now - last_change) > DEBOUNCE_MS) {
        last_change = now;
        btn_prev = btn_now;

        if (btn_now) {
          /* ======= START CAPTURE ======= */
          float ax=0, ay=0, az=0;
          int p[5] = {0};                /* percentages 0..100 */
          uint8_t straight_mask = 0;

          /* sequence detection state across all sequence rules */
          const char* seq_match_label = NULL;
          size_t seq_hold1[SEQ_RULES_N]; memset(seq_hold1, 0, sizeof(seq_hold1));
          size_t seq_hold2[SEQ_RULES_N]; memset(seq_hold2, 0, sizeof(seq_hold2));
          uint8_t seq_state[SEQ_RULES_N]; memset(seq_state, 0, sizeof(seq_state)); /* 0=wait s1, 1=wait s2 */
          systime_t seq_t_step1[SEQ_RULES_N]; memset(seq_t_step1, 0, sizeof(seq_t_step1));

          systime_t deadline = chVTGetSystemTime() + TIME_MS2I(CAPTURE_MS);

          while (chVTGetSystemTimeX() < deadline) {
            (void)lsm6_read_accel_g(&ax,&ay,&az);         /* keep last accel */
            read_fingers_percent(p, &straight_mask);      /* keep last percentages + straight mask */
            orientation_t o = classify_orientation(ax, ay, az);

            /* ==== sequence check for all seq rules ==== */
            for (size_t r=0; r<SEQ_RULES_N; ++r) {
              if (seq_match_label) break; /* already matched */

              if (seq_state[r] == 0) {
                /* waiting for stable step1 */
                if (o == SEQ_RULES[r].s1_ori && straight_mask == SEQ_RULES[r].s1_mask) {
                  if (++seq_hold1[r] >= SEQ_HOLD_SAMPLES) {
                    seq_state[r]   = 1;
                    seq_t_step1[r] = chVTGetSystemTime();
                  }
                } else {
                  seq_hold1[r] = 0;
                }
              } else {
                /* waiting for stable step2 after min gap */
                if (TIME_I2MS(chVTGetSystemTime() - seq_t_step1[r]) >= SEQ_MIN_GAP_MS) {
                  if (o == SEQ_RULES[r].s2_ori && straight_mask == SEQ_RULES[r].s2_mask) {
                    if (++seq_hold2[r] >= SEQ_HOLD_SAMPLES) {
                      seq_match_label = SEQ_RULES[r].label; /* sequence recognized */
                      break;
                    }
                  } else {
                    seq_hold2[r] = 0;
                  }
                }
              }
            }

            chThdSleepMilliseconds(SAMPLE_MS);
          }

          /* Final decision: prefer a sequence match; else single-pose using last sample */
          if (seq_match_label) {
            esp_send_text(seq_match_label);
          } else {
            orientation_t o_final = classify_orientation(ax, ay, az);
            const char* single = match_sign(o_final, straight_mask);
            if (single) esp_send_text(single);
          }

          /* PC debug from last sample */
          char fbits[6];
          for (int i=0;i<5;++i) fbits[4-i] = (straight_mask & (1U<<i)) ? '1' : '0';
          fbits[5] = '\0';
          const char* otext = ori_to_text(classify_orientation(ax, ay, az));
          chprintf(chp,
            "[CAPTURE] last ori=%s  ax=%.3f ay=%.3f az=%.3f  fingers(straight)=%s  P=%d%% %d%% %d%% %d%% %d%%  %s\r\n",
            otext, ax, ay, az, fbits, p[0], p[1], p[2], p[3], p[4],
            seq_match_label ? "[SEQ matched]" : "");
        }
      }
    }

    chThdSleepMilliseconds(5);
  }
}

/* ================= main() ================= */
int main(void) {
  halInit();
  chSysInit();

  pins_init();

  /* Start serial ports */
  sdStart(&SD1, &sd1cfg);   /* ESP link */
  sdStart(&SD2, NULL);      /* PC log */

  /* Start I2C for IMU */
  i2cStart(&I2CD1, &i2ccfg);

  /* Start ADC */
  adcStart(&ADCD1, NULL);

  chprintf(chp, "System init. Waiting for button press...\r\n");

  /* Single capture thread (no continuous streaming) */
  chThdCreateStatic(waCapture, sizeof(waCapture), NORMALPRIO, CaptureThread, NULL);

  while (true) chThdSleepMilliseconds(1000);
}
