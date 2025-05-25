
// #include <Event.h>
// #include <Logic.h>

#include <avr/sleep.h>

#define TEST_TIME 0

#define SLEEP_TIME  60 // 300 // 900

#define led       PIN_PA7
#define reset_ESP PIN_PA3

#define BANDGAP 1 // No regulator
//#define ANALOG_INPUT PIN_PA1  // 10K NTC

#define USE_ESP 0

#define RTC_PERIOD	0x8000 // Overflow at 32768 ticks from RTC
#define RTC_MILLIS	1000   // That matches 1000ms

#if USE_ESP
#include <Wire.h>
#endif

#define SLEEPINT	// Do not count PIT interrupts but use RTC interrupt at exact time


void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
    ;                                   /* Wait for all register to be synchronized */
  RTC.PER = RTC_PERIOD - 1;		/* 1 sec overflow */
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  // For overflow ticks while sleeping:
  // RTC.INTCTRL = RTC_OVF_bm;

  RTC.CTRLA |= RTC_RTCEN_bm | RTC_RUNSTDBY_bm;

#ifndef SLEEPINT
  RTC.PITCTRLA = RTC_PITEN_bm;    /* Disable PIT counter, should not be done, see erratum */
#endif
}

#ifdef SLEEPINT
volatile uint8_t sleep_cnt;

ISR(RTC_CNT_vect)
{
  // if (RTC.INTFLAGS & RTC_CMP_bm) {
    sleep_cnt--;
    RTC.INTFLAGS = RTC_CMP_bm;          /* Clear interrupt flag by writing '1' (required) */
  // }
  // if (RTC.INTFLAGS & RTC_OVF_bm) {
  //   RTC.INTFLAGS = RTC_OVF_bm;          /* Clear interrupt flag by writing '1' (required) */
  // }
}
#else
volatile uint16_t ticks;

ISR(RTC_PIT_vect)
{
  if (ticks)
    ticks--;

  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
}
#endif


void sleepDelay(uint16_t n)
{
  ADC0.CTRLA &= ~ADC_ENABLE_bm; //Very important on the tinyAVR 2-series
#ifndef SLEEPINT
  uint16_t cticks;
  uint8_t period;

  if (n < 100) {
      period = RTC_PERIOD_CYC8_gc; // 1/4 ms
      cticks = (n << 2);
  } else if (n < 1000) {
      period = RTC_PERIOD_CYC64_gc; // 2 ms
      cticks = (n >> 1);
  } else if (n < 5000) {
      period = RTC_PERIOD_CYC1024_gc; // 32 ms
      cticks = (n >> 5);
  } else {
      period = RTC_PERIOD_CYC8192_gc; // 256 ms
      cticks = (n >> 8);
  }

  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm)  // Wait for new settings to synchronize
    ;
  RTC.PITCTRLA = period | RTC_PITEN_bm;    // Enable PIT counter: enabled
 
  RTC.PITINTCTRL |= RTC_PI_bm;             /* PIT Interrupt: enabled */
  ticks = cticks;

  while (ticks)
    sleep_cpu();

  //RTC.PITCTRLA = 0;    /* Disable PIT counter, should not be done, see erratum */
  RTC.PITINTCTRL &= ~RTC_PI_bm;           /* PIT Interrupt: disabled */
#else
  while (RTC.STATUS /* & RTC_CMPBUSY_bm */)  // Wait for new settings to synchronize
    ;

  uint32_t tdelay;
  noInterrupts();
  uint16_t cnt = RTC.CNT;
  RTC.CMP = (cnt + (tdelay = (n * 32UL) + uint16_t(n / 4 * 3))) & (RTC_PERIOD-1); // With this calculation every multiple of 4ms is exact!
  // Note 20011 ms = 655360 ticks = 20 overflows exactly, so CMP will be set to the current CNT and this works OK
  // 9005 will set CMP to CNT+1

  if (RTC.CNT == RTC.CMP) {
    tdelay -= RTC_PERIOD;
  } else if (((RTC.CMP - cnt) & (RTC_PERIOD-1)) <= 2) { // overflow is/was near, 4Mhz clock or faster
    while (RTC.CNT != RTC.CMP) // Wait for it...
      ;
    tdelay -= RTC_PERIOD;
  }

#if RTC_PERIOD != 0x8000
  sleep_cnt = tdelay / RTC_PERIOD + 1; // Calculate number of wrap arounds (overflows)
#else
  sleep_cnt = 2 * (tdelay >> 16) + ((tdelay & 0x8000) ? 2 : 1); // 10 times faster...
#endif
  RTC.INTFLAGS = RTC_CMP_bm; // Clear interrupt flag by writing '1'
  RTC.INTCTRL |= RTC_CMP_bm;
  interrupts();
  uint64_t start = millis();
  while (sleep_cnt) {
    sleep_cpu();
  }
#if !TEST_TIME
  noInterrupts();
  set_millis(start + n);
  interrupts();
#endif

  RTC.INTCTRL &= ~RTC_CMP_bm;
#endif
  ADC0.CTRLA |= ADC_ENABLE_bm;
}

#ifdef ANALOG_INPUT
float get_temp()
{
  analogReference(VDD);
  analogRead(ANALOG_INPUT);
  uint16_t v = analogRead(ANALOG_INPUT);
  float steinhart = v;
  steinhart = 1023 / steinhart - 1;
  steinhart = 10000 * steinhart;
  steinhart = steinhart / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 4050;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  return steinhart;
}
#endif

#if 1
unsigned int getBandgap ()
{
#if 0
  analogReference(INTERNAL1V1);
  analogRead(ADC_INTREF);
  uint16_t Vcc_value = 0x400 * 1100UL / analogRead(ADC_INTREF); /* calculate the Vcc value */
#else
  VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;    /* Set the Vref to 1.1V*/

  /* The following section is partially taken from Microchip App Note AN2447 page 13*/

  ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc    /* ADC internal reference, the Vbg*/;
  ADC0.CTRLC = ADC_PRESC_DIV4_gc        /* CLK_PER divided by 4 */
               | ADC_REFSEL_VDDREF_gc   /* Vdd (Vcc) be ADC reference */
               | 0 << ADC_SAMPCAP_bp;   /* Sample Capacitance Selection: disabled */

  uint16_t Vcc_value = 0;                /* measured Vcc value */

  ADC0.CTRLA = 1 << ADC_ENABLE_bp     /* ADC Enable: enabled */
               | 1 << ADC_FREERUN_bp  /* ADC Free run mode: enabled */
               | ADC_RESSEL_10BIT_gc; /* 10-bit mode */

  ADC0.COMMAND |= 1;                  // start running ADC
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  Vcc_value = 0x400 * 1100UL / ADC0.RES /* calculate the Vcc value */;
#endif
  return Vcc_value;
} // end of getBandgap
#endif

#if 1
unsigned int getBattery (unsigned voltage)
{
  analogReference(VDD);
  analogSampleDuration(31);
  analogRead(PIN_PA6);
  unsigned val = analogRead(PIN_PA6);

  unsigned int results = 2UL * voltage * val / 0x400;

  return results;
} // end of getBattery
#endif


void blinkN(uint8_t n, uint8_t l = led)
{
  for (uint8_t i = 0; i < n; i++) {
    digitalWriteFast(l, HIGH);
    sleepDelay(10);
    digitalWriteFast(l, LOW);
    sleepDelay(700);
  }
}


// the setup routine runs once when you press reset:
void setup() {
#if !USE_ESP
  Serial.swap(1); // A1,A2 = TX,RX
  Serial.begin(9600, SERIAL_TX_ONLY);
  Serial.println("Start"); Serial.flush();
#endif

  RTC_init();                           /* Initialize the RTC timer */
#ifdef SLEEPINT
#if TEST_TIME
  set_sleep_mode(SLEEP_MODE_IDLE);
#else
  set_sleep_mode(SLEEP_MODE_STANDBY);
#endif
#else
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#endif
  sleep_enable();                       /* Enable sleep mode, but not going to sleep yet */

  pinMode(PIN_PA1, INPUT_PULLUP);
  pinMode(PIN_PA2, INPUT_PULLUP);
  pinMode(PIN_PA3, INPUT_PULLUP);
  //  pinMode(PIN_PA4, INPUT_PULLUP);
  //  pinMode(PIN_PA5, INPUT_PULLUP);
  pinMode(PIN_PA6, INPUT); // _PULLUP);
  //pinMode(PIN_PA7, INPUT_PULLUP);
  //  pinMode(PIN_PB0, INPUT_PULLUP);
  //  pinMode(PIN_PB1, INPUT_PULLUP);
  //  pinMode(PIN_PB2, INPUT_PULLUP);
  //  pinMode(PIN_PB3, INPUT_PULLUP);

  pinMode(led, OUTPUT);

  for (byte i = 0; i < 3; i++) {
    blinkN(1, led);
  }

#if TEST_TIME
 while (1) {
    blinkN(1);
    uint16_t testn;
    switch (random() & 0x07) {
      case 0: testn = 9005; break;
      case 1: testn = 29016; break;
      case 2: testn = 20011; break;
      default:
          testn = random(500, 2500);
    }
    //Serial.println(testn); Serial.flush();
    uint32_t start = millis();
    sleepDelay(testn);
    // int rand = random(1,1000);
    // sleepDelay(rand);
    // Serial.flush();
    int32_t delta = millis() - start;
    Serial.print(delta); Serial.print(' '); Serial.println(testn); Serial.flush();
    if (delta < testn - 500 || delta > testn + 500) {
      Serial.println("Error"); Serial.flush();
      while (1) blinkN(1);
    }
  }
#endif

  sleepDelay(5000);

  // pinMode(PIN_PA2, OUTPUT);
  // Event0.set_generator(gen::rtc_ovf);
  // Event0.set_user(user::evouta_pin_pa2);
  // Event0.start();
  // pinMode(PIN_PA7, OUTPUT);
  // Logic1.enable = true;
  // Logic1.input0 = logic::in::tca0;
  // Logic1.input1 = logic::in::tca0;
  // Logic1.input2 = logic::in::tca0;
  // Logic1.output = logic::out::enable;
  // Logic1.filter = logic::filter::disable;
  // Logic1.truth = 0x55;
  // Logic1.init();
  // Logic1.start();
}


volatile unsigned voltage;
volatile unsigned temperature;

#if USE_ESP
volatile unsigned cnt;

// function that executes whenever data is requested by master
// this function is registered as an event
void requestEvent() {
  Wire.write(cnt >> 8);
  Wire.write(cnt & 0xFF);
  Wire.write(voltage >> 8);
  Wire.write(voltage & 0xFF);
#ifdef ANALOG_INPUT
  Wire.write(temperature >> 8);
  Wire.write(temperature & 0xFF);
#endif
  ++cnt;
}
#endif

// the loop routine runs over and over again forever:
void loop() {

  getBandgap();
  unsigned int bg = getBandgap();

  sleepDelay(3000);
#if !BANDGAP
  getBattery(bg);
  int bat = getBattery(bg);
  voltage = bat;
  int v = bat + 50;
#else
  int v = bg + 50;
#endif
  blinkN(v / 1000);
  sleepDelay(1500);
  blinkN((v % 1000) / 100);

#ifdef ANALOG_INPUT
  sleepDelay(3000);
  digitalWrite(led, HIGH);
  float temp = get_temp();
  // v = int(temp + 0.5);
  // blinkN(v / 10);
  // sleepDelay(1500);
  // blinkN(v % 10);
  pinMode(led, INPUT_PULLUP);

  temperature = 10 * temp;
#endif

#if USE_ESP
  if (bat > 3300 || bg > 3200) {
    digitalWrite(reset_ESP, LOW);
    pinMode(reset_ESP, OUTPUT);
    sleepDelay(32);
    pinMode(reset_ESP, INPUT_PULLUP);

    //sleepDelay(3000);

    auto startcnt = cnt;

    Wire.begin(1);                // join i2c bus with address #1
    Wire.onRequest(requestEvent); // register event

    auto start = millis();
    while (cnt == startcnt && millis() - start < 2000)
      ;
    delay(200);
    Wire.end();
  }
#endif

#if ANALOG_INPUT
  pinMode(led, OUTPUT);
#endif

  for (uint16_t s = 0; s < SLEEP_TIME; s += 5) {
    sleepDelay(5000);
    blinkN(1);
  }
}
