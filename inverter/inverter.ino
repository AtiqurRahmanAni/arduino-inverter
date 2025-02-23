#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define ZEROCROSS_PIN PD2  // pin for the external interrupt (Input with Pullup)
#define INV_PIN PD4        // pin to trigger the INV_PIN (Output)
#define OCR_VALUE_INC 16
#define OCR1A_VAL 16
#define INVERTER_OFF_DELAY 4  // time in seconds to turn off inverter after power restore
#define INVERTER_ON_THR 1.6
#define INVERTER_OFF_THR 2.6
#define AC_VOLT_FACTOR 76.28
#define DC_VOLT_FACTOR 16.83
#define LOW_BATT_THR 10.00
#define CHARGE_OFF_THR 14.20
#define CHARGE_OFF_DELAY 4

bool isInverterOn = false;
volatile float peakVoltage = 0;
volatile float maxVoltage = 0;
volatile unsigned int invOffSecCounter = 0;
unsigned int chargeOnSecCounter = 0;
int lowBatSecCounter = 0;
bool isLowBattery = false;
bool isCharging = false;
bool chargeOnceAfterMainsFail = false;
char cstr[20];

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.home();
  lcd.clear();

  DDRD |= _BV(INV_PIN);
  DDRD &= ~_BV(ZEROCROSS_PIN);
  PORTD &= ~_BV(INV_PIN);

  cli();
  TCCR1A = 0;                               // clear timer1 register (TCCR1A)
  TCCR1B = 0;                               // clear timer1 register (TCCR1B)
  TCCR1B |= (1 << CS12) | (1 << CS10);      // set prescaler of timer1 to 1024
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);  // activate timer compare match interrupt for OCR0A and OCR0B
  OCR1A = 0;
  OCR1B = 0;
  TCNT1 = 0;
  attachInterrupt(digitalPinToInterrupt(2), zeroCross, CHANGE);
  sei();

  delay(100);
}

void zeroCross() {
  cli();
  TCNT1 = 0;
  sei();
  // rising edge
  if (PIND & _BV(2)) {
    OCR1A = OCR1A_VAL;  // interrupt every 1ms to read AC voltage
  } else {
    // falling edge
    OCR1B = TCNT1 + 79;  // interrupt after 5ms to confirm power outage
    peakVoltage = maxVoltage;
  }

  if (peakVoltage <= INVERTER_ON_THR) {
    invOffSecCounter = 0;
  }

  if (!isInverterOn && !isLowBattery && peakVoltage <= INVERTER_ON_THR) {
    turnOnInverter();
  }

  maxVoltage = 0;
}

ISR(TIMER1_COMPA_vect) {
  int x = analogRead(A2);
  x = x <= 20 ? 0 : x;  // Filter out noise
  maxVoltage = max(maxVoltage, (x / 1024.0) * 5.0);
  OCR1A = TCNT1 + OCR1A_VAL;
}

ISR(TIMER1_COMPB_vect) {
  if (!(PIND & _BV(2))) {

    if (!isInverterOn && !isLowBattery) {
      turnOnInverter();
    }

    peakVoltage = 0;
    maxVoltage = 0;
  }
}

void turnOnInverter() {
  chargeOff();
  PORTD |= _BV(INV_PIN);
  isInverterOn = true;
  invOffSecCounter = 0;
  chargeOnceAfterMainsFail = false;
}

void turnOffInverter() {
  PORTD &= ~_BV(INV_PIN);
  isInverterOn = false;
}

float readBatteryVolt() {
  float vbat = 0.0;
  for (byte i = 0; i < 5; i++) {
    vbat += (analogRead(A3) / 1024.0) * DC_VOLT_FACTOR;
  }
  return vbat / 5;
}

void chargeOn() {
  // set charge pin high
  isCharging = true;
  chargeOnceAfterMainsFail = true;
}

void chargeOff() {
  // set charge pin low
  isCharging = false;
  chargeOnSecCounter = 0;
}

void loop() {

  if (isInverterOn && peakVoltage >= INVERTER_OFF_THR) {
    invOffSecCounter++;
  }
  if (invOffSecCounter >= INVERTER_OFF_DELAY && peakVoltage >= INVERTER_OFF_THR) {
    turnOffInverter();
  }

  float batteryVoltage = readBatteryVolt();

  if (isInverterOn && batteryVoltage <= LOW_BATT_THR) {
    lowBatSecCounter++;
  } else {
    lowBatSecCounter = 0;
  }
  if (isInverterOn && lowBatSecCounter >= INVERTER_OFF_DELAY) {
    turnOffInverter();
    isLowBattery = true;
  }
  if (batteryVoltage >= 11.00) {
    isLowBattery = false;
  }

  if (!isCharging && !chargeOnceAfterMainsFail && !isInverterOn) {
    chargeOnSecCounter++;
  }
  if (!isCharging && !chargeOnceAfterMainsFail && chargeOnSecCounter >= CHARGE_OFF_DELAY) {
    chargeOn();
  }
  if (isCharging && batteryVoltage >= CHARGE_OFF_THR) {
    chargeOff();
  }

  lcd.setCursor(0, 0);
  sprintf(cstr, "AC:%3dV", (int)(peakVoltage * AC_VOLT_FACTOR));
  lcd.print(cstr);

  lcd.print(isInverterOn ? "---INV ON " : "--INV OFF");

  lcd.setCursor(0, 1);
  int whole = (int)batteryVoltage;
  int fractional = (int)((batteryVoltage - whole) * 100);
  sprintf(cstr, "VB:%02d.%02dV", whole, fractional);
  lcd.print(cstr);

  lcd.setCursor(12, 1);
  if (isCharging) {
    lcd.print("CHRG");
  } else {
    lcd.setCursor(12, 1);
    lcd.print("FULL");
  }

  // lcd.clear();
  // lcd.print(peakVoltage);

  delay(1000);
}
