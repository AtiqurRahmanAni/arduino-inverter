#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define ZEROCROSS_PIN PD2  //pin for the external interrupt (Input with Pullup)
#define INV_PIN PD4        //pin to trigger the INV_PIN (Output)
#define OCR_VALUE_INC 20
#define OCR1A_VAL 20
#define INVERTER_OFF_DELAY 4  // time in seconds to turn off inverter after power restore
#define INVERTER_ON_THR 2.6
#define INVERTER_OFF_THR 3.0
#define AC_VOLT_FACTOR 61.63
#define DC_VOLT_FACTOR 16.83
#define LOW_BATT_THR 10.00

bool isInverterOn = false;
volatile float peakVoltage = 0;
volatile float maxVoltage = 0;
volatile unsigned int invOffsecCounter = 0;
int lowBatsecCounter = 0;
bool isLowBattery = false;
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
  TCCR1A = 0;                               //clear timer1 register (TCCR1A)
  TCCR1B = 0;                               //clear timer1 register (TCCR1B)
  TCCR1B |= (1 << CS12) | (1 << CS10);      //set prescaler of timer1 to 1024
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);  //activate timer compare match interrupt for OCR0A and OCR0B
  OCR1A = 0;
  OCR1B = 157;
  TCNT1 = 0;
  attachInterrupt(digitalPinToInterrupt(2), zeroCross, CHANGE);
  sei();

  delay(100);
}

void zeroCross() {
  if (PIND & _BV(2)) {
    // rising edge
    TCNT1 = 0;
    OCR1A = OCR1A_VAL;
  } else {
    // falling edge
    OCR1B = (TCNT1 + 157) % 0xFFFF;  // interrupt after 10ms to confirm power outage
    peakVoltage = maxVoltage;
  }

  if (peakVoltage <= INVERTER_ON_THR) {
    invOffsecCounter = 0;
  }

  if (!isInverterOn && !isLowBattery && peakVoltage <= INVERTER_ON_THR) {
    turnOnInverter();
  }

  maxVoltage = 0;
}

ISR(TIMER1_COMPA_vect) {
  OCR1A = OCR1A + OCR_VALUE_INC < 0xFFFF ? OCR1A + OCR_VALUE_INC : OCR1A_VAL;
  int x = analogRead(A2);
  x = x < 30 ? 0 : x;  // Filter out noise
  maxVoltage = max(maxVoltage, (x / 1024.0) * 5.0);
}

ISR(TIMER1_COMPB_vect) {
  if (!(PIND & _BV(2))) {
    if (!isInverterOn && !isLowBattery) turnOnInverter();
    peakVoltage = 0;
    maxVoltage = 0;
  }
}

void turnOnInverter() {
  PORTD |= _BV(INV_PIN);
  isInverterOn = true;
  invOffsecCounter = 0;
}

void turnOffInterter() {
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


void loop() {

  if (isInverterOn && peakVoltage >= INVERTER_OFF_THR) {
    invOffsecCounter++;
  }
  if (invOffsecCounter >= INVERTER_OFF_DELAY && peakVoltage >= INVERTER_OFF_THR) {
    turnOffInterter();
  }

  float batteryVoltage = readBatteryVolt();

  if (isInverterOn && batteryVoltage <= LOW_BATT_THR) {
    lowBatsecCounter++;
  } else {
    lowBatsecCounter = 0;
  }
  if (isInverterOn && lowBatsecCounter >= INVERTER_OFF_DELAY) {
    turnOffInterter();
    isLowBattery = true;
  }
  if(batteryVoltage >= 11.00) {
    isLowBattery = false;
  }

  lcd.setCursor(0, 0);
  sprintf(cstr, "AC:%3dV", (int)(peakVoltage * AC_VOLT_FACTOR));
  lcd.print(cstr);

  lcd.print(isInverterOn ? "---INV ON " : "--INV OFF");

  lcd.setCursor(2, 1);
  int whole = (int)batteryVoltage;
  int fractional = (int)((batteryVoltage - whole) * 100);
  sprintf(cstr, "VBat:%d.%02dV", whole, fractional);
  lcd.print(cstr);

  // lcd.clear();
  // lcd.print(peakVoltage);

  delay(1000);
}
