/* Icon made by Pixel perfect from www.flaticon.com */
#include <Arduino_GFX_Library.h>
Arduino_DataBus *bus = new Arduino_HWSPI(3 /* DC */, -1 /* CS */);
Arduino_ST7789 *gfx = new Arduino_ST7789(bus, 2 /* RST */, 3 /* rotation */, true /* IPS */, 135 /* width */, 240 /* height */, 53 /* col offset 1 */, 40 /* row offset 1 */, 52 /* col offset 2 */, 40 /* row offset 2 */);

#include <Wire.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
// uncomment below line if cannot calculate readings
#define REVERSE_LED

// Icon
#include "heartrate.h"
#include "oxygen.h"

// Interrupt pin
const byte oxiInt = 0; // pin connected to MAX30102 INT

uint32_t elapsedTime, timeStart;

uint32_t aun_ir, aun_red;
uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy;

void setup()
{
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

  Wire.begin();

  Serial.begin(115200);
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->draw16bitRGBBitmap(0, 7, (uint16_t*)heartrate.pixel_data, heartrate.width, heartrate.height);
  gfx->draw16bitRGBBitmap(0, 71, (uint16_t*)oxygen.pixel_data, oxygen.width, oxygen.height);

  Serial.println("Initializing");

  gfx->setTextColor(WHITE, BLACK);
  gfx->setTextSize(2 /* x scale */, 2 /* y scale */);
  gfx->setCursor(72, 0);
  gfx->print("Initializing");

  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);

  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy); //Reads/clears the interrupt status register
  maxim_max30102_init();  //initialize the MAX30102
  old_n_spo2 = 0.0;

  Serial.println(F("Time[s]\tSpO2\tHR\tRatio\tCorr"));

  timeStart = millis();
}

void loop()
{
  float n_spo2, ratio, correl; //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heartrate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;

  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    while (digitalRead(oxiInt) == 1); //wait until the interrupt pin asserts
#ifdef REVERSE_LED
    maxim_max30102_read_fifo(&aun_ir, &aun_red); //read from MAX30102 FIFO
#else
    maxim_max30102_read_fifo(&aun_red, &aun_ir); //read from MAX30102 FIFO
#endif
    if (aun_ir < 5000)
    {
      break;
    }
    if (i == 0)
    {
      gfx->setTextColor(WHITE, BLACK);
      gfx->setTextSize(2 /* x scale */, 2 /* y scale */);
      gfx->setCursor(72, 0);
      gfx->print("Measuring... ");
    }
    *(aun_ir_buffer + i) = aun_ir;
    *(aun_red_buffer + i) = aun_red;
  }

  if (aun_ir < 5000)
  {
    gfx->setTextColor(WHITE, BLACK);
    gfx->setTextSize(2 /* x scale */, 2 /* y scale */);
    gfx->setCursor(72, 0);
    gfx->print("Put On Finger");
  }
  else
  {
    //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heartrate, &ch_hr_valid, &ratio, &correl);
    elapsedTime = millis() - timeStart;
    elapsedTime /= 1000; // Time in seconds

    if (ch_hr_valid && ch_spo2_valid) {
      Serial.print(elapsedTime);
      Serial.print("\t");
      Serial.print(n_spo2);
      Serial.print("\t");
      Serial.print(n_heartrate, DEC);
      Serial.print("\t");
      Serial.print(ratio);
      Serial.print("\t");
      Serial.print(correl);
      Serial.println("");

      gfx->setTextSize(7 /* x scale */, 7 /* y scale */, 2 /* pixel_margin */);
      gfx->setTextColor(GREEN, BLACK);
      gfx->setCursor(72, 20);
      gfx->printf("%d ", n_heartrate);
      gfx->setTextColor(ORANGE, BLACK);
      gfx->setCursor(72, 84);
      gfx->printf("%.1f", n_spo2);

      old_n_spo2 = n_spo2;
    }
  }
}
