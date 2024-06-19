#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>

constexpr int ledPin=12;

volatile uint64_t prevTime=0; //us
volatile uint64_t delta=0; //us
volatile double speed=15.2; //m/s
constexpr double LED_DISTANCE = 0.068169; // m
constexpr double MICROSECONDS_TO_SECONDS = 1e-6;
volatile uint64_t debounceTime = 200; // Debounce time in microseconds
double getSpeed(uint64_t dt);
double getSpeed(uint64_t dt) {  
  Serial.println("SPEED CALCULATION STARTED");
  if(dt == 0)
    {
      return -1.0000002;
      }
    
  Serial.println("c1");
  Serial.println(LED_DISTANCE,10);
  Serial.println("c2");
  Serial.println(static_cast<double>(dt),10);
  Serial.println("c3");
  Serial.println((LED_DISTANCE / static_cast<double>(dt)),10);
  Serial.println("c4");
  Serial.println((LED_DISTANCE / static_cast<double>(dt)) / MICROSECONDS_TO_SECONDS,10);


  return (LED_DISTANCE / static_cast<double>(dt)) / MICROSECONDS_TO_SECONDS;
  // return (LED_DISTANCE / static_cast<double>(dt)) * MICROSECONDS_TO_SECONDS;
}
IRAM_ATTR   void ledInterrupt(){
   static uint64_t lastInterruptTime = 0;
  uint64_t currentTime = micros64();

// If interrupts come faster than the debounce time, ignore them
if (currentTime - lastInterruptTime > debounceTime) {
  delta = (prevTime > currentTime) ? (prevTime - currentTime) : (currentTime - prevTime);
  prevTime = currentTime;
}

lastInterruptTime = currentTime;
}
void setUpInterrupts(){
  attachInterrupt(digitalPinToInterrupt(ledPin), ledInterrupt, FALLING);
}
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}


void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}


double calculateKineticEnergy(double massInGrams, double velocityInMetersPerSecond) {
  double massInKilograms = massInGrams / 1000.0;
  return 0.5 * massInKilograms * pow(velocityInMetersPerSecond, 2);
}

void setup() {

  Serial.begin(9600);
  delay(2000);
  Serial.println("Attaching interrupts");
  setUpInterrupts();
   WiFi.mode(WIFI_OFF);
   if (WiFi.getMode() == WIFI_OFF)
      Serial.println(F("\nWifi mode is WIFI_OFF, until it is explicitly changed"));
  Wire.begin(2,14);
   byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C,true,false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(100); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

}

void loop() {
  Serial.println("Loop started");
  speed = getSpeed(delta);
  display.clearDisplay();
  display.setTextSize(1);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  char buf2[80];
  snprintf(buf2, 80, "Speed: %3.2f m/s\r\nDelta: %llu us\r\n0.2g %3.2f \r\n0.3g %3.2f",speed, delta,calculateKineticEnergy(0.2, speed),calculateKineticEnergy(0.3, speed));
  Serial.println(speed,10);
  Serial.println(delta);
  display.println(buf2);
    display.display();
  delay(1000);
}
