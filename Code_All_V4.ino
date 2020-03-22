/*
 * Arduino with LM35 analog temperature sensor and SSD1306 OLED display.
 * This is a free software with NO WARRANTY.
 * https://simple-circuit.com/
 */
 
#include <Wire.h>              // include Arduino wire library (required for I2C devices)
#include <Adafruit_GFX.h>      // include Adafruit graphics library
#include <Adafruit_SSD1306.h>  // include Adafruit SSD1306 OLED display driver
#define OLED_Address 0x3C // 0x3C device address of I2C OLED. Few other OLED has 0x3D

Adafruit_SSD1306 display(128, 64);  // Create display

String message = "";
bool messageReady = false;
 
int a=0;
int lasta=0;
int lastb=0;
int LastTime=0;
int ThisTime;
bool BPMTiming=false;
bool BeatComplete=false;
int BPM=0;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

int tCelsius;
char _buffer[8];

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
//int data[STORE_SIZE][5]; //array for saving past data
//byte currentIndex=0; //stores current data array index (0-255)
boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred

byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;
const int buttonPin = 3;
const int powerPin = 12;
boolean powerState = true;
char temperature[3] = "";

#define UpperThreshold 800
#define LowerThreshold 500
#define  LM35_pin  A2
 
void setup(void)
{

     // initialize the pushbutton pin as an input:
 pinMode(buttonPin, INPUT_PULLUP);

 pinMode(powerPin, INPUT);

 pinMode(11, OUTPUT); //output pin for LED 
 // digitalWrite(11, HIGH);

  if (digitalRead(powerPin) == HIGH) {
  Serial1.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_Address);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);  // set text color to white and black background
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  }
}
 
void loop()
{    

  if(a>127 || fall == true || digitalRead(powerPin) == LOW) {
      display.clearDisplay();
      a=0;
      lasta=a;
      dtostrf(tCelsius/10.0,3,1,temperature);
      String output = String(temperature) + " " + String(BPM) + " " + String(fall);
      Serial1.print(output);
}

  if (digitalRead(powerPin) == HIGH) {
    
      if (powerState == false && fall == false) {
  display.clearDisplay();
    display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);  // set text color to white and black background
display.writeFillRect(0,50,128,16,BLACK);
  display.display();
  a=0;
lasta=a;
  powerState = true;
}

display.setCursor(20,0);
display.print("HEALTH MONITOR");

ThisTime=millis();
 analogReference(VDD);
// analogRead(0);
// delay(10);
 int value=analogRead(0);
int b=50-(value/25);
display.writeLine(lasta,lastb,a,b,WHITE);
lastb=b;
lasta=a;

if(value>UpperThreshold)
{
if(BeatComplete)
{
BPM=ThisTime-LastTime;
BPM=int(60/(float(BPM)/1000));
BPMTiming=false;
BeatComplete=false;
tone(8,1000,250);
}
if(BPMTiming==false)
{
LastTime=millis();
BPMTiming=true;
}
}
if((value<LowerThreshold)&(BPMTiming))
BeatComplete=true;

if (fall == false) {
    //display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);  // set text color to white and black background
display.writeFillRect(0,50,128,16,BLACK);
display.setCursor(0,50);
display.print("BPM:");
display.print(BPM);

  display.setCursor(50, 50);            // move cursor to position (15, 0) pixel
  display.print("Temp:");

  //  display.setCursor(20,0);
 // display.print("HEALTH MONITOR");

  analogReference(INTERNAL1V1);  // set positive reference voltage to 1.1V
//  analogRead(LM35_pin);
//  delay(5);
  tCelsius = 10 * analogRead(LM35_pin) / 9.3;
 
  // print temperature in degree Celsius
  if (tCelsius >= 1000)    // if temperature >= 100.0 °C
    sprintf(_buffer, "%03u.%1u C", tCelsius / 10, tCelsius % 10);
  else
    sprintf(_buffer, " %02u.%1u C", tCelsius / 10, tCelsius % 10);
  //display.setCursor(23, 10);
  display.print(_buffer);
 
//   print degree symbols ( ° )
  display.drawCircle(112, 50, 1, WHITE);
 
  // update the display
  display.display();
  a++;
}

  mpu_read();
 //2050, 77, 1947 are values for calibration of accelerometer
 // values may be different for you
 ax = (AcX-2050)/16384.00;
 ay = (AcY-77)/16384.00;
 az = (AcZ-1947)/16384.00;

 //270, 351, 136 for gyroscope
 gx = (GyX+270)/131.07;
 gy = (GyY-351)/131.07;
 gz = (GyZ+136)/131.07;

 // calculating Amplitute vactor for 3 axis
 float Raw_AM = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
 int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied 
                        // it by for using if else conditions 

 //delay(500);
 

 if (trigger3==true){
    trigger3count++;
    if (trigger3count>=10){ 
       angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
       //delay(10);
       if ((angleChange>=0) && (angleChange<=10)){ //if orientation changes remains between 0-10 degrees
           fall=true; trigger3=false; trigger3count=0;
             }
       else{ //user regained normal orientation
          trigger3=false; trigger3count=0;
       }
     }
  }
 if (fall==true){ //in event of a fall detection
  powerState = false;
   //buttonState = digitalRead(buttonPin);
   display.clearDisplay();
     display.setTextSize(2);
   display.setCursor(35,10);
display.print("Fall");
   display.setCursor(10,30);
display.print("Detected!");
display.display();
   digitalWrite(11, HIGH);
   delay(1000);
display.clearDisplay();
display.display();
digitalWrite(11, LOW);
   delay(1000);
   if (digitalRead(buttonPin) == LOW){ //LOW
    // digitalWrite(buttonPin, LOW);
  
    fall=false;
    boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
    boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
    boolean trigger3=false; //stores if third trigger (orientation change) has occurred
   }
  // exit(1);
   }
 if (trigger2count>=6){ //allow 0.5s for orientation change
   trigger2=false; trigger2count=0;
   }
 if (trigger1count>=6){ //allow 0.5s for AM to break upper threshold
   trigger1=false; trigger1count=0;
   }
 if (trigger2==true){
   trigger2count++;
   //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
   angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
   if (angleChange>=30 && angleChange<=400){ //if orientation changes by between 80-100 degrees
     trigger3=true; trigger2=false; trigger2count=0;
       }
   }
 if (trigger1==true){
   trigger1count++;
   if (AM>=12){ //if AM breaks upper threshold (3g)
     trigger2=true;
     trigger1=false; trigger1count=0;
     }
   }
 if (AM<=2 && trigger2==false){ //if AM breaks lower threshold (0.4g)
   trigger1=true;
   }
//It appears that delay is needed in order not to clog the port
 //delay(100);
}

if (digitalRead(powerPin) == LOW) {
     display.clearDisplay();
     display.setTextSize(2);
   display.setCursor(25,25);
display.print("PAUSED");
display.display();
powerState = false;
}

}

void mpu_read(){
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
