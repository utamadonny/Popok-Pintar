#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#define HTU21D_I2CADDR      0x40
#define HTU21D_READTEMP     0xE3
#define HTU21D_READHUM      0xE5
#define HTU21D_WRITEREG     0xE6
#define HTU21D_READREG      0xE7
#define HTU21D_RESET        0xFE
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <util/delay.h>
#define OLED_RESET 4

//pin
const int buzzer = 5;
const int ledm = 6; // merah BAK
const int ledk = 7; //kuning BAB
const int leds =8; // hijau ON


class HTU21D {
    public:
        HTU21D();
    boolean begin(void);
    
        void readSensor(void);
        float getHumidity(void);
        float getTemperature_C(void);
        float getTemperature_F(void);
    
 
    private:
        float readTemperature(void);
        float readHumidity(void);
        void reset(void);
        boolean readData(void);
    
        float humidity;
        float temperature_C;
        float temperature_F;
};

HTU21D::HTU21D() {
}
HTU21D htu;
Adafruit_SSD1306 display(OLED_RESET);
unsigned long previousMillis = 0;
unsigned long interval = 100;

// Your threshold value
int sensorThres = 400;

const int MQ_PIN=A0;                                //define which analog input channel you are going to use
int RL_VALUE=20;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=2.5;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                    //which is derived from the chart in datasheet

int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in 
                                                    //normal operation
#define         GAS_H2S             0    
 
/*****************************Globals***********************************************/
float           H2SCurve[3]  =  {1,0.17,-0.305};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)                                                    
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
pinMode(buzzer, OUTPUT);
pinMode(ledm, OUTPUT);
pinMode(ledk, OUTPUT);
pinMode(leds, OUTPUT);
}
  

void loop() {

int analogSensor = analogRead(MQ_PIN);
Serial.print(analogSensor);Serial.print("    ");

long iPPM_H2S = 0;

iPPM_H2S = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2S);  
long IppMcalibrate = iPPM_H2S;

  // put your main code here, to run repeatedly:
unsigned long currentMillis = millis();
htu.readSensor();
if (isnan(htu.getHumidity()) || isnan(htu.getTemperature_C())) {
Serial.println("HTU21D sensor read failure!");
return;
 }
Serial.print(htu.getHumidity()); Serial.print(" %\t");
Serial.print(htu.getTemperature_C()); Serial.print(" *C\t\t");
Serial.print(htu.getTemperature_F()); Serial.print(" *F\t\t");
Serial.print(analogSensor); Serial.print(" X");
Serial.print(IppMcalibrate); Serial.println(" ppm");


if (currentMillis - previousMillis >= interval) {           // OLED
    previousMillis = currentMillis;
display.clearDisplay();
display.setTextColor(WHITE);
display.setTextSize(1);
display.setCursor(0, 0);
display.println(htu.getHumidity());
display.setCursor(35, 0);
display.println("%");
display.setCursor(50, 0);
display.println(htu.getTemperature_C());
display.setCursor(95, 0);
display.println("C");
display.setCursor(0, 10);
display.println(analogSensor);
display.setCursor(65, 10);
display.println("X");
display.setCursor(0, 20);
display.println(IppMcalibrate);
display.setCursor(65, 20);
display.println("ppm");
display.display();
}

if (htu.getHumidity()>= 70 && htu.getTemperature_C()>= 33 && IppMcalibrate>= 1){ //          pengondisian
  digitalWrite(ledk, HIGH);
  digitalWrite(buzzer, HIGH);
  digitalWrite(leds, LOW);
  digitalWrite(ledm, LOW);
}

if (htu.getHumidity()>= 70 && htu.getTemperature_C()>= 33){
  digitalWrite(ledm, HIGH);
  digitalWrite(ledk, LOW);
  digitalWrite(buzzer, HIGH);
  digitalWrite(leds, LOW);
  
}

digitalWrite(leds, HIGH);

}


float MQResistanceCalculation(int raw_adc){
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
float MQCalibration(int mq_pin){
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

}
float MQRead(int mq_pin){
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
long MQGetGasPercentage(float rs_ro_ratio, int gas_id){
  if ( gas_id == GAS_H2S ) {
     return MQGetPercentage(rs_ro_ratio,H2SCurve);
  } 
 
  return 0;
}
long  MQGetPercentage(float rs_ro_ratio, float *pcurve){
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


boolean HTU21D::begin(void) {
  Wire.begin();
  
  reset();

  Wire.beginTransmission(HTU21D_I2CADDR);
  Wire.write(HTU21D_READREG);
  Wire.endTransmission();
  Wire.requestFrom(HTU21D_I2CADDR, 1);
  return (Wire.read() == 0x2); // after reset should be 0x2
}
void HTU21D::readSensor(void) {
    humidity = (readHumidity()-readHumidity()*0.03442)+(readHumidity()-readHumidity()*0.03442)*0.0614; 
    temperature_C = readTemperature();
    temperature_F = temperature_C * 1.8 + 32;
}
float HTU21D::getHumidity(void) {
    return humidity;
}
float HTU21D::getTemperature_C(void) {
    return temperature_C;
}
float HTU21D::getTemperature_F(void) {
    return temperature_F;
}
void HTU21D::reset(void) {
    Wire.beginTransmission(HTU21D_I2CADDR);
    Wire.write(HTU21D_RESET);
    Wire.endTransmission();
    delay(15);
}
float HTU21D::readTemperature(void) {
  
  Wire.beginTransmission(HTU21D_I2CADDR);
  Wire.write(HTU21D_READTEMP);
  Wire.endTransmission();
  
  delay(50); // add delay between request and actual read!
  
  Wire.requestFrom(HTU21D_I2CADDR, 3);
  while (!Wire.available()) {}

  uint16_t t = Wire.read();
  t <<= 8;
  t |= Wire.read();

  uint8_t crc = Wire.read();

  float temp = t;
  temp *= 175.72;
  temp /= 65536;
  temp -= 46.85;

  return temp;
}
float HTU21D::readHumidity(void) {
  // OK lets ready!
  Wire.beginTransmission(HTU21D_I2CADDR);
  Wire.write(HTU21D_READHUM);
  Wire.endTransmission();
  
  delay(50); // add delay between request and actual read!
  
  Wire.requestFrom(HTU21D_I2CADDR, 3);
  while (!Wire.available()) {}

  uint16_t h = Wire.read();
  h <<= 8;
  h |= Wire.read();

  uint8_t crc = Wire.read();

  float hum;
  hum = h;
  hum *= 125;
  hum /= 65536;
  hum -= 6;

  return hum;
}
