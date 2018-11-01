#include <dht.h>
#include <PMsensor.h>
#include <SoftwareSerial.h>
#include "DustSensor.h"
#include "Wire.h"
#include "Adafruit_SI1145.h"
#include <Adafruit_MPL3115A2.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}
#include "MQ135.h"


#define TCAADDR 0x70
/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (1)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;    //Ro is initialized to 10 kilo ohms
#define DHT11_PIN 2
#define BLUETOOTH_SPEED 38400
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_SI1145 uv = Adafruit_SI1145();
MQ135 gasSensor = MQ135(A0);
dht DHT;
SoftwareSerial mySerial(9, 10); // RX, TX
PMsensor PM;
int MPL;
int SI;

int sound =A2;
int count=0;
float LPG,CO,SMOKE,CO2,SOUND,BACKTEMP,DUST,PRESSURE,FRONTTEMP,HUMIDITY,VIS,IR,UV;
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

 void printJSON(float LPG,float CO, float SMOKE,float CO2,float SOUND,float BACKTEMP,float HUMIDITY,float DUST,float PRESSURE,float FRONTTEMP,float VIS,float IR,float UV){
  Serial.print("{\"LPG\":");
  Serial.print(LPG);
  Serial.print("\"CO\":");
  Serial.print(CO);
  Serial.print("\"SMOKE\":");
  Serial.print(SMOKE);
  Serial.print("\"CO2\":");
  Serial.print(CO2);
  Serial.print("\"SOUND\":");
  Serial.print(SOUND);
  Serial.print("\"BACKTEMP\":");
  Serial.print(BACKTEMP);
  Serial.print("\"HUMIDITY\":");
  Serial.print(HUMIDITY);
  Serial.print("\"DUST\":");
  Serial.print(DUST);
  Serial.print("\"PRESSURE\":");
  Serial.print(PRESSURE);
  Serial.print("\"FRONTTEMP\":");
  Serial.print(FRONTTEMP);
  Serial.print("\"VIS\":");
  Serial.print(VIS);
  Serial.print("\"IR\":");
  Serial.print(IR);
  Serial.print("\"UV\":");
  Serial.print(UV);
  Serial.println("}");
  
  mySerial.print("{\"LPG\":");
  mySerial.print(LPG);
  mySerial.print("\"CO\":");
  mySerial.print(CO);
  mySerial.print("\"SMOKE\":");
  mySerial.print(SMOKE);
  mySerial.print("\"CO2\":");
  mySerial.print(CO2);
  mySerial.print("\"SOUND\":");
  mySerial.print(SOUND);
  mySerial.print("\"BACKTEMP\":");
  mySerial.print(BACKTEMP);
  mySerial.print("\"HUMIDITY\":");
  mySerial.print(HUMIDITY);
  mySerial.print("\"DUST\":");
  mySerial.print(DUST);
  mySerial.print("\"PRESSURE\":");
  mySerial.print(PRESSURE);
  mySerial.print("\"FRONTTEMP\":");
  mySerial.print(FRONTTEMP);
  mySerial.print("\"VIS\":");
  mySerial.print(VIS);
  mySerial.print("\"IR\":");
  mySerial.print(IR);
  mySerial.print("\"UV\":");
  mySerial.print(UV);
  mySerial.println("}");
 }

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}


float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 

  return val; 
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs;  
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    

  return 0;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
void setup(){
  Serial.begin(9600);
  mySerial.begin(BLUETOOTH_SPEED);
  PM.init(3, A3);  
  Wire.begin();
  Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
           
            if (addr == 0x60) {
               if (uv.begin()) {
                  Serial.println("  --> Found Si1145 Light Sensor!!!");
                  SI=t;
                  delay(1000);
                }
                else if (baro.begin()) {
                  Serial.println("--> Found MPL !!!");
                  MPL=t;
                  }
            }
      }
    }}
     Ro = MQCalibration(MQ_PIN);
     Serial.println(gasSensor.getRZero());           
     Serial.println("\ndone");
}

void loop()
{
   float lpg=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
   LPG=LPG+lpg;
   float co=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
   CO=CO+co;
   float smoke=MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) ;
   SMOKE=SMOKE+smoke;
   float co2=gasSensor.getPPM();
   CO2=CO2+co2;
   float sound = analogRead(sound);
   SOUND=SOUND+sound;
   int chk = DHT.read11(DHT11_PIN);
   float backtemp=DHT.temperature;
   BACKTEMP=BACKTEMP+backtemp;
   float humidity=DHT.humidity;
   HUMIDITY=HUMIDITY+humidity;

  float data = 0;
  int err = PMsensorErrSuccess;
  if ((err = PM.read(&data, true, 0.1)) != PMsensorErrSuccess) {
    Serial.print("data Error = ");
    Serial.println(err);
    delay(3000);
  }
  float dust=data;
  DUST=DUST+dust;
  tcaselect(MPL);
  
  float pressure = baro.getPressure();
  PRESSURE=PRESSURE+pressure;
  float fronttemp = baro.getTemperature();
  FRONTTEMP=FRONTTEMP+fronttemp;
  tcaselect(SI);
  float vis=uv.readVisible();
  VIS=VIS+vis;
  float ir=uv.readIR();
  IR=IR+ir;
  float uvi = uv.readUV();
  UV=UV+uvi/100.0;
  count++;
  if(count==5)
  {
    printJSON(LPG/5.0,CO/5.0,SMOKE/5.0,CO2/5.0,SOUND/5.0,BACKTEMP/5.0,HUMIDITY/5.0,DUST/5.0,PRESSURE/5.0,FRONTTEMP/5.0,VIS/5.0,IR/5.0,UV/5.0);
    count=0;
    LPG=0;CO=0;SMOKE=0;CO2=0;SOUND=0;BACKTEMP=0;HUMIDITY=0;DUST=0;PRESSURE=0;FRONTTEMP=0;VIS=0;IR=0;UV=0;
  }
          
  delay(2000);

}

