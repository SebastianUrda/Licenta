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
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_SI1145 uv = Adafruit_SI1145();
int MPL;
int SI;
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
#define RZERO 553.2
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
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
dht DHT;
MQ135 gasSensor = MQ135(A0);
SoftwareSerial mySerial(9, 10); // RX, TX
#define DHT11_PIN 2
PMsensor PM;
 
//DustSensor dust = DustSensor(A3, 3);
int smokeA0 = A0;
int smokeA1 = A1;
int sound =A2;

#define BLUETOOTH_SPEED 38400
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
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
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
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

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
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

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
void setup(){
  Serial.begin(9600);
  pinMode(smokeA0, INPUT);
  pinMode(smokeA1, INPUT);
  mySerial.begin(BLUETOOTH_SPEED);
  mySerial.println("MERE!");

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
                  Serial.print("Vis: "); Serial.println(uv.readVisible());
                  Serial.print("IR: "); Serial.println(uv.readIR());
                   float UVindex = uv.readUV();
                   // the index is multiplied by 100 so to get the
                    // integer index, divide by 100!
                    UVindex /= 100.0;  
                    Serial.print("UV: ");  Serial.println(UVindex);
                }
                else if (baro.begin()) {
                  Serial.println("--> Found MPL !!!");
                  MPL=t;
                  float pascals = baro.getPressure();
                  // Our weather page presents pressure in Inches (Hg)
                   // Use http://www.onlineconversion.com/pressure.htm for other units
                   Serial.print(pascals); Serial.println(" Pascals ");
                   float altm = baro.getAltitude();
                   Serial.print(altm); Serial.println(" meters");
                   float tempC = baro.getTemperature();
                   Serial.print(tempC); Serial.println("*C");

                  }
            }
           
        
      }
    }}
    
     Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration                    
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  float rzero = gasSensor.getRZero();
  Serial.print(rzero);
Serial.println("\ndone");
}

void loop()
{
  int chk = DHT.read11(DHT11_PIN);
  //create some variables to store the color data in
  int smoke1 = analogRead(smokeA0);
  Serial.print("Smoke1: ");
  Serial.println(smoke1);
  mySerial.print("Smoke1: ");
  mySerial.println(smoke1);
   Serial.print("LPG:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("CO:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   Serial.print("SMOKE:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
   Serial.print( "ppm" );
   Serial.print("    ");  
   Serial.print("CO2:"); 
   Serial.print(gasSensor.getPPM());
   Serial.print( "ppm" );
   Serial.print("\n");
   mySerial.print("LPG:"); 
   mySerial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
   mySerial.print( "ppm" );
   mySerial.print("    ");   
   mySerial.print("CO:"); 
   mySerial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
   mySerial.print( "ppm" );
   mySerial.print("    ");   
   mySerial.print("SMOKE:"); 
   mySerial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
   mySerial.print( "ppm" );
   mySerial.print("    ");  
   mySerial.print("CO2:"); 
   mySerial.print(gasSensor.getPPM());
   mySerial.print( "ppm" );
   mySerial.print("\n");
   
   mySerial.print("\n");
  int soundIn = analogRead(sound);
  Serial.print("Sound: ");
  Serial.println(soundIn);
  mySerial.print("Sound: ");
  mySerial.println(soundIn);
 
  Serial.print("Temperature: ");
  Serial.println(DHT.temperature);
  mySerial.print("Temperature: ");
  mySerial.println(DHT.temperature);
  
  Serial.print("Humidity: ");
  Serial.println(DHT.humidity);
  mySerial.print("Humidity: ");
  mySerial.println(DHT.humidity);

  /*SensorData data = dust.read();

  Serial.print("Raw Signal Value (0-1023): ");
  Serial.println(data.voMeasured);
  mySerial.print("Raw Signal Value (0-1023): ");
  mySerial.println(data.voMeasured);
  Serial.print("Voltage: ");
  Serial.println(data.calcVoltage);
  mySerial.print("Voltage: ");
  mySerial.println(data.calcVoltage);
  Serial.print("Dust Density: ");
  Serial.println(data.dustDensity);
  mySerial.print("Dust Density: ");
  mySerial.println(data.dustDensity);
  */
  float data = 0;
  int err = PMsensorErrSuccess;
  if ((err = PM.read(&data, true, 0.1)) != PMsensorErrSuccess) {
    Serial.print("data Error = ");
    Serial.println(err);
    delay(3000);
  }
  Serial.print("Dust: ");
  Serial.println(data);
  mySerial.print("Dust: ");
  mySerial.println(data);
  tcaselect(MPL);
    float pascals = baro.getPressure();
                  // Our weather page presents pressure in Inches (Hg)
                   // Use http://www.onlineconversion.com/pressure.htm for other units
                   Serial.print(pascals); Serial.println(" Pascals ");
                   mySerial.print(pascals); mySerial.println(" Pascals ");
                   float altm = baro.getAltitude();
                   Serial.print(altm); Serial.println(" meters");
                   mySerial.print(altm); mySerial.println(" meters");
                   float tempC = baro.getTemperature();
                   Serial.print(tempC); Serial.println("*C");
                   mySerial.print(tempC); mySerial.println("*C");
      tcaselect(SI);
                  Serial.print("Vis: "); Serial.println(uv.readVisible());
                  mySerial.print("Vis: "); mySerial.println(uv.readVisible());
                  Serial.print("IR: "); Serial.println(uv.readIR());
                   mySerial.print("IR: "); mySerial.println(uv.readIR());
                   float UVindex = uv.readUV();
                   // the index is multiplied by 100 so to get the
                    // integer index, divide by 100!
                    UVindex /= 100.0;  
                    Serial.print("UV: ");  Serial.println(UVindex);
                    mySerial.print("UV: ");  mySerial.println(UVindex);
  Serial.println();
  mySerial.println();
          
  delay(2000);

}

