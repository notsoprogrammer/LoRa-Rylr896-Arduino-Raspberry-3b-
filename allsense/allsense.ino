const int UPDATE_FREQUENCY = 5000;     // update frequency in ms
//const float CALIBRATION_FACTOR = –4.0; // temperature calibration factor (Celsius)
const int ADDRESS = 116;
//const int ADDRESS2 = 120;
const int NETWORK_ID = 6;
const String PASSWORD = "92A0ECEC9000DA0DCF0CAAB0ABA2E0EF";
const String DELIMITER = "|";

//capacitivesoil
int moistureStatus;
int returnedADCData;
#define ADC_PIN A1     // Use any available ADC Pin, connect to A1 sensor pin
#define PWR_PIN 9      // Use any available digital pin, connect to VCC sensor pin
#define SOIL_WET 275   // Define max value we consider soil 'too wet'
#define SOIL_DRY 38//Libraries
#include <DHT.h>;
//Constants
#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
//Variables
int chk;
float humidity;  //Stores humidity value
float temperature; //Stores temperature value0   
// Define min value we consider soil 'too dry'



////library for BME280
//#include <DFRobot_BME280.h>
//#include <Adafruit_BME280.h>
//typedef DFRobot_BME280_IIC    BME;    // 
//BME   bme(&Wire, 0x77);   // select TwoWire peripheral and set sensor address
//#define SEA_LEVEL_PRESSURE    1015.0f
//float temperature;//bme280
//uint32_t pressure; //bme280
//float altitude;//bme280
//float humidity ;//bme280

//library ds18b20
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);//ds18b20

float Celcius=0;//ds18b20
float Fahrenheit=0;//ds18b20

//phsensor
#define PHSensorPin  A2    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5.0    //for arduino uno, the ADC reference is the AVCC, that is 5.0V(TYP)
#define OFFSET 0.00  //zero drift compensation
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage,phValue;
float soil_ph;
//
//void printLastOperateStatus(BME::eStatus_t eStatus) // show last sensor operate status
//{
//  switch(eStatus) {
//  case BME::eStatusOK:    Serial.println("everything ok"); break;
//  case BME::eStatusErr:   Serial.println("unknown error"); break;
//  case BME::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
//  case BME::eStatusErrParameter:    Serial.println("parameter error"); break;
//  default: Serial.println("unknown status"); break;
//  }
//}
void setup()
{
  Serial.begin(9600); 
  dht.begin();

  Serial.begin(115200); // default baud rate of module is 115200
  delay(1000);           // wait for LoRa module to be ready

  // needs all need to be same for receiver and transmitter
  Serial.print((String)"AT+ADDRESS=" + ADDRESS + "\r\n");
  delay(200);
  Serial.print((String)"AT+NETWORKID=" + NETWORK_ID + "\r\n");
  delay(200);
  Serial.print("AT+CPIN=" + PASSWORD + "\r\n");
  delay(200);
  Serial.print("AT+CPIN?\r\n"); // confirm password is set

  sensors.begin();//ds18b20
  
  pinMode(PWR_PIN, OUTPUT);    //Capacitive Set pin used to power sensor as output
  digitalWrite(PWR_PIN, LOW);  //Capacitive Set to LOW to turn sensor off at start
//  bme.reset();
//  Serial.println("bme read data test");
//  while(bme.begin() != BME::eStatusOK) {
//  Serial.println("bme begin faild");
//  printLastOperateStatus(bme.lastOperateStatus);
//  delay(2000);
//  }
//  Serial.println("bme begin success");
//  delay(100);
}
void loop()
{
  updateReadings();
  delay(UPDATE_FREQUENCY);
}

void updateReadings()
{
  
  soilmoisture();
  dhttemp();
  dhthum();
//  bmetemp();
//  bmepress();
//  bmealt();
//  bmehum();
  soiltemp();
  phsensor();
  String payload =buildPayload (moistureStatus,temperature,humidity,Celcius,soil_ph);
  Serial.print(payload);
  displayResults(moistureStatus,temperature,humidity,Celcius,soil_ph);
}
void soilmoisture()
{
  returnedADCData = readSoilADC();        // Read sensor analog output
 
 if (returnedADCData < SOIL_WET) {           // Determine status of our soil moisture situation

   moistureStatus = 1; //too wet
 }
 else if (returnedADCData >= SOIL_WET && returnedADCData < SOIL_DRY) {
  
   moistureStatus = 2; //good
 }
else {

   moistureStatus = 3; //dry
 }
}
//===============================================================================
//  Function readSoilADC returns the analog soil moisture measurement https://protosupplies.com/product/capacitive-soil-moisture-sensor-module/
//===============================================================================
int readSoilADC()
{
  digitalWrite(PWR_PIN, HIGH);       // Turn sensor power on
  delay(1000);                      // Allow circuit time to settle
  int val_ADC = analogRead(ADC_PIN); // Read analog value from sensor
  delay(100);                        // Small delay probably not needed
  digitalWrite(PWR_PIN, LOW);        // Turn sensor power off
  return val_ADC;                    // Return analog moisture value
}

void dhthum()
{
  humidity=dht.readHumidity();
}

void dhttemp()
{
    temperature=dht.readTemperature();
}
//void bmetemp()
//{
//  temperature =bme.getTemperature();
//}
//void bmepress()
//{
//  pressure = bme.getPressure() / 100.0F;
//}
//void bmealt()
//{
// altitude = bme.calAltitude(SEA_LEVEL_PRESSURE, pressure); 
//}
//void bmehum()
//{
//  humidity = bme.getHumidity();
//}
void soiltemp() //soil temperature and humidity
{
  sensors.requestTemperatures(); 
  Celcius=sensors.getTempCByIndex(0);
  Fahrenheit=sensors.toFahrenheit(Celcius);  
}

void phsensor()
{
  
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(PHSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT)
         analogBufferIndex = 0;
   }
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 1000U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
      {
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      }
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the value more stable by the median filtering algorithm
      phValue = 3.5 * averageVoltage + OFFSET;
      soil_ph = phValue;
   }
}

int getMedianNum(int bArray[], int iFilterLen)
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
      bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++)
      {
      for (i = 0; i < iFilterLen - j - 1; i++)
          {
        if (bTab[i] > bTab[i + 1])
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void displayResults(int m,float t,float u,float s,float h)
{
  Serial.print("Soil Moisture: ");
  Serial.println(m);
  Serial.print("Temperature: ");
  Serial.println(t);
//  Serial.print("Pressure: ");
//  Serial.println(p);
//  Serial.print("Altitude: ");
//  Serial.println(a);
  Serial.print("Humidity: ");
  Serial.println(u);
  Serial.print("Soil Temperature: ");
  Serial.println(s);
  Serial.print("Soil pH Level: ");
  Serial.println(h);
 Serial.println("———-");
}
String buildPayload(int m,float t,float u,float s,float h)
{
  String readings = "";
  readings += m;
  readings += DELIMITER;
  readings += t;
//  readings += DELIMITER;
//  readings += p;
//  readings += DELIMITER;
//  readings += a;
  readings += DELIMITER;
  readings += u;
  readings += DELIMITER;
  readings += s;
  readings += DELIMITER;
  readings += h;
  readings += DELIMITER;

  
  String payload = "";
  payload += "AT+SEND=";
  payload += ADDRESS;
  payload += ",";
  payload += readings.length();
  payload += ",";
  payload += readings;
  payload += "\r\n";

  return payload;
}
