#include <SD.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//OLED
#define SCREEN_WIDTH 128 // ancho de la pantalla OLED
#define SCREEN_HEIGHT 64 // altura de la pantalla OLED

// Declara un objeto de la clase Adafruit_SSD1306 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//GPS
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);
TinyGPSPlus gps;

//variables gps
float Latitud=0;
float Longitud=0;
float Altitud=0;
float Speed=0;


//DS3231
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
 

//Direccion I2C de la IMU
#define MPU 0x69
 
//Ratios de conversion
#define A_R 16384.0 // 32768/2
#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];

String valores;
String valores2;

long tiempo_prev;
float dt;

void setup()
{
  // Inicializa la pantalla OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();  // Limpia la pantalla

  // Configura el texto
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(F("Test OLED"));
  
  // Muestra el contenido en la pantalla OLED
  display.display();

  Serial.begin(115200);
  if(!SD.begin(5))
  {
    Serial.println("Error al inicializar la tarjeta SD");
    return;
  }
  Serial.println("Tarjeta SD inicializada correctamente");
  Wire.begin(); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);  
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
    }
  //rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  //delay(3000);

  //Begin serial communication Neo6mGPS
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(2000);
}

void loop()
{
  
  //Serial.println("Esto es elo que leo de la tarjeta SD");
  /*while(myFile.available())
  {
    Serial.write(myFile.read());
  }
  myFile.close();*/
  
  // gps
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    mpu(Latitud,Longitud,Altitud,Speed);
    while (neogps.available())
    {
     // mpu(Latitud,Longitud,Altitud);
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  //If newData is true
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
  }
  else
  {
    Serial.println("No data :(");
  }

  if (gps.location.isValid() == 1)
  {
    Latitud = gps.location.lat();
    Serial.println(Latitud,6);

    Longitud = gps.location.lng();
    Serial.println(Longitud,6);

    Speed = gps.speed.kmph();

    Altitud = gps.altitude.meters();  
    Serial.println(Altitud,6);
  }
  else
  {
    Latitud = 0;
    Longitud = 0;
    Altitud = 0;    
  }
}

void mpu(float Latitud, float Longitud, float Altitud, float Speed)
{
  DateTime now = rtc.now();
  uint32_t unixtime = now.unixtime();
  unixtime += 23;
  DateTime newTime = DateTime(unixtime);

  //Leer los valores del Acelerometro de la IMU
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
  AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  //A partir de los valores del acelerometro, se calculan los angulos Y, X
  //respectivamente, con la formula de la tangente.
  Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
  //Leer los valores del Giroscopio
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
  GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  //Calculo del angulo del Giroscopio
  Gy[0] = GyX/G_R;
  Gy[1] = GyY/G_R;
  Gy[2] = GyZ/G_R;
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Aplicar el Filtro Complementario
  Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
  Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];
  //Integración respecto del tiempo paras calcular el YAW
  Angle[2] = Angle[2]+Gy[2]*dt;

  //Mostrar los valores por consola
  valores = String(Angle[0]) + "," + String(Angle[1]-12);
  //Serial.println(valores);
  valores2 = String(newTime.day()) + "/" + String(newTime.month()) + "/" + String(newTime.year()) + ", " + String(newTime.hour()) + ":" + String(newTime.minute()) + ":" + String(newTime.second()) + ", " + valores + ", " + String(Latitud,6) + ", " + String(Longitud,6) + ", " + String(Altitud,6) + ", " + String(Speed,3);
  Serial.println(valores2);
  delay(50);
  File myFile = SD.open("/test5.csv", FILE_APPEND);
  if(!myFile)
  {
    Serial.println("Error al abrir el archivo");
    return;
  }
  myFile.println(valores2);
  myFile.close();
  //Serial.println("Datos escritos en el archivo");
  myFile = SD.open("/test5.csv",FILE_READ);
  if (!myFile)
  {
    Serial.println("Error al abrir el archivo");
    return;
  }

  // Configura el texto
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(newTime.day());
  display.print("-");
  display.print(newTime.month());
  display.print("-");
  display.print(newTime.year());
  display.setCursor(80,0);
  display.print(newTime.hour());
  display.print(":");
  display.print(newTime.minute());
  display.print(":");
  display.print(newTime.second());
  display.setCursor(0,10);
  display.print("Latitud: ");
  display.print(Latitud);
  display.setCursor(0,30);
  display.print("X: ");
  display.println(Angle[0]);
  display.setCursor(0,20);
  display.print("Longitud: ");
  display.print(Longitud);
  display.setCursor(0,40);
  display.print("Y: ");
  display.println(Angle[1]);



  // Muestra el contenido en la pantalla OLED
  display.display();
  delay(10);
  // Limpia la pantalla
  display.clearDisplay();

  

}