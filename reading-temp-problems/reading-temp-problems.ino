 #include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (0.0)

#define TX A14
#define RX A15

SoftwareSerial mySerial(RX, TX); //TX, RX
Adafruit_BME280 bme; // I2C
File dataFile;

const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

int buttonState = 0;

const int chipSelect = 4;

int i;

int itteration;

String dataTransmission = "";

//sets Temp_Constants

//O3 Temp_Constants
float O3_Temp_Constants_Eq1[] = {0.9, 0.9, 1.0, 1.3, 1.5, 1.7, 2.0, 2.5, 3.7};
float O3_Temp_Constants_EQ2[] = {0.5, 0.5, 0.6, 0.8, 0.9, 1.0, 1.2, 1.5, 2.2};
float O3_Temp_Constants_EQ3[] = {0.5, 0.5, 0.5, 0.6, 0.6, 1.0, 2.8, 5.0, 5.3};
float O3_Temp_Constants_EQ4[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 8.5, 23.0, 103.0};

//SO2 Temp_Constants
float SO2_Temp_Constants_EQ1[] = {1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.9, 3.0, 5.8};
float SO2_Temp_Constants_EQ2[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.2, 1.9, 3.6};
float SO2_Temp_Constants_EQ3[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 3.5, 7.0};
float SO2_Temp_Constants_EQ4[] = { -4.0, -4.0, -4.0, -4.0, -4.0, 0.0, 20.0, 140.0, 450.0};

//H2S Temp_Constants
float H2S_Temp_Constants_EQ1[] = { -0.6, -0.6, 0.1, 0.8, -0.7, -2.5, -2.5, -2.2, -1.8};
float H2S_Temp_Constants_EQ2[] = {0.2, 0.2, 0.0, -0.3, 0.3, 1.0, 1.0, 0.9, 0.7};
float H2S_Temp_Constants_EQ3[] = { -14.0, -14.0, 3.0, 3.0, 2.0, 1.0, -1.2, -1.2, -1.2};
float H2S_Temp_Constants_EQ4[] = {52.0, 51.0, 48.0, 45.0, 26.0, 0.0, -65.0, -125.0, -180.0};

//CO Temp_Constants
float CO_Temp_Constants_EQ1[] = {0.7, 0.7, 0.7, 0.7, 1.0, 3.0, 3.5, 4.0, 4.05};
float CO_Temp_Constants_EQ2[] = {0.2, 0.2, 0.2, 0.2, 0.3, 1.0, 1.2, 1.3, 1.5};
float CO_Temp_Constants_EQ3[] = { -1.0, -0.5, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
float CO_Temp_Constants_EQ4[] = {55.0, 55.0, 55.0, 50.0, 31.0, 0.0, -50.0, -150.0, -250.0};

//NO2 Temp_Constants
float NO2_Temp_Constants_EQ1[] = {1.3, 1.3, 1.3, 1.3, 1.0, 0.6, 0.4, 0.2, -1.5};
float NO2_Temp_Constants_EQ2[] = {2.2, 2.2, 2.2, 2.2, 1.7, 1.0, 0.7, 0.3, -2.5};
float NO2_Temp_Constants_EQ3[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.4, -0.1, -4.0};
float NO2_Temp_Constants_EQ4[] = {7.0, 7.0, 7.0, 7.0, 7.0, 0.0, 0.5, 5.0, 67.1};

//NO Temp_Constants
float NO_Temp_Constants_EQ1[] = {2.9, 2.9, 2.2, 1.8, 1.7, 1.6, 1.5, 1.4, 1.3};
float NO_Temp_Constants_EQ2[] = {1.8, 1.8, 1.4, 1.1, 1.1, 1.0, 0.9, 0.9, 0.8};
float NO_Temp_Constants_EQ3[] = {0.8, 0.8, 0.8, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3};
float NO_Temp_Constants_EQ4[] = { -25.0, -25.0, -25.0, -25.0, -16.0, 0.0, 10.0, 35.0, 132.0};

float OP1_Conversion, OP2_Conversion, Temp_Constant, OP1_Zero, OP2_Zero;

int  Value_OP_A1, Value_OP_A2, Value_OP_B1, Value_OP_B2,  Value_OP_C1, Value_OP_C2, Value_OP_D1,
     Value_OP_D2, Value_OP_E1, Value_OP_E2,  Value_OP_F1,  Value_OP_F2;

float WEC_RAW, WEC_Fixed, Ambient_Temp;

float PPB_NO, PPB_NO2, PPB_CO, PPB_O3, PPB_SO2, PPB_H2S;

float Sensor_Pin_OP_A_1 = A0; //CO OP 1
float Sensor_Pin_OP_A_2 = A1; //CO OP 2

float Sensor_Pin_OP_B_1 = A2; //NO2 OP 1
float Sensor_Pin_OP_B_2 = A3; //NO2 OP 2

float Sensor_Pin_OP_C_1 = A4; //NO OP 1
float Sensor_Pin_OP_C_2 = A5; //NO OP 2

float Sensor_Pin_OP_D_1 = A8; //H2S OP 1
float Sensor_Pin_OP_D_2 = A9; // H2S OP 2

float Sensor_Pin_OP_E_1 = A10; // SO2 OP 1
float Sensor_Pin_OP_E_2 = A11; // SO2 OP 2

float Sensor_Pin_OP_F_1 = A12; //O3 OP 1
float Sensor_Pin_OP_F_2 = A13; // O3 OP 2


//gets Temp_Constant for Lists
float getTemp_Constant(){
 
  Ambient_Temp = bme.readTemperature();
  
  if (-30.0 < Ambient_Temp && Ambient_Temp<=-25.0){
 
    Temp_Constant = 0;
 
 
  }else if(-25.0 < Ambient_Temp && Ambient_Temp <= -15.0){
    
    Temp_Constant = 1;
    
   
    
 }else if (-15.0 < Ambient_Temp && Ambient_Temp<= -5.0){
 
    Temp_Constant = 2;
 
    
 }else if (-5.0< Ambient_Temp && Ambient_Temp<= 5.0){
 
    Temp_Constant = 3;
 
 
 }else if (5.0 < Ambient_Temp && Ambient_Temp<= 15.0){
 
    Temp_Constant = 4;
   
 
 }else if(15.0 < Ambient_Temp && Ambient_Temp<= 25.0){
 
    Temp_Constant = 5;
    
 
 }else if(25.0 < Ambient_Temp && Ambient_Temp<= 35.0){
 
    Temp_Constant = 6;
   
 
 }else if (35.0 < Ambient_Temp && Ambient_Temp <=50.0) {
 
    Temp_Constant = 7;

  }else{
 
    Serial.println("TEMP OUTSIDE ALLOWABLE RANGE");
  }
}


//Equations for WEC_RAW
float Calculate_PPB_Equation_1(float OP1_Conversion, float OP2_Conversion, float Temp_C) {

  WEC_RAW = OP1_Conversion - Temp_C * OP2_Conversion;

  return abs(WEC_RAW);
}



float Calculate_PPB_Equation_2(float OP1_Conversion, float OP2_Conversion, float Temp_C, float OP1_Zero, float OP2_Zero) {

  WEC_RAW = OP1_Conversion - Temp_C * (OP1_Zero / OP2_Zero) * OP2_Conversion;

  return abs(WEC_RAW);
}



float Calculate_PPB_Equation_3(float OP1_Conversion, float OP2_Conversion, float Temp_C, float OP1_Zero, float OP2_Zero) {

  WEC_RAW = OP1_Conversion - (OP1_Zero - OP2_Zero) - (Temp_C * OP2_Conversion);

  return abs(WEC_RAW);
}



float Calculate_PPB_Equation_4(float OP1_Conversion, float OP1_Zero, float Temp_C) {

  WEC_RAW = OP1_Conversion - OP1_Zero - Temp_C;

  return abs(WEC_RAW);

}



//Calculates WEC_RAW O3
float Calculate_PPB_O3() {

  i = getTemp_Constant();

  Value_OP_F1 = analogRead(Sensor_Pin_OP_F_1); // Sets O3 OP 1

  Value_OP_F2 = analogRead(Sensor_Pin_OP_F_2); // Sets O3 OP 2

  float conversion_O3_A = (Value_OP_F1) * (5 / 1023.0);

  float conversion_O3_B = (Value_OP_F2) * (5 / 1023.0);

  Calculate_PPB_Equation_3(conversion_O3_A, conversion_O3_B, O3_Temp_Constants_EQ3[i], 0.245, .244);

  WEC_Fixed = abs(WEC_RAW / .000289);

  Serial.print(WEC_Fixed);
  Serial.print("   ");

  dataFile.print(WEC_Fixed, 4);
  dataFile.print("   ");

  //Save data for transmission via HC-12 modules
  dataTransmission += floatToString(WEC_Fixed) + ",";
}


//Calculates WEC_RAW SO2
float Calculate_PPB_SO2() {

  i = getTemp_Constant();

  Value_OP_E1 = analogRead(Sensor_Pin_OP_E_1); // Sets SO2 OP 1

  Value_OP_E2 = analogRead(Sensor_Pin_OP_E_2); // Sets SO2 OP 2

  float conversion_SO2_A = ( Value_OP_E1) * (5.0 / 1023.0);

  float conversion_SO2_B = (Value_OP_E2) * (5.0 / 1023.0);

  Calculate_PPB_Equation_4(conversion_SO2_A, .332, SO2_Temp_Constants_EQ4[i]);

  WEC_Fixed = abs(WEC_RAW / .000340);

  Serial.print(WEC_Fixed);
  Serial.print("   ");
  
  dataFile.print(WEC_Fixed, 4);
  dataFile.print("   ");

  //Save data for transmission via HC-12 modules
  dataTransmission += floatToString(WEC_Fixed) + ",";
}


//Calculates WEC_RAW H2S
float Calculate_PPB_H2S() {
  

//  i = getTemp_Constant();
//
//  Value_OP_D1 = analogRead(Sensor_Pin_OP_D_1); // Sets H2S OP 1
//
//  Value_OP_D2 = analogRead(Sensor_Pin_OP_D_2); // Sets H2S OP 2
//
//  float conversion_H2S_A = ( Value_OP_D1) * (5.0 / 1023.0)  + .015;
//
//  float conversion_H2S_B = ( Value_OP_D2) * (5.0 / 1023.0) + .015;
//
//  Calculate_PPB_Equation_2(conversion_H2S_A, conversion_H2S_B, H2S_Temp_Constants_EQ2[i], .229, .245);
//
//  WEC_Fixed = abs(WEC_RAW / .001532);
//
//
//  //Serial.print(WEC_Fixed, 3);
//  Serial.print(WEC_Fixed);
  Serial.print("   ");

  //Save data for transmission via HC-12 modules
  //dataTransmission += floatToString(WEC_Fixed) + ",";
}


//Calculates WEC_RAW NO
float Calculate_PPB_NO() {

  i = getTemp_Constant();

  Value_OP_C1 = analogRead(Sensor_Pin_OP_C_1); // Sets NO OP 1

  Value_OP_C2 = analogRead(Sensor_Pin_OP_C_2); // Sets NO OP 2

  float conversion_NO_A = (Value_OP_C1) * (5.0 / 1023.0);

  float conversion_NO_B = ( Value_OP_C2) * (5.0 / 1023.0);

  Calculate_PPB_Equation_3(conversion_NO_A, conversion_NO_B, NO_Temp_Constants_EQ3[i], .262, .229);

  WEC_Fixed = abs(WEC_RAW / .000516);

  Serial.print(WEC_Fixed);
  Serial.print("   ");
  
  dataFile.print(WEC_Fixed, 4);
  dataFile.print("   ");

  //Save data for transmission via HC-12 modules
  dataTransmission += floatToString(WEC_Fixed) + ",";
}

//Calculates WEC_RAW NO2
float Calculate_PPB_NO2() {

  i = getTemp_Constant();

  Value_OP_B1 = analogRead(Sensor_Pin_OP_B_1); // Sets NO2 OP 1

  Value_OP_B2 = analogRead(Sensor_Pin_OP_B_2); // Sets NO2 OP 2

  float conversion_NO2_A = (Value_OP_B1) * (5.0 / 1023.0) + .015;

  float conversion_NO2_B = (Value_OP_B2) * (5.0 / 1023.0) + .015;

  Calculate_PPB_Equation_3(conversion_NO2_A, conversion_NO2_B, NO2_Temp_Constants_EQ3[i], .225, .234);

  WEC_Fixed = abs(WEC_RAW / .000226);

  Serial.print(WEC_Fixed);
  Serial.print("   ");
  
  dataFile.print(WEC_Fixed, 4);
  dataFile.print("   ");

  //Save data for transmission via HC-12 modules
  dataTransmission += floatToString(WEC_Fixed) + ",";
}

//Calculates WEC_RAW CO
float Calculate_PPB_CO() {

  i = getTemp_Constant();

  Value_OP_A1 = analogRead(Sensor_Pin_OP_A_1); // Sets CO OP 1

  Value_OP_A2 = analogRead(Sensor_Pin_OP_A_2); // Sets CO OP 2

  //Converts Data for Algo.
  float conversion_CO_A = (Value_OP_A1) * (5.0 / 1023.0);

  float conversion_CO_B = (Value_OP_A2) * (5.0 / 1023.0);

  Calculate_PPB_Equation_2(conversion_CO_A, conversion_CO_B, CO_Temp_Constants_EQ2[i], .233, .234);

  WEC_Fixed = abs(WEC_RAW / .000478);

  Serial.print(WEC_Fixed);
  Serial.print("   ");

  dataFile.print(WEC_Fixed, 4);
  dataFile.print("  ");

  //Save data for transmission via HC-12 modules
  dataTransmission += floatToString(WEC_Fixed) + ",";
}

String floatToString(float num) {
  String result = "";
  float temp = num * 10;
  result += String(temp).substring(0, String(temp).indexOf(".") - 1);
  result += "." + String(temp).charAt(String(temp).indexOf(".") - 1);
  result += String(temp).substring(String(temp).indexOf(".") + 1, sizeof(String(temp)));
  
  return result;
}


void setup() {
  
   //GAS SENSORS
  //Kill
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(chipSelect, OUTPUT);
  pinMode(53, OUTPUT);
  
  Serial.begin(9600);
  mySerial.begin(9600);
  bme.begin();
  SD.begin();
  
  delay(2000); 
  dataFile = SD.open("data.txt", FILE_WRITE);
 
  delay(1000);
  dataFile.print("\n");
  dataFile.println("____CO____|____O3____|____SO2____|____NO2____|____NO____");
  dataFile.print("\n");

  Serial.print(bme.readTemperature());
  Serial.print("\n");

  Serial.println("____CO____|____O3____|____SO2____|____NO2____|____NO____");

}



void loop() {

  buttonState = digitalRead(buttonPin);

  if (itteration >= -60) { //Set itteration to how many minutes you want 

    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);

    Serial.print(itteration);

    Serial.print("  ");
    Serial.print(bme.readTemperature());
    Serial.print("  ");

    Calculate_PPB_CO();
    
    Calculate_PPB_O3();

    Calculate_PPB_SO2();

    Calculate_PPB_H2S();

    Calculate_PPB_NO2();

    Calculate_PPB_NO();

    mySerial.print(dataTransmission);
    dataTransmission = "";
    
    dataFile.print("     ");
    
    dataFile.print(bme.readTemperature());

    dataFile.println("\n");

    Serial.print(itteration);
    Serial.print("\n");

    itteration++;

    delay(500);  
    
  } else {
    Serial.print("DONE");
    dataFile.print("\n");
    dataFile.print("--------------------------------------------------------------------------------------------");
    dataFile.close();
    for (;;) {}

  }
}






