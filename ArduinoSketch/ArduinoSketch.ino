#include "MPU9250.h"

MPU9250 mpu;

const int flex1 = A0;// Pin connected to voltage divider output
const int flex2 = A1;
const int flex3 = A2;
const int flex4 = A3;
const int flex5 = A4;//tants according to your project's design
const float VCC =5;      // voltage at Ardunio 5V line
const float R_DIV = 47000.0;  // resistor used to create a voltage divider

float flex1Upper, flex1Lower, flex2Upper, flex2Lower, flex3Upper, flex3Lower, flex4Upper, flex4Lower, flex5Upper, flex5Lower;
int sampleSizeForFlexSensorsCalibration = 10;

void setup() {
  Serial.begin(9600);
  Serial.flush();
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }
  
  pinMode(flex1, INPUT);
  pinMode(flex2, INPUT);
  pinMode(flex3, INPUT);
  pinMode(flex4, INPUT);
  pinMode(flex5, INPUT);

  float sumLowers1=0;
  float sumLowers2=0;
  float sumLowers3=0;
  float sumLowers4=0;
  float sumLowers5=0;

  float sumUppers1=0;
  float sumUppers2=0;
  float sumUppers3=0;
  float sumUppers4=0;
  float sumUppers5=0;

  Serial.println("@Starting;");
  delay(2000);
  
  Serial.println("@Open your hand imbecile;");
  delay(2000);
  for(int i=0; i<sampleSizeForFlexSensorsCalibration; i++){//Lowers
    sumLowers1+=R_DIV * (VCC / (analogRead(flex1) * VCC / 1023.0) - 1.0);
    sumLowers2+=R_DIV * (VCC / (analogRead(flex2) * VCC / 1023.0) - 1.0);
    sumLowers3+=R_DIV * (VCC / (analogRead(flex3) * VCC / 1023.0) - 1.0);
    sumLowers4+=R_DIV * (VCC / (analogRead(flex4) * VCC / 1023.0) - 1.0);
    sumLowers5+=R_DIV * (VCC / (analogRead(flex5) * VCC / 1023.0) - 1.0);
  }

  Serial.println("@Close your hand imbecile;");
  delay(2000);
  
  for(int i=0; i<sampleSizeForFlexSensorsCalibration; i++){//Uppers
    sumUppers1+=R_DIV * (VCC / (analogRead(flex1) * VCC / 1023.0) - 1.0);
    sumUppers2+=R_DIV * (VCC / (analogRead(flex2) * VCC / 1023.0) - 1.0);
    sumUppers3+=R_DIV * (VCC / (analogRead(flex3) * VCC / 1023.0) - 1.0);
    sumUppers4+=R_DIV * (VCC / (analogRead(flex4) * VCC / 1023.0) - 1.0);
    sumUppers5+=R_DIV * (VCC / (analogRead(flex5) * VCC / 1023.0) - 1.0);
  }
  
  flex1Lower=sumLowers1/sampleSizeForFlexSensorsCalibration;
  flex1Upper=sumUppers1/sampleSizeForFlexSensorsCalibration;

  flex2Lower=sumLowers2/sampleSizeForFlexSensorsCalibration;
  flex2Upper=sumUppers2/sampleSizeForFlexSensorsCalibration;

  flex3Lower=sumLowers3/sampleSizeForFlexSensorsCalibration;
  flex3Upper=sumUppers3/sampleSizeForFlexSensorsCalibration;

  flex4Lower=sumLowers4/sampleSizeForFlexSensorsCalibration;
  flex4Upper=sumUppers4/sampleSizeForFlexSensorsCalibration;

  flex5Lower=sumLowers5/sampleSizeForFlexSensorsCalibration;
  flex5Upper=sumUppers5/sampleSizeForFlexSensorsCalibration;
}

void loop() {
  float Rflex1 = R_DIV * (VCC / (analogRead(flex1) * VCC / 1023.0) - 1.0);
  float Rflex2 = R_DIV * (VCC / (analogRead(flex2) * VCC / 1023.0) - 1.0);
  float Rflex3 = R_DIV * (VCC / (analogRead(flex3) * VCC / 1023.0) - 1.0);
  float Rflex4 = R_DIV * (VCC / (analogRead(flex4) * VCC / 1023.0) - 1.0);
  float Rflex5 = R_DIV * (VCC / (analogRead(flex5) * VCC / 1023.0) - 1.0);

  float angle1 = map(Rflex1, flex1Lower, flex1Upper, 0, 90.0);
  float angle2 = map(Rflex2, flex2Lower, flex2Upper, 0, 90.0);
  float angle3 = map(Rflex3, flex3Lower, flex3Upper, 0, 90.0);
  float angle4 = map(Rflex4, flex4Lower, flex4Upper, 0, 90.0);
  float angle5 = map(Rflex5, flex5Lower, flex5Upper, 0, 90.0);

  if (mpu.update()){
   static uint32_t prev_ms = millis();
   if(millis() > prev_ms) {
      char flexSensorRes[45] = "", imuRes[25]="", res[75]="";
      char bufferT1Flex[8], bufferT2Flex[8], bufferT3Flex[8], bufferT4Flex[8], bufferT5Flex[8];
      char bufferT1IMU[8], bufferT2IMU[8], bufferT3IMU[8];
      int bufferStartIndex = 0;
     
      dtostrf(angle1, 4, 2, bufferT1Flex);
      dtostrf(angle2, 4, 2, bufferT2Flex);
      dtostrf(angle3, 4, 2, bufferT3Flex);
      dtostrf(angle4, 4, 2, bufferT4Flex);
      dtostrf(angle5, 4, 2, bufferT5Flex);

      dtostrf(mpu.getRoll(), 4, 2, bufferT1IMU);
      dtostrf(mpu.getPitch(), 4, 2, bufferT2IMU);
      dtostrf(mpu.getYaw(), 4, 2, bufferT3IMU);

      int bufferT1FlexEnd = getNonNumberIndex(bufferT1Flex);
      int bufferT2FlexEnd = getNonNumberIndex(bufferT2Flex);
      int bufferT3FlexEnd = getNonNumberIndex(bufferT3Flex);
      int bufferT4FlexEnd = getNonNumberIndex(bufferT4Flex);
      int bufferT5FlexEnd = getNonNumberIndex(bufferT5Flex);

      int bufferT1IMUEnd = getNonNumberIndex(bufferT1IMU);
      int bufferT2IMUEnd = getNonNumberIndex(bufferT2IMU);
      int bufferT3IMUEnd = getNonNumberIndex(bufferT3IMU);

      for(int i=0;i<bufferT1FlexEnd;i++)flexSensorRes[bufferStartIndex++]=bufferT1Flex[i];
      flexSensorRes[bufferStartIndex++]='|';
     
      for(int i=0;i<bufferT2FlexEnd;i++)flexSensorRes[bufferStartIndex++]=bufferT2Flex[i];
      flexSensorRes[bufferStartIndex++]='|';

      for(int i=0;i<bufferT3FlexEnd;i++)flexSensorRes[bufferStartIndex++]=bufferT3Flex[i];
      flexSensorRes[bufferStartIndex++]='|';

      for(int i=0;i<bufferT4FlexEnd;i++)flexSensorRes[bufferStartIndex++]=bufferT4Flex[i];
      flexSensorRes[bufferStartIndex++]='|';

      for(int i=0;i<bufferT5FlexEnd;i++)flexSensorRes[bufferStartIndex++]=bufferT5Flex[i];
      for(int i=bufferStartIndex;i<44;i++)flexSensorRes[i]=';';

      bufferStartIndex = 0;
      
      for(int i=0;i<bufferT1IMUEnd;i++)imuRes[bufferStartIndex++]=bufferT1IMU[i];
      imuRes[bufferStartIndex++]='|';

      for(int i=0;i<bufferT2IMUEnd;i++)imuRes[bufferStartIndex++]=bufferT2IMU[i];
      imuRes[bufferStartIndex++]='|';
     
      for(int i=0;i<bufferT3IMUEnd;i++)imuRes[bufferStartIndex++]=bufferT3IMU[i];
      for(int i=bufferStartIndex;i<24;i++)imuRes[i]=';';
      
      bufferStartIndex = 0;
      for(int i=0;i<44;i++){
        if(flexSensorRes[i]==';')break;
        res[bufferStartIndex++] = flexSensorRes[i];
      }
      res[bufferStartIndex++]='~';
      for(int i=0;i<24;i++){
        if(imuRes[i]==';')break;
        res[bufferStartIndex++] = imuRes[i];
      }
      
      for(int i=bufferStartIndex;i<74;i++)res[i]=';';
     
      Serial.println(res);
      prev_ms = millis();
    }
  }
}

int getNonNumberIndex(char r[8]){
  for(int k=0;k<8;k++){
    if(r[k] != '.'&&
        r[k] != '-'&&
        r[k] != '0'&&
        r[k] != '1'&&
        r[k] != '2'&&
        r[k] != '3'&&
        r[k] != '4'&&
        r[k] != '5'&&
        r[k] != '6'&&
        r[k] != '7'&&
        r[k] != '8'&&
        r[k] != '9'){
      return k;
    }
  }
}
