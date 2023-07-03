#include "LoRa_E32.h"

LoRa_E32 e32ttl(&Serial2);
typedef struct{
  float SICAKLIK ;
  float BASINC ;
  float YUKSEKLIK ;
  float IVME_X ;
  float IVME_Y ; 
  float IVME_Z ; 
  float GYRO_X ; 
  float GYRO_Y ; 
  float GYRO_Z ; 
  float ENLEM  ;
  float BOYLAM ;
}
  Signal;
  Signal data;
void setup() {
  Serial.begin(9600);
  
  e32ttl.begin();
  delay(500);

}

void loop() {

    while (e32ttl.available()  > 1) {
    Serial.println("VERİ YOLLUYORUM :)");
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    data = *(Signal*) rsc.data;
    rsc.close();
    
    Serial.print(F("SICAKLIK = "));
    Serial.print(data.SICAKLIK);
    Serial.println(" *C");
    Serial.print(F("BASINÇ = "));
    Serial.print(data.BASINC); //displaying the Pressure in hPa, you can change the unit
    Serial.println(" hPa");
    Serial.print(F("YÜKSEKLİK = "));
    Serial.print(data.YUKSEKLIK+8); //The "1019.66" is the pressure(hPa) at sea level in day in your region
    Serial.println(" m");                    //If you don't know it, modify it until you get your current altitude
    
    Serial.print("IVME_X : ");Serial.print(data.IVME_X);                //X EKSENİNDE İVME
    Serial.print("\tIVME_Y : ");Serial.print(data.IVME_Y);              //Y EKSENİNDE İVME 
    Serial.print("\tIVME_Z : ");Serial.println(data.IVME_Z);            //Z EKSENİNDE İVME 
    Serial.print("GYRO_X : ");Serial.print(data.GYRO_X);
    Serial.print("\tGYRO_Y : ");Serial.print(data.GYRO_Y);
    Serial.print("\tGYRO_Z : ");Serial.println(data.GYRO_Z);

    Serial.print("ENLEM: ");Serial.print(data.ENLEM ,6);
    Serial.print("\tBOYLAM: ");Serial.print(data.BOYLAM,6);
    Serial.println();
    Serial.println();
    
  }

}
