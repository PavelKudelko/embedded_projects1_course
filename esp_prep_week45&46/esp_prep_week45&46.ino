/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com  
*********/

#include "FS.h"
// #include "SPIFFS.h"
 
void setup() {
  Serial.begin(9600);
  Serial.println("hellloooo");
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  File file = SPIFFS.open("/test_example.txt", "r");
  if(!file){
    Serial.println("/nFailed to open file for reading");
    //return;
  }

    File file2 = SPIFFS.open("/test.txt", "r");
  if(!file2){
    Serial.println("/nFailed to open file for reading");
    return;
  }
  
  Serial.println();
  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();


    while(file2.available()){
    Serial.write(file2.read());
  }
  file2.close();
}
 
void loop() {

}
