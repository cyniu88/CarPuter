/*
Modificamos o codigo do TCC1 para tirar o controle do motor DC e colocar um servo motor

Obs.: O mane' do Biliki estragou o codigo do TCC1 que NAO ESTA' FUNCIONANDO!!!

*/


  #include <SoftwareSerial.h>
  //#include <Timer.h>
  
  #define R 2
  #define PWM 3
  #define L 4 
  #define MIN 40
  #define SET 5

  int i, j = 0;
  float kc = 0.2, ki = 0.05, kd = 0;
  
  SoftwareSerial mySerial(7, 8); // RX, TX
  
  String check = "";
  char c;
  int flag = 0, sair = 0;
 
 
  unsigned int throttle;
  int tolerancia = 5;
  int targetAcel = 50;
  int servo = 100;
  int LoopDelay = 500;
  
  void setup() {
      Serial.begin(38400);
      pinMode(R, OUTPUT);
      pinMode(L, OUTPUT);
      pinMode(PWM, OUTPUT); 
    
      mySerial.begin(38400);
      delay(800);
      mySerial.flush(); 
      Serial.flush(); 
  
      while (1) {
          if (mySerial.available())
          {  
              delay(100);  
              //transfere o buffer do mySerial para a string check
              while (mySerial.available() > 0)  
              {
                  c = mySerial.read();
                  check += c;
                  flag = 1;
              }
              if(flag==1) {
                  Serial.print(check);
                  Serial.write("\n");
                  throttle_calc();
                  check.remove(0);
                  flag = 0;
              }
          }
      
          if (Serial.available()) {  
              delay(100);  
              while (Serial.available() > 0)  //transfere o buffer do mySerial para a string check
              {
                  c = Serial.read();
                  check += c;
                  flag = 1;
              }
          
              if(flag==1) {
                  if (check == "sair") {
                      mySerial.write("0111\r\n");
                      check.remove(0);
                      mySerial.flush();
                      break;
                  }
                  
                  if (check == "auto") {
                      mySerial.flush();
                      check = "AT+RESET\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(1000);
                      resposta();
                      
                      check = "AT+ROLE=1\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(1000);
                      resposta();
                      
                      check = "AT+CMODE=0\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(1000);
                      resposta();
                      
                      check = "AT+INIT\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(1000);
                      resposta();
                      
                      check = "AT+BIND=8818,56,6898EB\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(3000);
                      resposta();
                      
                      check = "AT+PAIR=8818,56,6898EB,10\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(11000);
                      resposta();
                       
                      check = "AT+LINK=8818,56,6898EB\r\n";
                      mySerial.print(check);
                      check.remove(0);
                      delay(11000);
                      resposta();
                      
                      mySerial.write("ATSP5\r\n");                  
                      delay(1000);
                      resposta();
                      
                      mySerial.write("0111\r\n");
                      resposta();
                      
                      check.remove(0);
                      mySerial.flush();
                      break;
                      
                  }
                  
                  Serial.print(check);
                  Serial.write("\n");
                  mySerial.flush();
                  check += "\r\n";
                  mySerial.print(check);
                  check.remove(0);
                  flag = 0;
              }
          }
      }
  }
  
  void loop() {  
      
      if (Serial.available() > 0) {
          delay(10);
          targetAcel = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
          
          if (targetAcel < 20) 
              targetAcel = 20;
              
          if (targetAcel > 250) 
              targetAcel = 250;      
      }
  
 
//      t.update();
  
  
      getThrottle();  
      
      if (throttle != -1) {
          LoopDelay = 250;
          if (throttle > (targetAcel+tolerancia))
              if (servo > 10) servo=servo-SET;
          if (throttle < (targetAcel-tolerancia))
              if (servo < 245) servo=servo+SET;
      }
      else LoopDelay = 300;
  
// Debugging...  
      Serial.print("   targetAcel: ");
      Serial.print(targetAcel);
      Serial.print("   throttle: ");
      Serial.print(throttle);
      Serial.print("   Servo: ");
      Serial.print(servo);
      Serial.print("   Delay: ");
      Serial.print(LoopDelay);
      Serial.println();
      
      delay(LoopDelay);
  }
  
  
  
  
  //THROTTLE LOAD
  void throttle_calc(){
      boolean valid;  
     
      valid=false;
      
      if ((check[5]=='4') && (check[6]=='1') && (check[8]=='1') && (check[9]=='1')){ 
          valid=true;
      } else {
          valid=false;
      }
      
      if (valid){
          String loadHex(check[11]); 
          String loadHex2(check[12]); 
          
          String loadHexTotal=loadHex+loadHex2;
          int DecimalDecode=hexToDec(loadHexTotal);
          throttle=round((float(DecimalDecode)/255)*100); //Arredonda e devolve valor final
      }   
       else throttle=-1;
  }
  
  unsigned int hexToDec(String hexString) {  
      unsigned int decValue = 0;
      int nextInt;
      
      for (int i = 0; i < hexString.length(); i++) {      
          nextInt = int(hexString.charAt(i));
          if (nextInt >= 48 && nextInt <= 57) 
              nextInt = map(nextInt, 48, 57, 0, 9);
          if (nextInt >= 65 && nextInt <= 70) 
              nextInt = map(nextInt, 65, 70, 10, 15);
          if (nextInt >= 97 && nextInt <= 102) 
              nextInt = map(nextInt, 97, 102, 10, 15);
              
          nextInt = constrain(nextInt, 0, 15);          
          decValue = (decValue * 16) + nextInt;
      }
      
      return decValue;
  }
  
  
  void getThrottle() {  
      while (mySerial.available() > 0)
      {
          c = mySerial.read();
          check += c;
          flag = 1;
      }
      
      if(flag==1) {
          throttle_calc();
          check.remove(0);
          mySerial.flush(); 
          mySerial.write("0111\r\n");
          flag = 0;
      }
  }


void resposta() {
    while (mySerial.available() > 0)  
        {
            c = mySerial.read();
            check += c;
        }
    Serial.print(check);
    Serial.write("\n"); 
    check.remove(0);    
 }
