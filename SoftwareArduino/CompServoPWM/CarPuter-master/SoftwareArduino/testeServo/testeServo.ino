  #include <SoftwareSerial.h>
  #include <Servo.h> 


  float ontem = millis();
  
  int i, j, posicaoCabo = 0;
  int tolerancia = 0, toleranciaki = 20;
  int targetAceleracao = 500, targetVelocidade = 50;
  float soma = 0, FposicaoCabo = 0, lastError = 0;  
  float kc = 0.2, ki = 0.05, kd = 0;

  SoftwareSerial mySerial(7, 8); // RX, TX
  
  String check = "";
  char c;
  int flag = 0, sair = 0;
  unsigned int throttle;
  Servo myServo; 


  
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
          //ajuste();
          check = "";
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
    check = "";    
 }

  
  void setup() {
      Serial.begin(38400);    
      mySerial.begin(38400);
      delay(800);
      mySerial.flush(); 
      Serial.flush(); 
      myServo.attach(3); 
  
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
                  check = "";
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
                      mySerial.write("ATSP5\r\n");
                      check = "";
                      mySerial.flush();
                      break;
                  }
                  
                  if (check == "auto") {
                      mySerial.flush();
                      check = "AT+RESET\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+ROLE=1\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+CMODE=0\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+INIT\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(1000);
                      resposta();
                      
                      check = "AT+BIND=8818,56,6898EB\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(3000);
                      resposta();
                      
                      check = "AT+PAIR=8818,56,6898EB,10\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(11000);
                      resposta();
                       
                      check = "AT+LINK=8818,56,6898EB\r\n";
                      mySerial.print(check);
                      check = "";
                      delay(11000);
                      resposta();
                      
                      mySerial.write("ATSP5\r\n");                  
                      delay(1000);
                      resposta();
                      
                      mySerial.write("0111\r\n");
                      resposta();
                      
                      check = "";
                      mySerial.flush();
                      break;
                      
                  }
                  
                  Serial.print(check);
                  Serial.write("\n");
                  mySerial.flush();
                  check += "\r\n";
                  mySerial.print(check);
                  check = "";
                  flag = 0;
              }
          }
      }
  }
  
  void loop() {  

  
      getThrottle();

      if ((throttle < 130) && (throttle > 50))
        myServo.write(throttle);

      Serial.print(" T ");
      Serial.print(throttle);
      Serial.println();

    delay(300);

  }
  