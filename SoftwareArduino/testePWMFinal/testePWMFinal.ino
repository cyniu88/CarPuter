#include <SoftwareSerial.h>
#include <PWM.h>


#define RxD 7
#define TxD 8
#define pinServo 3

int pwm = 0;
int32_t frequency = 150;
bool success = false;
bool flagT = true;


SoftwareSerial mySerial(RxD, TxD);

// Variáveis para controlar a entrada da serial
String check = "";
char c;
int flag = 0, sair = 0;


unsigned int throttle, rpm;

void rpm_calc() {
  boolean valid;  
  int i;

  Serial.println("check rpm resp");
  Serial.println(check);

  valid=false;

  if ((check[5]=='4') && (check[6]=='1') && (check[8]=='0') && (check[9]=='C')){ 
    valid=true;
  } else {
    valid=false;
  }

  if (valid){
    rpm = 0;
    for (i = 10; i < 14; i++) {
      if ((check[i] >= 'A') && (check[i] <= 'F')) {
        check[i] -= 55;
      } 
      if ((check[i] >= '0') && (check[i] <= '9')) {
        check[i] -= 48;
      }
      rpm = (rpm << 4) | (check[i] & 0xf);
    }
    rpm = rpm >> 2;
  }     
}


void throttle_calc(){
  boolean valid;  
  Serial.println("check tr resp");
  Serial.println(check);

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
    throttle=round((float(DecimalDecode)/255)*100);
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
  while (mySerial.available() > 0) {
    c = mySerial.read();
    check += c;
    flag = 1;
  }

  if(flag==1) {
    throttle_calc();
    check = "";
    mySerial.flush(); 
    flag = 0;
  }
}



void getRPM() {  
  while (mySerial.available() > 0) {
    c = mySerial.read();
    check += c;
    flag = 1;
  }

  if(flag==1) {
    rpm_calc();
    check = "";
    mySerial.flush(); 
    flag = 0;
  }
}



void resposta() {
  while (mySerial.available() > 0) {
    c = mySerial.read();
    check += c;
  }
  Serial.print(check);
  Serial.write("\n"); 
  check = "";    
}




void pareiaBT() {
  while (1) {

    if (Serial.available()) {  
      delay(100);  
      while (Serial.available() > 0) {
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


void setup() {

  InitTimersSafe();
  success = SetPinFrequencySafe(pinServo, frequency);
  Serial.begin(38400);    
  mySerial.begin(38400);
  delay(800);
  mySerial.flush(); 
  Serial.flush(); 
  pareiaBT();
}

void loop() {  
  if (success) {
    if (flagT) {
      flagT = false;
      getThrottle();
    } else {
      flagT = true;
      getRPM();
    }

    
    // Debugging...
    Serial.print(throttle);
    Serial.print("\t");
    Serial.print(rpm);
    Serial.println();
  }

  delay(300);

}
