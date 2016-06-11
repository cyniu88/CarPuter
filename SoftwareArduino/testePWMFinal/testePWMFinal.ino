#include <SoftwareSerial.h>
#include <PWM.h>

#define LED 13
#define LASTERRORINI 100000
#define LIMTOP -10
#define LIMDOWN 5
#define LIMDELTA 100
#define SUMMAX 10
#define INTERACTIONS 10
#define RESPONSETIME 5
#define TOL 1
#define FAIXA 5
#define TOLKI 15
#define RxD 7
#define TxD 8
#define pinServo 3
#define Reset 10

float lastProcess = 0;
float lastError = LASTERRORINI;
float kp = 0.06, ki = 0.003, kd = 0.0, kb = 0.3;
float sum = 0;
float servo = 0, pid = 0;
int pwm = 0;
int targetVelocity = 100, lastTargetVelocity = 100;
int velocityBand = 100, velocity = 0, velocityMedia = 0;
int countInteractions = 0, count = 0;

// Flags de controle
boolean goingUp = true;
boolean flagLight = false;
boolean flagBreak = true;
boolean flagPID = false;
boolean flagActivatePID = false;

// Variáveis para controlar o PWM
int32_t frequency = 150;
bool success = false;


SoftwareSerial mySerial(RxD, TxD);

// Variáveis para controlar a entrada da serial
String check = "";
char c;
int flag = 0, sair = 0;


unsigned int throttle, rpm;

float PID() {
  float P, I, D, B, myPID, dt, error;
  float breaker = 0, delta = 0;

  error = targetVelocity - velocity;

  dt = (millis() - lastProcess) / 1000.0;

  P = kp * error;

  sum += (error * dt);
  I = ki * sum;

  if ((dt > 0) && (lastError != 100000)) {
    delta = ((error - lastError) / dt);
    D = kd * delta;
  }
  else 
    D = delta = 0;

  if (((error > 0) && (lastError < 0)) || ((error < 0) && (lastError > 0)) || (error == 0) || (abs(delta) < LIMDELTA))
    flagBreak = false;

  if ((targetVelocity != lastTargetVelocity))
    flagBreak = true;

  if ((targetVelocity - lastTargetVelocity) > 0)
    goingUp = true;
  if ((targetVelocity - lastTargetVelocity) < 0)
    goingUp = false;

  lastTargetVelocity = targetVelocity;

  if ((flagBreak) && (error != 0))
    if (goingUp) breaker = -1 * abs(delta / (error * error));
    else breaker = abs(delta / (error * error));
  else
    breaker = 0;

  B = kb * breaker;

  if(B > LIMDOWN)
    B = LIMDOWN;      
  if(B < LIMTOP)
    B = LIMTOP;       
  
  lastError = error;
  lastProcess  = millis();

  myPID = P + I + D + B;
  
  return myPID;
}



void IA() {
  if (!flagPID) {
    if ((velocity < (velocityBand + FAIXA)) && (velocity > (velocityBand - FAIXA))) {
      countInteractions++;
    } else {
      countInteractions = 0;
      velocityBand = velocity;
    }
    if (countInteractions > INTERACTIONS) {
      digitalWrite(LED, HIGH);
      flagLight = true;
      targetVelocity = velocity;
      servo = pwm;        
    }
    if (flagLight) {
      if (count > 3) { 
        flagActivatePID = true;
        flagPID = true;
      } else {
        sum = 0;
      }
      count++;
    }
  } else {
    if (throttle > targetVelocity) {
      flagPID = false;
      flagActivatePID = false;
      digitalWrite(LED, LOW);
      flagLight = false;
      countInteractions = 0;
      count = 0;
      velocityBand = velocity;
    }
  }
}


void rpm_calc() {
  boolean valid;  
  int i;

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
    mySerial.write("0111\r\n");
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
    mySerial.write("010C\r\n");
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
    if (mySerial.available()) {  
      delay(100);  
      while (mySerial.available() > 0) {
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
    getThrottle();
    getRPM();
    velocity = rpm;
    IA();
    pid = PID(); 

    if (flagLight) {  
      if (flagActivatePID)  
        servo += pid;      

      if (servo > 90)
        servo = 90;
      if (servo < 30)
        servo = 30;

      if ((velocity > (targetVelocity + TOLKI)) || (velocity < (targetVelocity - TOLKI))) 
        sum = 0;

      if (sum > SUMMAX)
        sum = SUMMAX;
      if (sum < -SUMMAX)
        sum = -SUMMAX;

      pwm = (int)servo;

    } else {
      pwm = throttle;
    }

    // Debugging...
    Serial.print(throttle);
    Serial.print("\t");
    Serial.print(targetVelocity);
    Serial.print("\t");
    Serial.print(velocity);      
    Serial.print("\t");
    Serial.print(velocityBand);      
    Serial.print("\t");
    Serial.print(pwm);
    Serial.print("\t");
    Serial.print(sum);
    Serial.print("\t");
    Serial.print(countInteractions);
    Serial.print("\t");
    Serial.print(flagLight);
    Serial.print("\t");
    Serial.println();

  }

  delay(300);

}
