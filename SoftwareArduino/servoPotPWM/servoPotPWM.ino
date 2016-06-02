  #include <PWM.h>
  #include <SoftwareSerial.h>
  
  #define LED 13
  #define LASTERRORINI 100000
  #define LIMTOP -10
  #define LIMDOWN 5
  #define LIMDOWNSERVO 1
  #define LIMTOPSERVO 25
  #define LIMDELTA 100
  #define SUMMAX 10
  #define INTERACTIONS 20
  #define RESPONSETIME 10
  #define TOL 1
  #define FAIXA 5
  #define TOLKI 15

  float lastProcess = 0;
  float lastError = LASTERRORINI;
  float kp = 0.06, ki = 0.003, kd = 0.0, kb = 0.3;
  float servo = 10, pid = 0;
  float breaker = 0, delta = 0, sum = 0;
  int targetVelocity = 10, lastTargetVelocity = 10, pwm = 0; 
  int velocityFaixa = 10, velocity = 0, velocityMedia = 0;
  int lastSensor = 0, throttle = 0, throttleMedia = 0;
  int countInteractions = 0, count = 0;
  boolean flagBreak = true, flagPID = false, goingUp = true, flagLight = false, flagAtivaPID = false;
  int32_t frequency = 150;
  bool success = false;
  int pinServo = 9;
  
  
  void setup() {
    InitTimersSafe();
    success = SetPinFrequencySafe(pinServo, frequency);
    // Define o LED como saída
    pinMode(LED, OUTPUT);  
    // Inicializa as entidades seriais com a mesma velocidade de comunicação
    Serial.begin(38400);  
    //mySerial.begin(38400);
    // Limpa qualquer dado que estiver nas seriais
    //mySerial.flush(); 
    Serial.flush();
    // Inicializa a váriavel de tempo
    lastProcess = millis(); 
  
  }
  
  void loop() {  
    if (success) {
    // Lê o valor do potenciômetro usando a média das leituras para evitar ruídos
      throttleMedia = 0;
  
      for (int j = 0; j < 100; j++)  {
          leitura = analogRead(A1);
          throttle = map(leitura, 0, 1023, 35, 100);

          throttleMedia += throttle;
      }

      throttle = throttleMedia / 100;
  

      // Define a posição do ângulo do servo
      pwmWrite(pinServo, throttle);
  
      delay(100);  
  
  // Debugging...  
  
      Serial.print(throttle);    
      Serial.print("\t");    
      Serial.print(leitura);    


      Serial.println();
    }
  }


