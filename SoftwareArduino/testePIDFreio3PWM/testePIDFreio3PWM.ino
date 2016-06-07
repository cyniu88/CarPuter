// https://en.wikipedia.org/wiki/PID_controller
// Pot=175  ==>  Servo=147
// Pot=060  ==>  Servo=20

  #include <PWM.h>
  #include <SoftwareSerial.h>
  
  #define LASTERRORINI 100000
  //#define LIMTOP -10 bom subida
  //#define LIMDOWN 15
  #define LIMTOP -10
  #define LIMDOWN 5
  #define SUMMAX 5
  #define TOL 2
  #define TOLKI 15

  float lastProcess = 0;
  float lastError = LASTERRORINI;
  float kp = 0.1, ki = 0.001, kd = 0.003, kb = 5;
  float servo = 55;
  float breaker = 0, delta = 0, sum = 0;
  int targetAcel = 70, lasttargetAcel = 70, pwm = 0; 
  int throttle = 0, throttleMedia = 0;
  boolean flagBreak = true;
  int32_t frequency = 150;
  bool success = false;
  int pinServo = 3;
  void setup() {
      InitTimersSafe();
      success = SetPinFrequencySafe(pinServo, frequency);
      // Inicializa as entidades seriais com a mesma velocidade de comunicação
      Serial.begin(38400);    
      //mySerial.begin(38400);
      // Limpa qualquer dado que estiver nas seriais
      //mySerial.flush(); 
      Serial.flush();
      // Define o pino 9 para o PWM(?) do servo

      // Inicializa a váriavel de tempo
      lastProcess = millis(); 
  
  }
  
  void loop() { 
   if (success) { 
      // Lê o valor alvo da serial
      if (Serial.available() > 0) {
          delay(10);
          targetAcel = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
          
          if (targetAcel < 0) 
              targetAcel = 0;
              
          if (targetAcel > 180) 
              targetAcel = 180;      
      }
  

      // Lê o valor do potenciômetro usando a média das leituras para evitar ruídos
      throttleMedia = 0;
      
      for (int j = 0; j < 100; j++)  { 
          throttle = analogRead(A0);
          // Mapeia o valor do potenciômetro de 0-1023 para os ângulos de 0-180 
          throttle = map(throttle, 0, 1023, 0, 179); 
          throttleMedia += throttle;
      }  
      throttle = throttleMedia / 100.0;

      // Soma o valor do ângulo atual do servo com a correção do PID
      servo += PID();

      // Define os limites de rotação do servo
      if (servo > 167)
        servo = 167;
      if (servo < 35)
        servo = 35;

      // Zera a soma dos erros integrativos do PID caso a leitura do potenciômetro esteja fora da tolerância
      if ((throttle > (targetAcel + TOLKI)) || (throttle < (targetAcel - TOLKI))) 
        sum = 0;
  
      // Define um intervalo máximo para a soma dos erros integrativos
      if (sum > SUMMAX)
        sum = SUMMAX;
      if (sum < -SUMMAX)
        sum = -SUMMAX;

      // Ajusta o valor do servo para inteiro
      pwm = (int)servo;

      // Define a posição do ângulo do servo
      pwmWrite(pinServo, pwm);                  

      delay(1);  
  
// Debugging...  
      Serial.print(targetAcel);
      Serial.print("\t");
      Serial.print(throttle);
      Serial.print("\t");
      Serial.print(pwm);
      Serial.print("\t");
      Serial.print(sum);
      Serial.print("\t");
      Serial.print(delta);
      Serial.print("\t");
      Serial.print(breaker);
      Serial.print("\t");
      //Serial.println();
   }
  }


  float PID() {
      float P, I, D, B, myPID, dt, error;
    
      // Cáculo do erro associado as medições
      error = targetAcel - throttle;

      // Cálculo do delta de tempo
      dt = (millis() - lastProcess) / 1000.0;

      // Cálculo da componente proporcional
      P = kp * error;

      // Cálculo da componente integrativa
      sum += (error * dt);
      I = ki * sum;

      // Cálculo da componente derivativa
      // Para evitar divisão por 0 confere-se o dt > 0 e se o lastError é o definido pela primeira vez
      if ((dt > 0) && (lastError != 100000)) {
        delta = ((error - lastError) / dt);
        D = kd * delta;
      }
      else 
        D = delta = 0;

      // Define que o freio não deve ser usado no caso de os sinais da última leitura e da atual forem diferentes(quando a curva muda de sinal), ou se a atual for 0(chegou no setpoint)
      if (((error > 0) && (lastError < 0)) || ((error < 0) && (lastError > 0)) || (error == 0))
        flagBreak = false;

      // Define o uso do freio caso o setpoint mudar
      if ((targetAcel != lasttargetAcel))
        flagBreak = true;

      // Atualiza o último setpoint
      lasttargetAcel = targetAcel;

      // Caso o freio esteja definido pra ser usado e o setpoint não tenha sido alcançado
      if ((flagBreak) && (error != 0))
        breaker = delta / (error * error);
      else
        breaker = 0;

      // Cálculo da componente do freio
      B = kb * breaker;

      // Define limites de atuação do freio
      if(B > LIMDOWN)
        B = LIMDOWN;
        
      if(B < LIMTOP)
        B = LIMTOP;       
      
      // Atualização dos valores do último erro e tempo
      lastError = error;
      lastProcess  = millis();

      // Resultado do PID
      myPID = P + I + D + B;

      Serial.print(B);
      Serial.print("\t");
      Serial.print(error);
      Serial.print("\t");
      Serial.print(flagBreak);
      Serial.println();
      
      return myPID;
  }
