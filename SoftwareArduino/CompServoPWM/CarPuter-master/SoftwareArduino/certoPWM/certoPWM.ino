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
  int32_t frequency = 50;
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
      velocityMedia = throttleMedia = 0;
  
      for (int j = 0; j < 100; j++)  {
          throttle = analogRead(A1);
          velocity = analogRead(A0);
          // Mapeia o valor do potenciômetro de 0-1023 para os ângulos de 0-180 
          velocity = map(velocity, 100, 1000, 1, 25);
          throttle = map(throttle, 0, 1023, 1, 25);
          velocityMedia += velocity;
          throttleMedia += throttle;
      }
  
      velocity = velocityMedia / 100;
      throttle = throttleMedia / 100;
  
      // Chama função que verifica quando o sistema deve entrar em ação
      IA();
      pid = PID(); 
                      
      // Verifica se o PID será usado
      if (flagLight) {
        
        // Soma o valor do ângulo atual do servo com a correção do PID    
        if (flagAtivaPID)  
          servo += pid;      
  
        // Define os limites de rotação do servo
        if (servo > LIMTOPSERVO)
          servo = LIMTOPSERVO;
        if (servo < LIMDOWNSERVO)
          servo = LIMDOWNSERVO;
  
        // Zera a soma dos erros integrativos do PID caso a leitura do potenciômetro esteja fora da tolerância
        if ((velocity > (targetVelocity + TOLKI)) || (velocity < (targetVelocity - TOLKI))) 
          sum = 0;
    
        // Define um intervalo máximo para a soma dos erros integrativos
        if (sum > SUMMAX)
          sum = SUMMAX;
        if (sum < -SUMMAX)
          sum = -SUMMAX;
          
        // Ajusta o valor do servo para inteiro
        pwm = (int)servo;
  
      } else {
        // Define a velocidade
        pwm = throttle;
      }
  
      // Define a posição do ângulo do servo
      pwmWrite(pinServo, pwm);
  
      delay(100);  
  
  // Debugging...  
  
      Serial.print(throttle);
      Serial.print("\t");
      Serial.print(targetVelocity);
      Serial.print("\t");
      Serial.print(velocity);      
      Serial.print("\t");
      Serial.print(velocityFaixa);      
      Serial.print("\t");
      Serial.print(pwm);
      Serial.print("\t");
      Serial.print(sum);
      Serial.print("\t");
      Serial.print(delta);
      Serial.print("\t");
      Serial.print(breaker);
      Serial.print("\t");
      Serial.print(countInteractions);
      Serial.print("\t");
      Serial.print(flagPID);
      Serial.print("\t");
      Serial.print(flagLight);
      Serial.print("\t");
      Serial.println();
    }
  }


  void IA() {    
    if (!flagPID) {
      //flagPid = false
      if ((velocity < (velocityFaixa + FAIXA)) && (velocity > (velocityFaixa - FAIXA))) {
        countInteractions++;
      } else {
        countInteractions = 0;
        velocityFaixa = velocity;
      }

      if (countInteractions > INTERACTIONS) {
        digitalWrite(LED, HIGH);
        flagLight = true;
        targetVelocity = velocity;
        servo = pwm;        
      }

      if (flagLight) {
        if (count > 5) 
          flagAtivaPID = true;
        else
          sum = 0;
        count++;
        if (count < RESPONSETIME) {
          if (throttle < 3) {
            flagPID = true;
          }            
        } else {
          digitalWrite(LED, LOW);
          flagLight = false;
          countInteractions = 0;
          count = 0;
        }
      }
    } else {
      //flagPid = true
      if (throttle > targetVelocity) {
        flagPID = false;
        flagAtivaPID = false;
        digitalWrite(LED, LOW);
        flagLight = false;
        countInteractions = 0;
        count = 0;
        velocityFaixa = velocity;
      }
    }
  }

  float PID() {
    float P, I, D, B, myPID, dt, error;
  
    // Cáculo do erro associado as medições
    error = targetVelocity - velocity;

    // Cálculo do delta de tempo
    dt = (millis() - lastProcess) / 1000.0;

    // Cálculo da componente proporcional
    P = kp * error;

    // Cálculo da componente integrativa
    sum += (error * dt);
    I = ki * sum;

    // Cálculo da componente derivativa
    // Para evitar divisão por 0 confere-se o dt > 0 e se o lastError é o definido pela primeira vez
    if ((dt > 0) && (lastError != LASTERRORINI)) {
      delta = ((error - lastError) / dt);
      D = kd * delta;
    }
    else 
      D = delta = 0;

    // Define que o freio não deve ser usado no caso de os sinais da última leitura e da atual forem diferentes(quando a curva muda de sinal), ou se a atual for 0(chegou no setpoint)
    if (((error > 0) && (lastError < 0)) || ((error < 0) && (lastError > 0)) || (error == 0) || (abs(delta) < LIMDELTA))
      flagBreak = false;

    // Define o uso do freio caso o setpoint mudar
    if ((targetVelocity != lastTargetVelocity))
      flagBreak = true;

    if ((targetVelocity - lastTargetVelocity) > 0)
      goingUp = true;
    if ((targetVelocity - lastTargetVelocity) < 0)
      goingUp = false;


    // Atualiza o último setpoint
    lastTargetVelocity = targetVelocity;

    // Caso o freio esteja definido pra ser usado e o setpoint não tenha sido alcançado
    if ((flagBreak) && (error != 0))
      if (goingUp) breaker = -1 * abs(delta / (error * error));
      else breaker = abs(delta / (error * error));
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
    
    return myPID;
  }
