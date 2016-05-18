// https://en.wikipedia.org/wiki/PID_controller
// Pot=175  ==>  Servo=147
// Pot=060  ==>  Servo=20

  #include <Servo.h> 
  #include <SoftwareSerial.h>
  
  #define LED 13
  #define LASTERRORINI 100000
  #define LIMTOP -10
  #define LIMDOWN 5
  #define LIMDELTA 100
  #define SUMMAX 10
  #define INTERACTIONS 200
  #define RESPONSETIME 100
  #define TOL 1
  #define FAIXA 5
  #define TOLKI 25

  float lastProcess = 0;
  float lastError = LASTERRORINI;
  float kp = 0.06, ki = 0.003, kd = 0.0, kb = 0.3;
  float servo = 55, pid;
  float breaker = 0, delta = 0, sum = 0;
  int targetVelocity = 70, lastTargetVelocity = 70, pwm = 0; 
  int velocityFaixa = 100, velocity = 0, velocityMedia = 0;
  int lastSensor = 0, throttle = 0, throttleMedia = 0;
  int countInteractions = 0, count = 0;
  boolean flagBreak = true, flagPID = false, goingUp = true, flagLight = false, flagAtivaPID = false;
  
  Servo myServo;  // Cria a entidade que controla o servo
  
  void setup() {
    // Define o LED como saída
    pinMode(LED, OUTPUT);  
    // Inicializa as entidades seriais com a mesma velocidade de comunicação
    Serial.begin(38400);  
    //mySerial.begin(38400);
    // Limpa qualquer dado que estiver nas seriais
    //mySerial.flush(); 
    Serial.flush();
    // Define o pino 9 para o PWM(?) do servo
    myServo.attach(9); 
    // Inicializa a váriavel de tempo
    lastProcess = millis(); 
  
  }
  
  void loop() {  
    // Lê o valor do potenciômetro usando a média das leituras para evitar ruídos
    velocityMedia = throttleMedia = 0;

    for (int j = 0; j < 100; j++)  {
        throttle = analogRead(A1);
        velocity = analogRead(A0);
        // Mapeia o valor do potenciômetro de 0-1023 para os ângulos de 0-180 
        velocity = map(velocity, 0, 1023, 0, 179);
        throttle = map(throttle, 0, 1023, 70, 170);
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
      if (servo > 167)
        servo = 167;
      if (servo < 35)
        servo = 35;

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
      pwm = map(throttle, 70, 170, 35, 147);
    }

    // Define a posição do ângulo do servo
    myServo.write(pwm);

    delay(1);  

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
        if (count > 1)
          flagAtivaPID = true;
        count++;
        if (count < RESPONSETIME) {
          if (throttle < 80) {
            flagPID = true;
            //servo = 40; //no caso do carro
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
    if ((dt > 0) && (lastError != 100000)) {
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
    myPID = P;// + I + D + B;
    
    return myPID;
  }
