// https://en.wikipedia.org/wiki/PID_controller
// Pot=175  ==>  Servo=147
// Pot=060  ==>  Servo=20

  #include <Servo.h> 
  #include <SoftwareSerial.h>

  //SoftwareSerial mySerial(7, 8); // Cria a entidade serial para controlar o arduino nas portas 7 e 8 (RX, TX)
  Servo myServo;  // Cria a entidade que controla o servo
  
  float lastProcess = 0;
  float lastError = 100000;
  float kp = 0.10, ki = 0.001, kd = 0.0, kf = 0.2;
  float servo = 55, pid;
  int targetAcel = 70, lasttargetAcel = 70; 
  int throttle = 0, throttleMedia = 0;
  int pwm = 0, FlagFreio = 0;
  int tol = 1;
  int tolKi = 15, sumMax = 5;
  float Freio = 0, Delta = 0, sum = 0;
  int taxaDelay = 1;
  
  void setup() {
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
      // Lê o valor alvo da serial
      if (Serial.available() > 0) {
          delay(10);
          targetAcel = (Serial.read() - '0') * 100 + (Serial.read() - '0') * 10 + (Serial.read() - '0');
          
          if (targetAcel < 0) 
              targetAcel = 0;
              
          if (targetAcel > 180) 
              targetAcel = 180;      
      }
  
      //getThrottle();  
      // Lê o valor do potenciômetro
      throttleMedia = 0;
      for(int k=0;k<100;k++)  { 
          throttle = analogRead(A0);
          // Mapeia o valor do potenciômetro de 0-1023 para os ângulos de 0-180 
          throttle = map(throttle, 0, 1023, 0, 179); 
          throttleMedia += throttle;
      }  
      throttle = throttleMedia / 100.0;

      // Soma o valor do ângulo atual do servo com a correção do PID
      pid = PID();
      servo += pid;
      if (servo > 167) servo = 167;
      if (servo < 35) servo = 35;



      // Zera a soma dos erros integrativos do PID caso a leitura do potenciômetro esteja fora da tolerância
      if ((throttle > (targetAcel + tolKi)) || (throttle < (targetAcel - tolKi))) sum = 0;
  
      // Define um intervalo máximo para a soma dos erros integrativos
      if (sum > sumMax) sum = sumMax;
      if (sum < -sumMax) sum = -sumMax;
      
      /*
      // Tenta recuperar a comunicação Bluetooth aumentando o delay
      if (throttle != -1) taxaDelay = 5;      
      else taxaDelay = 6;
      */

      // Ajusta o valor do PID para o delay atual
      pwm = (int)servo;// * taxaDelay;
//pwm = 145;
      // Define a posição do ângulo do servo
      myServo.write(pwm);                  

      delay(1);//5 * taxaDelay);  
  
// Debugging...  

      //Serial.print("   targetAcel: ");
      Serial.print(targetAcel);
      Serial.print("\t");
      //Serial.print("   throttle: ");
      Serial.print(throttle);
      Serial.print("\t");
      Serial.print(pid);
      Serial.print("\t");
      Serial.print(pwm);
      Serial.print("\t");
      Serial.print(sum);
      Serial.print("\t");
      Serial.print(Delta);
      Serial.print("\t");
      Serial.print(Freio);
      Serial.println();
  }


  float PID() {
      float P, I, D, F, myPID, dt, error;
    
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
      if ((dt > 0)&&(lastError!=100000))  {
        Delta = ((error - lastError) / dt);
        D = kd * Delta;
      }
      else 
        D = Delta = 0;

      if(((error > 0) && (lastError < 0)) || ((error < 0) && (lastError > 0)) || (error == 0))    FlagFreio = 1;
      if((targetAcel != lasttargetAcel)) FlagFreio =0;
      lasttargetAcel = targetAcel;
      if((FlagFreio == 0)&&(error != 0))  {
          if(error > 0)  Freio = Delta/(abs(error));
          else           Freio = -Delta/error;
      }
      else
         Freio = 0;

      F = kf * Freio;
         
      if(F > 15) F = 15;
      if(F < -15) F = -15;      
         
      
      // Atualização dos valores do último erro e tempo
      lastError = error;
      lastProcess  = millis();

      // Resultado do PID
      myPID = P + I + D + F;
      
      //Serial.print(myPID);
      //Serial.println();
      
      return myPID;
  }

