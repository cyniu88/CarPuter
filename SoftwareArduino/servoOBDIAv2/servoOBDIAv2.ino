// Pot=175  ==>  Servo=147
// Pot=060  ==>  Servo=20

#include <SoftwareSerial.h>
#include <Servo.h>

// Define pino 13 para o LED
#define LED 13
// Define um valor inicial para o erro
#define LASTERRORINI 100000
// Define o limite do freio na subida
#define LIMTOP -10
// Define o limite do freio na descida
#define LIMDOWN 5
// Define o limite do delta
#define LIMDELTA 100
// Define o limite da soma
#define SUMMAX 10
// Define o número de interações para cálculo do PID
#define INTERACTIONS 10
// Define o número de interações para resposta do motorista
#define RESPONSETIME 5
// Define a tolerância do valor da velocidade em relação ao target
#define TOL 1
// Define a faixa para manter a velocidade constante
#define FAIXA 5
// Define a tolerância da velocidade em relação ao target para o acúmulo da soma da componente integrativa do PID
#define TOLKI 15

#define RxD 7                    // Pino do Arduino conectado no Tx do HC-05
#define TxD 8                    // Pino do Arduino conectado no Rx do HC-05
#define servoPin 9               // Pino do Arduino conectado no sinal do servo
#define Reset 10                 // Pino do Arduino conectado no Reset do HC-05 (reset com LOW)
#define PIO11 A2                 // Pino do Arduino conectado no PI011 do HC-05 (entrar no AT Mode com HIGH)
#define BT_CMD_RETRIES 5         // Número de tentativas para cada comando AT do Bluetooth no caso de não responder OK
#define OBD_CMD_RETRIES 3        // Número de tentativas para cada comando OBD no caso de não responder com o carácter '>'
#define RPM_CMD_RETRIES 5        // Número de tentativas para o comando de obter RPM
#define THROTTLE_CMD_RETRIES 5   // Número de tentativas para o comando de obter o THROTTLE

boolean bt_error_flag;           // Váriavel para o erro de comunicação com o Bluetooth
boolean obd_error_flag;          // Váriavel para o erro de comunicação com o ELM-327

int rpm = 0;                     // Váriavel para o valor do RPM
boolean rpm_error_flag;          // Váriavel para erro do RPM
boolean rpm_retries = 0;         // Váriavel para o número de tentativas do comando de RPM

boolean throttle_error_flag;     // Váriavel para erro do THROTTLE
boolean throttle_retries = 0;    // Váriavel para o número de tentativas do comando do THROTTLE
int throttle = 0;                // Váriavel para o valor do THROTTLE




// Variável para armazenar o valor do último tempo computado
float lastProcess = 0;
// Variável para armazenar o valor do último erro computado
float lastError = LASTERRORINI;
// Variáveis para armazenar os valores das contantes do PID(B)
float kp = 0.06, ki = 0.003, kd = 0.0, kb = 0.3;
// Variável para acumular a soma da componente integrativa do PID
float sum = 0;
// Variáveis para armazenar os valores que serão passados ao servo
float servo = 35, pid = 0;
int pwm = 0;
// Variáveis para leitura do target da velocidade
int targetVelocity = 100, lastTargetVelocity = 100;
// Variáveis para leitura da velocidade
int velocityBand = 100, velocity = 0, velocityMedia = 0;
// Variáveis para leitura da posição do cabo do acelerador
int throttleMedia = 0;
// Contadores de interações 
int countInteractions = 0, count = 0;

// Flags de controle
boolean goingUp = true; // Flag para definir se a velocidade está subindo ou descendo
boolean flagLight = false; // Flag para acionar o LED
boolean flagBreak = true; // Flag para acionar o uso do freio
boolean flagPID = false; // Flag para acionar o cálculo do PID  
boolean flagActivatePID = false; // Flag para ativar o uso do PID




SoftwareSerial blueToothSerial(RxD,TxD);     // Cria a entidade serial do Bluetooth
Servo myServo;                               // Cria a entidade que controla o servo

//-----------------------------------------------------//
//--------------Calcula o a correção PID---------------//
//-----------------------------------------------------//
float PID() {
  // Variáveis para uso no cáculo do PID
  float P, I, D, B, myPID, dt, error;
  float breaker = 0, delta = 0;

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
    // Define o sinal oposto a direção da curva para freá-la
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



void IA() {
  // Caso o cálculo do PID não esteja habillitado
  if (!flagPID) {
    // Verifica se a velocidade está dentro de uma faixa constante
    if ((velocity < (velocityBand + FAIXA)) && (velocity > (velocityBand - FAIXA))) {
      // Incrementa as interações
      countInteractions++;
    } else {
      // Caso a velocidade saia da faixa constante, redefine-se a faixa e zera as interações
      countInteractions = 0;
      velocityBand = velocity;
    }
    // Caso o motorista mantenha a faixa constante de velocidade por 300 interações
    if (countInteractions > INTERACTIONS) {
      // Liga-se o LED
      digitalWrite(LED, HIGH);
      flagLight = true;
      // Redefine-se o target da velocidade
      targetVelocity = velocity;
      // Define o valor do servo com o último valor enviad ao servo
      servo = pwm;        
    }
    // Caso o LED esteja aceso
    if (flagLight) {
      // Espera 5 interações para ativar o uso do PID
      if (count > 3) { 
        flagActivatePID = true;
        flagPID = true;
      } else {
        // Caso contrário, zera a soma do fator integrativo, pois o erro acumulado pode ser muito grande e atrapalhar a primeira correção do PID
        sum = 0;
      }
      // Incrementa a contagem 
      count++;
    }
  } else {
    // Caso o cáculo do PID esteja habilitado e o motorista ultrapassar o target da velocidade
    if (throttle > targetVelocity) {
      // Desativa o cálculo e o uso do PID, apaga o LED, zera os contadores e redefine a faixa de velocidae
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


//----------------------------------------------------------//
//----------------Inicialização do OBDII--------------------//
//----------------------------------------------------------//
void obd_init() {
   // Define a flag de erro do OBD como FALSE
   obd_error_flag = false;
   // Envia comando para resetar o OBD
   send_OBD_cmd("ATZ");
   delay(1000);
   // Envia comando para definir o protocolo correto
   send_OBD_cmd("ATSP5");
   delay(1000);
   // Envia o comando para obter o THROTTLE
   send_OBD_cmd("01111");
   delay(1000);
}

//----------------------------------------------------------//
//-----------start of bluetooth connection------------------//
void setupBlueToothConnection() {
   // Define a flag de erro do OBD como FALSE
   bt_error_flag = false;
   // Entra no modo AT do HC-05
   enterATMode();
   delay(500);
   // Reseta o HC-05
   sendATCommand("RESET");
   delay(1000);
   // Reseta para a configuração de fábrica
   sendATCommand("ORGL");
   // Define função como MASTER
   sendATCommand("ROLE=1");
   // Define conexão para o modo de endereço específico
   sendATCommand("CMODE=0");
   // Vincula o HC-05 ao endereço do Bluetooth do ELM
   sendATCommand("BIND=8818,56,6898EB,10");
   // Inicia a conexão
   sendATCommand("INIT");
   delay(1000);
   // Pareia o HC-05 com o ELM
   sendATCommand("PAIR=8818,56,6898EB,10,20");
   delay(1000);
   // Linka o HC-05 ao endereço do ELM
   sendATCommand("LINK=8818,56,6898EB,10");
   delay(1000);
   // Entra no modo de comando
   enterComMode();
   delay(500);
}

//----------------------------------------------------------//
//------------------Reseta o HC-05--------------------------//
//-------Seta o pino de reset do HC-05 para LOW por 2s------//
//----------------------------------------------------------//
void resetBT() {
   digitalWrite(Reset, LOW);
   delay(2000);
   digitalWrite(Reset, HIGH);
}

//----------------------------------------------------------//
//-----------Entra no modo de comunicação do HC-05----------//
//----------Seta o pino PIO11 do HC-05 para LOW-------------//
//----------------------------------------------------------//
void enterComMode() {
   // Limpa os dados que estiverem na serial do Bluetooth
   blueToothSerial.flush();
   delay(500);
   //digitalWrite(PIO11, LOW);
   //delay(500);
   // Define a taxa de comunicação padrão do HC-05
   blueToothSerial.begin(38400);
}

//----------------------------------------------------------//
//---------------Entra no modo AT do HC-05------------------//
//----------Seta o pino PIO11 do HC-05 para HIGH------------//
//----------------------------------------------------------//
void enterATMode() {
   // Limpa os dados que estiverem na serial do Bluetooth
   blueToothSerial.flush();
   delay(500);
   //digitalWrite(PIO11, HIGH);
   //delay(500);
   // Define a taxa de comunicação padrão do HC-05
   blueToothSerial.begin(38400);
}

//----------------------------------------------------------//
//---------------------Envia comando OBD--------------------//
//----------------------------------------------------------//
void send_OBD_cmd(char *obd_cmd) {
   char recvChar;
   boolean prompt;
   int retries;
   // Se não houver erro com a conexão do OBDII
   if (!(obd_error_flag)) {
      prompt = false;
      retries = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while((!prompt) && (retries < OBD_CMD_RETRIES)) {
         // Envia o comando OBD
         blueToothSerial.print(obd_cmd);
         blueToothSerial.print("\r");
         // Espera até receber algum dado do ELM
         while (blueToothSerial.available() <= 0);
         // Enquanto há dados mas não está no prompt
         while ((blueToothSerial.available() > 0) && (!prompt)) {
            // Lê os dados do ELM
            recvChar = blueToothSerial.read();
            // Se o carácter for o '>', então prompt é TRUE
            if (recvChar == 62) prompt = true;
         }
         // Incrementa as tentativas e espera 2s
         retries += 1;
         delay(2000);
      }
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro
      if (retries >= OBD_CMD_RETRIES) obd_error_flag = true;
   }
}

//----------------------------------------------------------//
//---------------Envia comando AT para o HC-05--------------//
//----------------------------------------------------------//
void sendATCommand(char *command) {
   char recvChar;
   char str[2];
   int i, retries;
   boolean OK_flag;
   // Se não houver erro com a conexão do Bluetooth
   if (!(bt_error_flag)) {
      retries = 0;
      OK_flag = false;
      Serial.print("entrou no if;");
      // Enquanto não receber um OK e as tentativas forem menores do que o máximo
      while ((retries < BT_CMD_RETRIES) && (!(OK_flag))) {
        Serial.print("entrou no while;");
         // Envia o comando AT para o HC-05
         blueToothSerial.print("AT");
         if (strlen(command) > 1) {
            // Concatena o '+' e o comando enviado como argumento
            blueToothSerial.print("+");
            blueToothSerial.print(command);
         }
         blueToothSerial.print("\n");
         Serial.print("mandou td");
         // Espera até receber algum dado
         while (blueToothSerial.available() <= 0);
         i = 0;
         // Enquanto há dados
         while (blueToothSerial.available() > 0) {
          Serial.print("mrecebeu?");
            // Lê do HC-05
            recvChar = blueToothSerial.read();
            if (i < 2) {
               // Monta a string
               str[i] = recvChar;
               i += 1;
            }
         }
         // Incrementa as tentativas e espera 2s
         retries += 1;
         Serial.print("tenta d novo;");
         // Se a string for OK, seta a flag de OK para TRUE
         if ((str[0] == 'O') && (str[1] == 'K')) OK_flag = true;
         delay(1000);
      }
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro
      if (retries >= BT_CMD_RETRIES) bt_error_flag = true;
   }
}

//-----------------------------------------------------//
//----------Recebe o valor do RPM do OBD---------------//
//---------Converte para um valor decimal--------------//
//-----------------------------------------------------//
void rpm_calc() {
   boolean prompt, valid;  
   char recvChar;
   char bufin[15];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!(obd_error_flag)) {
      valid = false;
      prompt = false;
      // Envia o comando 010C para obter o RPM, o 1 final é para indicar que o ELM deve esperar apenas uma resposta
      blueToothSerial.print("010C1");
      blueToothSerial.print("\r");
      // Espera até receber algum dado do ELM
      while (blueToothSerial.available() <= 0);
      i = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while ((blueToothSerial.available() > 0) && (!prompt)) {
         // Lê os dados do ELM
         recvChar = blueToothSerial.read();
         // A resposta normal para o comando é 010C1/r41 0C ?? ??>, então conta 15 carácteres e ignora o carácter 32 que é o espaço
         if ((i < 15) && (!(recvChar == 32))) {
            // Monta a string
            bufin[i] = recvChar;
            i += 1;
         }
         // Se o carácter 62 recebido for o '>', então está no prompt e a resposta do ELM está pronta  
         if (recvChar == 62) prompt = true;
      }
      // Se os 4 primeiros caractéres após o comando forem 410C
      if ((bufin[6] == '4') && (bufin[7] == '1') && (bufin[8] == '0') && (bufin[9] == 'C')) {
         // Então a resposta do RPM é válida
         valid = true;
      } else {
         valid = false;
      }
      // No caso da resposta válida
      if (valid) {
         // Zera as tentaivas e define a flag de erro como FALSE
         rpm_retries = 0;
         rpm_error_flag = false;

         // Para iniciar o cálculo do valor real do RPM
         // É necessário separar os dois bytes hexadecimais da resposta do OBD, por examplo A=0B e B=6C
         // A equação é ((A * 256) + B) / 4
         rpm = 0;
         // Entre os carácteres 10 e 14 do bufin está o valor do RPM
         for (i = 10; i < 14; i++) {
            // Se o carácter estiver entre 'A' e 'F'
            if ((bufin[i] >= 'A') && (bufin[i] <= 'F')) {
               // Subtraindo 55 temos o valor decimal do byte
               bufin[i] -= 55;
            } 
            // Se o carácter estiver entre '0' e '9'
            if ((bufin[i] >= '0') && (bufin[i] <= '9')) {
               // Subtraindo 48 temos o valor decimal do byte
               bufin[i] -= 48;
            }
            // Shifitando para a esquerda 4 bits e adicionando os 4 bits do novo carácter, temos rpm = (A * 256) + B
            rpm = (rpm << 4) | (bufin[i] & 0xf);
         }
         // Shifitando para a direita 2 bits, temos rpm = rpm/4
         rpm = rpm >> 2;
      }
   }
   // No caso de um valor inválido do RPM
   if (!valid) {
      // Seta a flag de erro como TRUE
      rpm_error_flag = true;
      // Incrementa uma tentativa
      rpm_retries += 1;
      // E zera o valor do RPM
      rpm = 0;
      // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
      if (rpm_retries >= RPM_CMD_RETRIES) obd_error_flag = true;
   }
   // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
   if ((obd_error_flag == true)) {
      rpm = 0;
      rpm_retries = 0;
   }
}

//-----------------------------------------------------//
//--------Recebe o valor do THROTTLE do OBD------------//
//---------Converte para um valor decimal--------------//
//-----------------------------------------------------//
void throttle_calc() {
   boolean prompt, valid;  
   char recvChar;
   char bufin[12];
   int i;
   // Se não houver erro com a conexão do OBDII
   if (!(obd_error_flag)) {
      valid = false;
      prompt = false;
      // Envia o comando 010C para obter o THROTTLE
      blueToothSerial.print("0111");
      blueToothSerial.print("\r");
      // Espera até receber algum dado do ELM
      while (blueToothSerial.available() <= 0);
      i = 0;
      // Enquanto não estiver no prompt OBD e as tentativas forem menores do que o máximo
      while ((blueToothSerial.available() > 0) && (!prompt)) {
         // Lê os dados do ELM
         recvChar = blueToothSerial.read();
         // A resposta normal para o comando é 0111/r41 11 ??>, então conta 12 carácteres e ignora o carácter 32 que é o espaço
         if ((i < 12) && (!(recvChar == 32))) {
            // Monta a string
            bufin[i] = recvChar;
            i += 1;
         }
         // Se o carácter 62 recebido for o '>', então está no prompt e a resposta do ELM está pronta  
         if (recvChar == 62) prompt = true;
      }
      // Se os 4 primeiros carácteres após o comando forem 4111
      if ((bufin[5] == '4') && (bufin[6] == '1') && (bufin[7] == '1') && (bufin[8] == '1')) {
         // Então a resposta do THROTTLE é válida
         valid=true;
      } else {
         valid=false;
      }
      // No caso da resposta válida
      if (valid) {
         // Zera as tentaivas e define a flag de erro como FALSE
         throttle_retries = 0;
         throttle_error_flag = false;
         // Para iniciar o cálculo do valor real do THROTTLE
         // É necessário separar o byte hexadecimais da resposta do OBD, por exemplo A=0B
         // A equação é (A / 255) * 100
         // Transforma os bytes em String
         String loadHex(bufin[9]); //11?
         String loadHex2(bufin[10]); //12?
         // Converte-os pra decimal com a função hexToDec
         String loadHexTotal = loadHex + loadHex2;
         int DecimalDecode = hexToDec(loadHexTotal);
         // Faz o cálculo de throttle = (A / 255) * 100
         throttle = round((float(DecimalDecode) / 255) * 100);
      }
      // No caso de um valor inválido do THROTTLE
      if (!valid) {
         // Seta a flag de erro como TRUE
         throttle_error_flag = true;
         // Incrementa uma tentativa
         throttle_retries += 1;
         // E zera o valor do THROTTLE
         throttle = 0;
         // Se atingiu o máximo de tentativas, define como TRUE a flag de erro do OBD
         if (throttle_retries >= THROTTLE_CMD_RETRIES) obd_error_flag = true;
      }
      // No caso de haver erro na comunicação OBD, zera o valor do rpm e das tentativas
      if ((obd_error_flag == true)) {
         throttle = 0;
         throttle_retries = 0;
      }
   }
}

//-----------------------------------------------------//
//----Converte um valor hexadecimal para decimal-------//
//-----------------------------------------------------//
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

//-----------------------------------------------------//
//-----Mostra o valor do RPM no monitor Serial---------//
//-----------------------------------------------------//
void rpm_show() {
   // Se não houver erro com a conexão do OBDII
   while (!(obd_error_flag)) {
      // Se o valor do RPM for entre 0 e 10000
      if ((rpm >= 0) && (rpm < 10000)) {
         // Exibe no monitor Serial
         Serial.print("RPM = ");
         Serial.print(rpm);
         Serial.println();
         // Caso haja algum erro, exibe a mensagem
         if (rpm_error_flag){
            Serial.print("Erro ao obter RPM");
         }
      } else {
         Serial.print("Valor incorreto do RPM");
      }
   }
}

//-----------------------------------------------------//
//----Mostra o valor do THROTTLE no monitor Serial-----//
//-----------------------------------------------------//
void throttle_show() {
   // Se não houver erro com a conexão do OBDII
   while (!(obd_error_flag)) {
      // Se o valor do THROTTLE for entre 0 e 10000
      if ((throttle >= 0) && (throttle < 10000)) {
         // Exibe no monitor Serial
         Serial.print("THROTTLE = ");
         Serial.print(throttle);
         Serial.println();
         // Caso haja algum erro, exibe a mensagem
         if (rpm_error_flag){
            Serial.print("Erro ao obter THROTTLE");
         }
      } else {
         Serial.print("Valor incorreto do THROTTLE");
      }
   }
}



//-----------------------------------------------------//
void setup() {
   // Define os pinos de entrada e saída
   pinMode(RxD, INPUT);
   pinMode(TxD, OUTPUT);
   /*
   pinMode(PIO11, OUTPUT);
   pinMode(Reset, OUTPUT);
   // Define o HC-05 para o COMMODE
   digitalWrite(PIO11, LOW);
   // Reseta o HC-05
   digitalWrite(Reset, HIGH);
   */
   // Inicializa a serial com a mesma velocidade de comunicação do Bluetooth
   Serial.begin(38400);
   // Limpa qualquer dado que estiver nas serial
   Serial.flush();
   // Define o pino 9 para o PWM(?) do servo
   //myServo.attach(servoPin); 
   // Inicializa a váriavel de tempo
   lastProcess = millis();
   
   // Inicia a conexão com o Bluetooth
   //setupBlueToothConnection();
   // No caso de haver erro na conexão com o Bluetooth
   if (bt_error_flag) {
      Serial.print("Erro de conexão com o Bluetooth");
   }
   // Inicializa a conexão OBDII
   //obd_init();
   // No caso de haver erro na conexão com o OBDII
   if (obd_error_flag) {
      Serial.print("Erro de conexão com o OBDII");
   }
   
}

//----------------------------------------------------------//
void loop(){
  // Lê o valor do potenciômetro usando a média das leituras para evitar ruídos
  /*velocityMedia = throttleMedia = 0;

  for (int j = 0; j < 100; j++) {
      // Lê os valores dos potenciômetros
      throttle = analogRead(A1);
      velocity = analogRead(A0);
      // Mapeia os valores dos potenciômetros de 0-1023 para os ângulos de 0-179 
      velocity = map(velocity, 240, 1000, 35, 179);
      throttle = map(throttle, 0, 1023, 0, 179);
      // Acumula os valores para tirar a média e assim diminuir ruídos
      velocityMedia += velocity;
      throttleMedia += throttle;
  }
  // Tira a média
  velocity = velocityMedia / 100;
  throttle = throttleMedia / 100;
                 
  */
  // Funções para testar com o carro
  throttle_calc();
  throttle_show();
  /*
  // Chama função que verifica quando o sistema deve entrar em ação
  IA();
  // Chama o cáculo do PID
  pid = PID(); 
  // Verifica se o PID será usado
  if (flagLight) {
          
    // Soma o valor do ângulo atual do servo com a correção do PID    
    if (flagActivatePID)  
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
    // Define que o servo deve seguir o controle manual
    pwm = throttle;
  }
  */

  // Tenta recuperar a comunicação do Bluetooth aumentando o delay
  //if (obd_error_flag) taxaDelay = 5;
  //else taxaDelay = 6;
  
   // Define a posição do ângulo do servo
  // myServo.write(pwm);
   delay(300);
/*
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
   Serial.println();*/
}
