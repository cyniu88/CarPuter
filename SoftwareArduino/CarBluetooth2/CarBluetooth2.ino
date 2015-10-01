/*
Aplicar um delay de 800ms após um comando para garantir que o próximo será executado
O comando AT disconecta o bluetooth case ele esteja conectado
Limpar o buffer serial antes de checar mensagem >> mySerial.flsuh()


MASTER mac address: 78A5048C378C
YELLOW mac address: 78A5048C47AF

Orange -> 78A5048C479F
Red -> 78A5048C37EA


*/

#include <SoftwareSerial.h>



SoftwareSerial mySerial(7, 8); // RX, TX
const int botao = 9;     // the number of the pushbutton pin
int ROLE = 1;    //variavel que armazena o estado do bluetooth
//char string check[20];  //string para armazenar mensagens de confirmacao
String check = "";
String MAC = "";
char c;
int flag = 0;
int i = 0;
int cont = 0;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  while (!Serial) {
  // initialize the pushbutton pin as an input:
  pinMode(botao, INPUT);
}

  // set the data rate for the SoftwareSerial port
  mySerial.begin(38400);
  delay(800);
  mySerial.flush(); 
  Serial.flush(); 
}

void loop() // run over and over
{
if (ROLE == 1)
   { 
    mySerial.flush();
    mySerial.write("AT\r\n");  //mantem o bluetooth em modo de slave, para que outros possam se conectar a ele
    delay(500);
    while (mySerial.available())
       Serial.write(mySerial.read());

    mySerial.write("AT+ROLE0\r\n");  //mantem o bluetooth em modo de slave, para que outros possam se conectar a ele

    delay(500);
    while (mySerial.available())
       Serial.write(mySerial.read());
    Serial.write("...tentou\n");
    ROLE = 0;
   }

if (digitalRead(botao) == 1)  //verifica o estado do botao e executa se ele estiver pressionado
//          {
//          Serial.write("\nBotao = 1\n");  //avisa que a rotina foi iniciada
//          mySerial.write("AT+ADDR?");  //mantem o bluetooth em modo de slave, para que outros possam se conectar a ele
//          delay(200);
//          while (mySerial.available())
//            Serial.write(mySerial.read());
//          Serial.write("\n....Botao = 0\n");  //avisa que a rotina foi iniciada
//          }


          {
          Serial.write("\nBotao = 1\n");  //avisa que a rotina foi iniciada
          
          
          //bloco de conexao
          MAC = "78A5048C479F";
          cont = 0;
          while(cont < 5)
            {
              if (connect(MAC) == 0) 
              {
                Serial.print("conectou\n");
                ROLE = 1;
                cont=0;
              }
          else 
              {
                Serial.print("erro\n");
                ROLE = 0;  //variavel de controle (o arduino nao sabe se o modulo esta como master ou slave)          
                cont++;
              }
            }
          }
          
if (mySerial.available())
  {  
    delay(800);  
    while (mySerial.available() > 0)  //transfere o buffer do mySerial para a string check
          {
              c = mySerial.read();
              check += c;
              flag = 1;
          }
    if(flag==1) {
      Serial.print(check);
      Serial.write("\n");
      check.remove(0);
      flag = 0;
    }
    //if (check == "OK+CONNAOK+CONN")    //verifica se a conexao foi efetuada 
  }
  
if (Serial.available())
  {  
    delay(800);  
    while (Serial.available() > 0)  //transfere o buffer do mySerial para a string check
          {
              c = Serial.read();
              check += c;
              flag = 1;
          }
    
      if(flag==1) {
        Serial.print(check);
        Serial.write("\n");
        mySerial.flush();
        check += "\r\n";
        mySerial.print(check);
        check.remove(0);
        flag = 0;
      }
    //if (check == "OK+CONNAOK+CONN")    //verifica se a conexao foi efetuada 
  }








         
         //bloco de transmissao
         
//          mySerial.write("teste");
//          delay(800);
//          mySerial.write("AT");  //desconecta os modulos
//          delay(800);
//          while (mySerial.available())  //printa tudo que esta no bufffer
//              Serial.write(mySerial.read());
//          Serial.write("\n");
         

//      if (mySerial.available() == "teste")
//            Serial.write("entendi");
    
  //loop para passar as mensagens entre o serial virtual e o TX RX (serve para permitir que voce mande comandos ao modulo pelo terminal)    
  
/*  
  
  if (mySerial.available())
    { //delay(800);
      //while (mySerial.available())
        Serial.write(mySerial.read());
    }
delay(500);
  if (Serial.available())
    { //delay(800);
      //while (Serial.available())
        mySerial.write(Serial.read());
    }
delay(500);  
*/  
  
//  while (mySerial.available())
//      Serial.write(mySerial.read());
//  delay (800);
//  while (Serial.available())
//      mySerial.write(Serial.read());
//  delay (800);
}


int connect(String MAC)
{
          mySerial.write("AT+ROLE1");  //coloca o modulo em modo master para se conectar a outro 
          delay(500);
          mySerial.flush();
          mySerial.print("AT+CON");  //conecta com o modulo especificado pelo mac
          mySerial.print(MAC);
          delay(500);
          while (mySerial.available() > 0)  //transfere o buffer do mySerial para a string check
          {
              c = mySerial.read();
              check += c;
          }
          if (check == "OK+CONNAOK+CONN")    //verifica se a conexao foi efetuada 
          {
          check.remove(0);
          return (0);
          }
          else    //caso a conexao tenha falhado, tenta novamente ate 5 vezes
          {
              check.remove(0);
              return (1);
          }
}
