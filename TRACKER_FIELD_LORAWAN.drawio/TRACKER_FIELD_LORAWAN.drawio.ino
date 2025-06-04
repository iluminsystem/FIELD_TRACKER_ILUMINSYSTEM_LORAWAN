/* =======================================================================
   iluminSystem!
   iluminsystem@gmail.com
   iluminSystem: https://www.iluminsystem.com.br
   Contato tel : 55 11 9-8833-5145

   Projeto: TRACKER FIELD LORAWAN iluminSystem
   uC: ESP32-WROOM-32E
   Board: TF1 LW Version 1.0
   Compilador: Arduino IDE 1.8.19
   Autor: Eng. LSS
   Data:  May de 2025
 
======================================================================= */
// =======================================================================
// --- Bibliotecas Auxiliares ---
#include <esp_task_wdt.h>                                                 //Inclui o WDT
#include <HardwareSerial.h>                                               //Bibliteca para Serial2 (GPIO16 e GPIO17)
#include <SSD1306.h>                                                      //Biblioteca para o display OLED SSD1306
#include <Wire.h>                                                         //Biblioteca para i2c
#include <TinyGPS++.h>                                                    //Biblioteca para GPS
#include "iluminSystem.h"                                                 //Arquivo HEXA para o Logo Iluminsystemy
SSD1306 display (0x3c, 21, 22);                                           //Endereço para display OLED
#include <EEPROM.h>                                                       //Biblioteca para utilização EEPROM
#define   EEPROM_SIZE 1                                                   //Biblioteca para memoria eeprom
#include <BH1750.h>                                                       //Biblioteca para luximetro
// =======================================================================
// --- Mapeamento de Hardware ---
#define   bt_esc    27                                                    //botão de esc
#define   bt_dec    26                                                    //botão de decremento '-'
#define   bt_inc    33                                                    //botão de incremento '+'
#define   bt_ent    32                                                    //botão enter
#define   RXD2      16
#define   TXD2      17
HardwareSerial neogps(1);
TinyGPSPlus gps;
BH1750 lightMeter;
// =======================================================================
// --- Constantes Auxiliares ---
#define M_MAX         6                                                  //número máximo de menus no projeto
String  RESET         = "";                                              //String para debug de função de RESET
String  textoRecebido = "";                                              //String para receber status JOIN rede LoRaWan
String  Join1         = "AT_JOIN_OK";                                    //String para comparar status JOIN "CONECTADO"
String  Join2         = "AT_JOIN_ERROR";                                 //String para comparar status JOIN "DESCONECTADO"
String  Join3         = "AT_ALREADY_JOINED";                             //String para comparar status JOIN "JÁ CONECTADO"
String  DEVEUI        = "";                                              //String para gravar DEVEUI
String  APPKEY        = "";                                              //String para gravar APPKEY
String  APPEUI        = "";                                              //String para gravar APPEUI
String  CHMASK        = "";                                              //String para gravar CHMASK
String  DATARATE      = "";                                              //String para gravar DATARATE
String  CLASSE        = "";                                              //String para gravar CLASSE
String  NJM           = "";                                              //String para gravar NJM
String  RESP          = "";
String  LAT           = "";
String  LNG           = "";
String  SPD           = "";
String  SAT           = "";
String  ALT           = "";
String  LUX           = "";
float   latitude      = 0.0;
float   longitude     = 0.0;
float   speeds        = 0.0;
float   satelite      = 0.0;
float   altitude      = 0.0;
int     pacote        = 0;
float   luxxi         = 0.0;
// =======================================================================
// --- Objetos ---

// =======================================================================
// --- Protótipo das Funções ---
void read_keyb();                                                         //leitura dos botões
void menu_sel();                                                          //controle de seleção de menus
void sub_menu();                                                          //ação nos sub-menus
void join();                                                              //executa join na rede lorawan
void sendData();                                                          //envia dados do field tracker para a rede lorawan
void gpsdata();                                                           //visualiza dados das coordenadas geografica do GPS
void gpsdataSEND();                                                           //visualiza dados das coordenadas geografica do GPS
void loradata();                                                          //visualiza dados das informações Lorawan do modulo
void reset();                                                             //realiza um reset no modulo lorawan
void read_lora();
void luximetro();
void luximetroSEND();
// =======================================================================
// --- Variáveis Globais ---
int              menu_num = 1;                                            //número para lista de menus                 
unsigned long    bl_time_save = 0,                                        //tempo para manter backlight ligado
                 bt_time      = 0;                                        //armazena o tempo de clique dos botões
bool             adj      = 0,                                            //indicador de ajuste de parâmetros
                 inc_fl   = 0,                                            //indicador de incremento
                 dec_fl   = 0;                                            //indicador de decremento
static bool      st_esc;
// =======================================================================
// --- Configurações Iniciais ---
void setup()
{
  Serial.begin(9600);                                                      //Usado para comunicação UART
  Serial2.begin(9600, SERIAL_8N1, 5, 4);                                   //usando RX2 e TX2 do ESP, GPIO02 e 04 para comunicação com modulo LoRa
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);                              //usando RX2 e TX2 do ESP, GPIO16 e 17 para comunicação com modulo GPS
  EEPROM.begin(EEPROM_SIZE);                                               //Inicia EEPROM com o tamanho de endereço
  Wire.begin();
  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));
  pinMode(bt_esc,  INPUT_PULLUP);                                          //entrada para botão esc com pull-up interno
  pinMode(bt_dec,  INPUT_PULLUP);                                          //entrada para botão dec com pull-up interno
  pinMode(bt_inc,  INPUT_PULLUP);                                          //entrada para botão int com pull-up interno
  pinMode(bt_ent,  INPUT_PULLUP);                                          //entrada para botão ent com pull-up interno
  
  display.init();                                                          //Inicialização do display OLED 
  display.flipScreenVertically();                                          //Inverte verticalmente os dados no display
  display.clear();                                                         //Limpa a tela do display
  display.drawXbm(0,0,iluminSystem_width , iluminSystem_height , iluminSystem_bits);   //Declara como posicionar o LOGO iluminsystem
  display.display();                                                       //Imprime informação no display
  delay(1500);                                                             //Aguarda 2s com o logo impresso na tela
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"iluminSystem");                                //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display
  delay(1000);                                                             //Aguarda o tempo
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,0,"FIELD TRACKER");                                //Imprime a String nas posições x e y
  display.drawString(64,20,"iluminSystem");                                //Imprime a String nas posições x e y
  display.drawString(64,40,"LoRaWan");                                     //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display
  delay(1500);                                                             //Tempo de espera
//========================================================================
// --- Inicio das Configurações LoRaWAN ---
  Serial2.write("AT+DEUI=?\r");                                            //Efetua o comando de requisitar DEVEUI do modulo
  delay(1000);                                                             //Tempo de espera
  char deveui;
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    deveui = Serial2.read();                                               //Faz a leitura da Serial2 e armazena na variavel
    DEVEUI += deveui;                                                      //Grava a variavel local na String Global
  }
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,10);                                  //Imprime a barra de progressão 
  display.display();                                                       //Imprime no display
  Serial2.write("AT+APPKEY=63:F4:B1:2A:AE:4F:DC:E5:BB:6B:C8:62:86:47:75:9E\r");  //Efetua o comando de gravar APPKEY no modulo
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    int b = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    Serial.write(b);                                                       //Imprime na Serial monitor a resposta do modulo
  }
  Serial2.write("AT+APPKEY=?\r");                                          //Efetua o comando de requisitar APPKEY do modulo
  char appkey;                                                             //variavel para armazenar char
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0) {                                        //Verifica se há resposta no modulo atraves da Serial2
    appkey = Serial2.read();                                               //Faz a leitura da Serial2 e armazena na variavel
    APPKEY += appkey;                                                      //Grava a variavel local na String Global
  }
     
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,20);                                  //Imprime a barra de progressão
  display.display();                                                       //Imprime no display
  Serial2.write("AT+APPEUI=FF:12:F8:00:00:00:22:AA\r");                    //Efetua o comando de gravar APPEUI no modulo
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    int b = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    Serial.write(b);                                                       //Imprime na Serial monitor a resposta do modulo
  }
  Serial2.write("AT+APPEUI=?\r");                                          //Efetua o comando de requisitar APPEUI do modulo
  char appeui;                                                             //variavel para armazenar char
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0) {                                        //Verifica se há resposta no modulo atraves da Serial2
    appeui = Serial2.read();                                               //Faz a leitura da Serial2 e armazena na variavel
    APPEUI += appeui;                                                      //Grava a variavel local na String Global
  }

  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,40);                                  //Imprime a barra de progressão
  display.display();                                                       //Imprime no display
  Serial2.write("AT+CHMASK=00ff:0000:0000:0000:0001:0000\r");              //Efetua o comando de gravar APPEUI no modulo
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    int b = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    Serial.write(b);                                                       //Imprime na Serial monitor a resposta do modulo
  }
  Serial2.write("AT+CHMASK=?\r");                                          //Efetua o comando de requisitar APPEUI do modulo
  char chmask;                                                             //variavel para armazenar char
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0) {                                        //Verifica se há resposta no modulo atraves da Serial2
    chmask = Serial2.read();                                               //Faz a leitura da Serial2 e armazena na variavel
    CHMASK += chmask;                                                      //Grava a variavel local na String Global
  }
  
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,60);                                  //Imprime a barra de progressão
  display.display();                                                       //Imprime no display
  Serial2.write("AT+DR=3\r");                                              //Efetua o comando de gravar DATARATE IGUAL A 3 no modulo
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    int b = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    Serial.write(b);                                                       //Imprime na Serial monitor a resposta do modulo
  }
  Serial2.write("AT+DR=?\r");                                              //Efetua o comando de requisitar DATARATE = 3 no modulo
  char datarate;                                                           //variavel para armazenar char
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0) {                                        //Verifica se há resposta no modulo atraves da Serial2
    datarate = Serial2.read();                                             //Faz a leitura da Serial2 e armazena na variavel
    DATARATE += datarate;                                                  //Grava a variavel local na String Global
  }

  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,80);                                  //Imprime a barra de progressão
  display.display();                                                       //Imprime no display
  Serial2.write("AT+CLASS=C\r");                                           //Efetua o comando de gravar a classe c no modulo
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    int b = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    Serial.write(b);                                                       //Imprime na Serial monitor a resposta do modulo
  }
  Serial2.write("AT+CLASS=?\r");                                           //Efetua o comando de requisitar a classe gravada no modulo no modulo
  char classe;                                                             //variavel para armazenar char
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0) {                                        //Verifica se há resposta no modulo atraves da Serial2
    classe = Serial2.read();                                               //Faz a leitura da Serial2 e armazena na variavel
    CLASSE += classe;                                                      //Grava a variavel local na String Global
  }

  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,90);                                  //Imprime a barra de progressão
  display.display();                                                       //Imprime no display
  Serial2.write("AT+NJM=1\r");                                             //Efetua o comando de gravar JOIN MOD do modulo (1 = OTAA  /  0 = ABP)
  delay(1000);                                                             //Tempo de espera
  while (Serial2.available() > 0){                                         //Verifica se há resposta no modulo atraves da Serial2
    int b = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    Serial.write(b);                                                       //Imprime na Serial monitor a resposta do modulo
  }
  Serial2.write("AT+NJM=?\r");                                             //Efetua o comando de requisitar JOIN MOD do modulo (1 PARA OTAA E 0 PARA ABP)                                                                //variavel para armazenar char
  delay(1000);                                                             //Tempo de espera
  if (Serial2.available() > 0) {                                        //Verifica se há resposta no modulo atraves da Serial2
    char njm = Serial2.read();                                                  //Faz a leitura da Serial2 e armazena na variavel
  if (njm == '1')                                                         //Se char njm for igual ao caracter 0
    NJM = "OTAA";                                                           //Grava a string "ABP" em NJM
  else                                                    //Senão, testa se njm é igual ao caracter 1
    NJM = "ABP";                                                           //Grava a string "OTAA" em NJM
  }                                                                        //Final do Else                                                                        //Fim do While
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,26,"Iniciando Sistema");                           //Imprime a String nas posições x e y
  display.drawProgressBar(0,0,127,10,100);                                  //Imprime a barra de progressão
  display.display();                                                       //Imprime no display
  
} //end setup
// =======================================================================
// --- Loop Infinito ---
void loop()
{
  if(!adj) menu_sel();                                                    //se flag de ajuste limpa, chama menu principal
  else     sub_menu();                                                    //senão, entra no sub-menu correspondente
  read_keyb();                                                            //lê teclado
  //read_lora();
  char resp;                                                              //Declaração de Variavel para armazenar resposta modulo
  while (Serial2.available()){                                            //Verifica se há resposta no modulo atraves da Serial2
    resp = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    RESP += resp;                                                         //Armazena todos os caracteres na String
    delay(10);
  }
  RESP.replace("\r", "");                                                  //Executa um corte na String apos \r
  RESP.replace("\n", "");
  if (RESP != 0){
    while (digitalRead(bt_esc) == HIGH){
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"DOWNLINK");                                    //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_10);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,String (RESP));                                 //Imprime a String nas posições x e y
  display.display();
  Serial2.write("AT+SEND=3:\r");
    }
  RESP = "";
  }  
} //end loop
// =======================================================================
// --- Desenvolvimento das Funções ---
// =======================================================================
// --- menu_sel ---
// → Controle de seleção dos menus
void menu_sel()
{
  switch(menu_num)                                                        //verifica menu_num
  {
    case 1:                                                               //menu 1
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_16);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_RIGHT);                          //Define o tipo de alinhamento do texto na tela
     display.drawString(70,0,">a) Join:  ");                              //Imprime a String nas posições x e y
     display.drawString(116,20,"b) Send Data:  ");                        //Imprime a String nas posições x e y
     display.drawString(111,40,"c) GPS Data:  ");                         //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     break;                                                               //encerra o laço
    case 2:                                                               //menu 2
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_16);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_RIGHT);                          //Define o tipo de alinhamento do texto na tela
     display.drawString(116,0,">b) Send Data:  ");                        //Imprime a String nas posições x e y
     display.drawString(110,20,"c) GPS Data:  ");                         //Imprime a String nas posições x e y
     display.drawString(115,40,"d) LoRa Data:  ");                        //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     break;                                                               //encerra o laço
    case 3:                                                               //menu 3
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_16);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_RIGHT);                          //Define o tipo de alinhamento do texto na tela
     display.drawString(110,0,">c) GPS Data:  ");                         //Imprime a String nas posições x e y
     display.drawString(115,20,"d) LoRa Data:  ");                        //Imprime a String nas posições x e y
     display.drawString(80,40,"e) Reset:  ");                             //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     break;                                                               //encerra o laço
    case 4:                                                               //menu 4
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_16);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_RIGHT);                          //Define o tipo de alinhamento do texto na tela
     display.drawString(116,0,">d) LoRa Data:  ");                        //Imprime a String nas posições x e y
     display.drawString(82,20,"e) Reset:  ");                             //Imprime a String nas posições x e y
     display.drawString(108,40,"f) Lux Meter:  ");                         //Imprime a String nas posições x e y   
     display.display();                                                   //Apresenta no display
     break;                                                               //encerra o laço
    case 5:                                                               //menu 5 
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_16);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_RIGHT);                          //Define o tipo de alinhamento do texto na tela
     display.drawString(82,0,">e) Reset:  ");                             //Imprime a String nas posições x e y
     display.drawString(108,20,"f) Lux Meter:  ");                         //Imprime a String nas posições x e y
     display.drawString(70,40,"a) Join:  ");                             //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     break;                                                               //encerra o laço
     case 6:                                                               //menu 5 
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_16);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_RIGHT);                          //Define o tipo de alinhamento do texto na tela
     display.drawString(108,0,">f) Lux Meter:  ");                         //Imprime a String nas posições x e y
     display.drawString(70,20,"a) Join:  ");                              //Imprime a String nas posições x e y
     display.drawString(116,40,"b) Send Data:  ");                        //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     break;                                                               //encerra o laço   
  }                                                                       //end switch
}                                                                         //end menu_sel

// =======================================================================
// --- sub_menu ---
void sub_menu()
{  
  switch(menu_num)                                                        //verifica menu_num
  {        
    case 1:                                                               //caso 1  
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_24);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_CENTER);                         //Define o tipo de alinhamento do texto na tela
     display.drawString(64,15,"Join");                                    //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     join();                                                              //Inicio da função para executar join na rede lorawan
     adj = 0;
     delay(1500);
     break;                                                               //encerra o laço

    case 2:                                                               //caso 2
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_24);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_CENTER);                         //Define o tipo de alinhamento do texto na tela
     display.drawString(64,15,"Send Data");                               //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     delay(1500);
     sendData();                                                          //Inicio da função para enviar dados na rede lorawan
     adj = 0;
     break;                                                               //encerra o laço

    case 3:                                                               //caso 3
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_24);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_CENTER);                         //Define o tipo de alinhamento do texto na tela
     display.drawString(64,15,"GPS Data");                                //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     delay(1500);
     gpsdata();                                                            //Inicio da função para ler gps e apresentar no lcd
     adj = 0;
     break;                                                               //encerra o laço

    case 4:                                                               //caso 4
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_24);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_CENTER);                         //Define o tipo de alinhamento do texto na tela
     display.drawString(64,15,"LoRa Data");                               //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     delay(1500);
     loradata();                                                          //Inicio da função para apresentar dados lorawan no lcd (DevEUI, APPKEY, APPEUI, CLASS, CHMASK...)                           
     adj = 0;
     break;                                                               //encerra o laço

    case 5:                                                               //caso 5
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_24);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_CENTER);                         //Define o tipo de alinhamento do texto na tela
     display.drawString(64,15,"Reset");                                   //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     reset();                                                             //Inicio da função para efetuar reset no modulo LoRawan
     adj = 0;
     break;                                                               //encerra o laço

    case 6:                                                               //caso 5
     display.clear();                                                     //Limpa o display
     display.setFont(ArialMT_Plain_24);                                   //Seta o tipo de fonte e o tamanho dela no display
     display.setTextAlignment(TEXT_ALIGN_CENTER);                         //Define o tipo de alinhamento do texto na tela
     display.drawString(64,15,"Lux Meter");                               //Imprime a String nas posições x e y
     display.display();                                                   //Apresenta no display
     delay(1500);
     luximetro();                                                         //Inicio da função para efetuar reset no modulo LoRawan
     adj = 0;
     break;
  } //end switch menu_num
} //end sub_menu

// =======================================================================
// --- read_keyb ---
void read_keyb()
{
  static bool st_esc=0,                                                   //status do botão esc
              st_dec=0,                                                   //status do botão dec
              st_inc=0,                                                   //status do botão inc
              st_ent=0,                                                   //status do botão ent
              fast_inc=0,                                                 //indicador para ajuste incremental rápido
              fast_dec=0;                                                 //indicador para ajuste decremental rápido    
  if(!digitalRead(bt_esc)) st_esc=1;                                      //se esc pressionado, status de esc setado
  if(!digitalRead(bt_dec)) st_dec=1;                                      //se dec pressionado, status de dec setado
  if(!digitalRead(bt_inc)) st_inc=1;                                      //se inc pressionado, status de inc setado
  if(!digitalRead(bt_ent)) st_ent=1;                                      //se ent pressionado, status de ent setado
  
  if(digitalRead(bt_esc) && st_esc)                                       //esc solto e status setado?
  {                                                                       //sim
    st_esc = 0;                                                           //limpa status
    adj    = 0;                                                           //limpa indicador de ajuste de parâmetro
    inc_fl = 0;                                                           //limpa flag de incremento
    dec_fl = 0;                                                           //limpa flag de decremento
    bl_time_save = millis();                                              //armazena tempo atual
    delay(150);                                                           //anti-bouncing
  }                                                                       //end if bt_esc
  
  if(digitalRead(bt_dec) && st_dec)                                       //dec solto e status setado?
  {                                                                       //sim
    st_dec = 0;                                                           //limpa status
    dec_fl = 1;                                                           //seta flag de decremento
    fast_dec = 0;                                                         //limpa flag de ajuste rápido
    if(!adj) menu_num--;                                                  //decrementa menu_num, se adj estiver limpa
    if(menu_num<1) menu_num=M_MAX;                                        //se menu_num menor que 1, volta para o máximo
    bl_time_save = millis();                                              //armazena tempo atual
    delay(150);                                                           //anti-bouncing
  }                                                                       //end if bt_dec

  if(digitalRead(bt_inc) && st_inc)                                       //inc solto e status setado?
  {                                                                       //sim
    st_inc = 0;                                                           //limpa status
    inc_fl = 1;                                                           //seta flag de incremento
    fast_inc = 0;                                                         //limpa flag de ajuste rápido
    if(!adj) menu_num++;                                                  //incrementa menu_num, se adj estiver limpa
    if(menu_num>M_MAX) menu_num=1;                                        //se menu_num maior que máximo, volta para 1
    bl_time_save = millis();                                              //armazena tempo atual
    delay(150);                                                           //anti-bouncing 
  }                                                                       //end if bt_inc

  if(digitalRead(bt_ent) && st_ent)                                       //ent solto e status setado?
  {                                                                       //sim
    st_ent = 0;                                                           //limpa status
    adj    = 1;                                                           //seta indicador de ajuste de parâmetros
    inc_fl = 0;                                                           //limpa flag de incremento
    dec_fl = 0;                                                           //limpa flag de decremento
    bl_time_save = millis();                                              //armazena tempo atual
    delay(150);                                                           //anti-bouncing   
  } 
} 
// =======================================================================
// --- join ---
void join()
{
  Serial2.write("AT+JOIN\r");
  display.clear();
  display.setFont(ArialMT_Plain_16);                                       
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             
  display.drawString(64,0,"EXECUTANDO:");                                                                                 
  display.drawString(64,20,"JOIN REDE");
  display.drawString(64,40,"LoRaWAN");                         
  display.display();
   while (Serial2.available() > 0) {
    int b = Serial2.read();
    Serial.write(b);
   }
  delay(1000);
  char join;
  for (int i = 0; i < 2; i++) {
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"JOIN REDE");
  display.drawString(64,40,"LoRaWAN"); 
  display.drawProgressBar(0,0,127,10,10);                                  //Imprime a barra de progressão 
  display.display();                                                       //Imprime no display
  delay(1000);
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"JOIN REDE");
  display.drawString(64,40,"LoRaWAN"); 
  display.drawProgressBar(0,0,127,10,30);                                  //Imprime a barra de progressão 
  display.display();                                                       //Imprime no display
  delay(1000);
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"JOIN REDE");
  display.drawString(64,40,"LoRaWAN"); 
  display.drawProgressBar(0,0,127,10,70);                                  //Imprime a barra de progressão 
  display.display();                                                       //Imprime no display
  delay(1000);
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"JOIN REDE");
  display.drawString(64,40,"LoRaWAN"); 
  display.drawProgressBar(0,0,127,10,100);                                  //Imprime a barra de progressão 
  display.display();                                                       //Imprime no display
  delay(1000);
    while (Serial2.available() > 0) {
      join = Serial2.read();
      textoRecebido += join;
    }
    Serial.println(textoRecebido); 
    textoRecebido.replace("\r", "");
    textoRecebido.replace("\n", "");
    if (textoRecebido == Join1) {
        display.clear();
        display.setFont(ArialMT_Plain_16);                                       
        display.setTextAlignment(TEXT_ALIGN_CENTER);                             
        display.drawString(64,0,"RECEBIDO:");                                                                                 
        display.setFont(ArialMT_Plain_10);                                       
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,33,Join1);                        
        display.display();
    } else if (textoRecebido == Join2) {
        display.clear();
        display.setFont(ArialMT_Plain_16);                                       
        display.setTextAlignment(TEXT_ALIGN_CENTER);                             
        display.drawString(64,0,"RECEBIDO:");                                                                                 
        display.setFont(ArialMT_Plain_10);                                       
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,33,Join2);                        
        display.display();
    } else if (textoRecebido == Join3) {
        display.clear();
        display.setFont(ArialMT_Plain_16);                                       
        display.setTextAlignment(TEXT_ALIGN_CENTER);                             
        display.drawString(64,0,"RECEBIDO:");                                                                                 
        display.setFont(ArialMT_Plain_10);                                       
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64,33,Join3);                        
        display.display();
    }
    textoRecebido = "";
   }
  Serial2.write("AT+NJS=?\r");
  delay(1000);
  if (Serial2.available() > 0) {
    char j = Serial2.read();
    if ( j == '1' ) {
      display.clear();
      display.setFont(ArialMT_Plain_16);                                       
      display.setTextAlignment(TEXT_ALIGN_CENTER);                             
      display.drawString(64,0,"RECEBIDO:");                                                                                 
      display.setFont(ArialMT_Plain_10);                                       
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64,33,"Join realizado com sucesso");                        
      display.display();      
    } else {
      display.clear();
      display.setFont(ArialMT_Plain_16);                                       
      display.setTextAlignment(TEXT_ALIGN_CENTER);                             
      display.drawString(64,0,"RECEBIDO:");                                                                                 
      display.setFont(ArialMT_Plain_10);                                       
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.drawString(64,33,"Join não realizado");                        
      display.display(); 
    }
  }
}
// =======================================================================
// --- sendData ---
void sendData()
{
  display.clear();
  display.setFont(ArialMT_Plain_16);                                       
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             
  display.drawString(64,0,"AGUARDANDO");                                                                                 
  display.setFont(ArialMT_Plain_16);                                       
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64,33,"DADOS GPS");                        
  display.display();
  delay(1000);
  gpsdataSEND();
  luximetroSEND();
  char payload[256];
  sprintf(payload, "AT+SEND=3:{\"T\":\"1\",\"PC\":%i,\"LA\":%.6f,\"LG\":%.6f,\"ST\":%.0f,\"LX\":%.2f}\r", pacote, latitude, longitude, satelite, luxxi);
  Serial2.write(payload);
  Serial.println(payload);
  display.clear();
  display.setFont(ArialMT_Plain_16);                                       
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             
  display.drawString(64,0,"ENVIANDO");                                                                                 
  display.setFont(ArialMT_Plain_16);                                       
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64,33,"PAYLOAD");                        
  display.display();
  delay(1000);
  pacote++;
}
// =======================================================================
// --- gpsdata ---
void gpsdata()
{ 
  while (digitalRead(bt_esc) == HIGH){
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  if(newData == true)                                                     //If newData is true
  {
    newData = false;
    //Serial.println(gps.satellites.value());
    {                                                                      //inicio da antiga função print_speed 
  if (gps.location.isValid() == 1)
  {
    display.clear();                                                       //Limpa o display
    display.setFont(ArialMT_Plain_10);                                     //Seta o tipo de fonte e o tamanho dela no display
    display.setTextAlignment(TEXT_ALIGN_CENTER);                           //Define o tipo de alinhamento do texto na tela
    
    latitude = gps.location.lat(),6;
    LAT = String(latitude, 6);
    display.drawString(15,5,"Lat: ");                                      //Imprime a String nas posições x e y
    display.drawString(55,5,LAT);
    
    longitude = gps.location.lng(),6;
    LNG = String(longitude, 6);
    display.drawString(15,20,"Lng: ");                                     //Imprime a String nas posições x e y
    display.drawString(55,20,LNG);                                         //Imprime a String nas posições x e y

    speeds = gps.speed.kmph();
    SPD = String(speeds, 2);
    display.drawString(15,35,"Spd: ");                                     //Imprime a String nas posições x e y
    display.drawString(40,35,SPD);                                         //Imprime a String nas posições x e y

    satelite = gps.satellites.value();
    SAT = String(satelite, 0);
    display.drawString(15,50,"Sat:");                                      //Imprime a String nas posições x e y
    display.drawString(30,50,SAT);                                         //Imprime a String nas posições x e y
    
    altitude = gps.altitude.meters(), 0;
    ALT = String(altitude, 0);
    display.drawString(80,50,"Alt:");                                      //Imprime a String nas posições x e y
    display.drawString(100,50,ALT);                                         //Imprime a String nas posições x e y
    display.display();                                                     //Imprime no display
    //delay(3000);     
  }
  else
  {
    display.clear();                                                       //Limpa o display
    display.setFont(ArialMT_Plain_16);                                     //Seta o tipo de fonte e o tamanho dela no display
    display.setTextAlignment(TEXT_ALIGN_CENTER);                           //Define o tipo de alinhamento do texto na tela
    display.drawString(64,20,"No Data");                                   //Imprime a String nas posições x e y
    display.display();                                                     //Imprime no display
    delay(1000);
    Serial.print("No Data");   
  }
}                                                                          //fim da antiga função print_speed
  }
  else
  {  
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"No Data");                                     //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display
  delay(1000); 
  Serial.print("No Data");
  } 
 }
}
// =======================================================================
// --- gpsdataSEND ---
void gpsdataSEND()
{ 
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }
  if(newData == true)                                                     //If newData is true
  {
    newData = false;
    //Serial.println(gps.satellites.value());
    {                                                                      //inicio da antiga função print_speed 
  if (gps.location.isValid() == 1)
  {
    display.clear();                                                       //Limpa o display
    display.setFont(ArialMT_Plain_10);                                     //Seta o tipo de fonte e o tamanho dela no display
    display.setTextAlignment(TEXT_ALIGN_CENTER);                           //Define o tipo de alinhamento do texto na tela
    
    latitude = gps.location.lat(),6;
    LAT = String(latitude, 6);
    display.drawString(15,5,"Lat: ");                                      //Imprime a String nas posições x e y
    display.drawString(55,5,LAT);
    
    longitude = gps.location.lng(),6;
    LNG = String(longitude, 6);
    display.drawString(15,20,"Lng: ");                                     //Imprime a String nas posições x e y
    display.drawString(55,20,LNG);                                         //Imprime a String nas posições x e y

    speeds = gps.speed.kmph();
    SPD = String(speeds, 2);
    display.drawString(15,35,"Spd: ");                                     //Imprime a String nas posições x e y
    display.drawString(40,35,SPD);                                         //Imprime a String nas posições x e y

    satelite = gps.satellites.value();
    SAT = String(satelite, 0);
    display.drawString(15,50,"Sat:");                                      //Imprime a String nas posições x e y
    display.drawString(30,50,SAT);                                         //Imprime a String nas posições x e y
    
    altitude = gps.altitude.meters(), 0;
    ALT = String(altitude, 0);
    display.drawString(80,50,"Alt:");                                      //Imprime a String nas posições x e y
    display.drawString(100,50,ALT);                                         //Imprime a String nas posições x e y
    display.display();                                                     //Imprime no display
    //delay(3000);     
  }
  else
  {
    display.clear();                                                       //Limpa o display
    display.setFont(ArialMT_Plain_16);                                     //Seta o tipo de fonte e o tamanho dela no display
    display.setTextAlignment(TEXT_ALIGN_CENTER);                           //Define o tipo de alinhamento do texto na tela
    display.drawString(64,20,"No Data");                                   //Imprime a String nas posições x e y
    display.display();                                                     //Imprime no display
    delay(1000);
    Serial.print("No Data");   
  }
}                                                                          //fim da antiga função print_speed
  }
  else
  {  
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,20,"No Data");                                     //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display
  delay(1000); 
  Serial.print("No Data");
  } 
}
// =======================================================================
// --- loradata ---
void loradata()
{
  delay(1000);
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"DEVEUI");                                      //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_10);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,DEVEUI);                                        //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"APPKEY");                                      //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_10);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,APPKEY);                                        //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"APPEUI");                                      //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_10);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,APPEUI);                                        //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"CHMASK");                                      //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_10);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,CHMASK);                                        //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"DATARATE");                                    //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_24);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(120,33,DATARATE);                                      //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"CLASSE");                                      //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_24);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(120,33,CLASSE);                                        //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"NJM");                                         //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_24);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,NJM);                                           //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  delay(2000);                                                             //Aguarda 2 segundos    
}
// =======================================================================
// --- reset ---
void reset()
{
  Serial2.write("ATZ\r");                                                 //Efetua o comando de reset no modulo
  delay(3000);                                                            //Tempo de espera
  char res;
  while (Serial2.available() > 0) {                                       //Verifica se há resposta no modulo atraves da Serial2
    res = Serial2.read();                                                 //Faz a leitura da Serial2 e armazena na variavel
    RESET += res;                                                         //Grava a variavel local na String Global
  }
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_10);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"RESET");                                       //Imprime a String nas posições x e y
  display.drawString(64,23,RESET);                                         //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display
  delay(1500);
  RESET = "";   
}
// =======================================================================
// --- read_lora ---
void read_lora()
{
  char resp;                                                              //Declaração de Variavel para armazenar resposta modulo
  while (Serial2.available()){                                            //Verifica se há resposta no modulo atraves da Serial2
    resp = Serial2.read();                                                //Faz a leitura da Serial2 e armazena na variavel
    RESP += resp;                                                         //Armazena todos os caracteres na String
  }
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"DOWNLINK");                                    //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_24);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,String (RESP));                                 //Imprime a String nas posições x e y
  display.display();
  delay(1000);
  //Serial.print("Resposta recebida: ");                                    //Imprime na Serial "print da resposta recebida"
  //Serial.println(RESP);
  RESP = "";
}
// =======================================================================
// --- luximetro ---
void luximetro()
{
  while (digitalRead(bt_esc) == HIGH){
  luxxi = lightMeter.readLightLevel();
  LUX = String(luxxi, 2);
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"LIGHT");                                       //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_24);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,LUX);                                           //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
  }
}
// =======================================================================
// --- luximetro ---
void luximetroSEND()
{
  luxxi = lightMeter.readLightLevel();
  LUX = String(luxxi, 2);
  display.clear();                                                         //Limpa o display
  display.setFont(ArialMT_Plain_16);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,00,"LIGHT");                                       //Imprime a String nas posições x e y
  display.setFont(ArialMT_Plain_24);                                       //Seta o tipo de fonte e o tamanho dela no display
  display.setTextAlignment(TEXT_ALIGN_CENTER);                             //Define o tipo de alinhamento do texto na tela
  display.drawString(64,33,LUX);                                           //Imprime a String nas posições x e y
  display.display();                                                       //Imprime no display 
}
