/*
-Interface entre microcontrolador ESP32 e SmartCards utilizando o gateway NCN6001-

Desenvolvido por: Gabriel de Lelis
Data: 08/04/2025
Versão: 2.0.0

Este programa tem o intuito de ler o ATR de SmartCards e trocar APDUs por meio do terminal serial. Utiliza o CI NCN6001 como gateway e um pino I/O
qualquer para comunicação bidirecional emulando o funcionamento de uma UART half-duplex.
*/

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#define INT_NCN 19    // Pino de sinalização de cartão inserido
#define CLK_NCN 21    // Pino de fornecimento de clock para o NCN6001
#define MISO 12       // Pino MISO do SPI do NCN6001
#define MOSI 13       // Pino MOSI do SPI do NCN6001
#define CLK_SPI 14    // Pino CLOCK do SPI do NCN6001
#define CS 15         // Pino CS do SPI do NCN6001

#define SOFT_SERIAL_PIN 22  //Pino para comunicação serial emulada bidirecional half-duplex (IO_NCN)

#define FLAG_PIN 18         // Pino de flag para fins de debug


#define PWM_FREQUENCY1 922560  //3571200 para 9600 baud iniciais // Frequência inicial do smartCard (baudRate = f/372)
                               // Frequência final do smartCard (baudRate = f*Di/Fi)
#define PWM_RESOLUTION 4       // Resolução do duty cycle em bits (mínimo necessário)
#define Di 32
#define Fi 512

String atrString = "";              // Variável para armazenar o ATR
String soft_serial_received = "";   // Variável global para armazenar os dados recebidos na softSerial
byte Ns = 0x00;                     // Send-sequence bit, alterna entre 0x00 e 0x40

SoftwareSerial softSerial(-1,-1);  // Inicializa UART emulada sem pinos definidos inicialmente


// Função para habilitar o clock para o NCN6001. Ativa ao receber 1 e desativa ao receber 0.
void enableFreeRunningClock(bool comando){
  if(comando == 0)
    ledcWrite(0, 0); // Duty cycle de 0% 
  else
    ledcWrite(0, 7); // Duty cycle de ~50% 
}

// Função que será chamada na interrupção, ao se inserir ou remover o cartão 
void IRAM_ATTR intInterrupt() {
  
}

// Função para enviar dados via SPI e receber a resposta. Recebe um byte para enviar e retorna um byte de resposta.
uint8_t sendSPI(uint8_t dataToSend){
  uint8_t  miso_data;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS, LOW);  // Lower CS/SS line to select the device
  miso_data = SPI.transfer(dataToSend);  // Send and receive data
  //digitalWrite(CS, HIGH); // Raise CS/SS line to end communication
  SPI.endTransaction();

  return miso_data;
}


// Configura a tensão do smart card. Recebe como parâmetro: -> 18=1.8V    33=3.3V     50=5.0V   0=0V
void configureVoltage(int voltage){
  if(voltage == 50){
    sendSPI(0x83);
  }else if(voltage == 33){
    sendSPI(0x82);
  }else if(voltage == 18){
    sendSPI(0x81);
  }else{
    sendSPI(0x80);
  }
}

//Liga e desliga um pino de flag rapidamente para fins de debug
void pin_flag(){
  digitalWrite(FLAG_PIN, HIGH);
  digitalWrite(FLAG_PIN, LOW);
}

// Calcula o LRC (Longitudinal Redundancy Check) de um array de bytes. Recebe um ponteiro para os dados e seu comprimento.
byte calc_LRC(byte* data, size_t length) {
  byte lrc = 0; // Inicializa o LRC como 0
  for (size_t i = 0; i < length; i++) {
    lrc ^= data[i]; // Calcula o LRC usando XOR bit a bit
  }
  return lrc; // Retorna o LRC calculado
}

// Função para encapsular I-block types. Recebe um array de bytes, seu tamanho e retorna o APDU encapsulado.
byte* encapsulateIBlock(byte* apdu, size_t length) {
  byte* encapsulatedAPDU = new byte[length + 4]; // Aloca memória para o APDU encapsulado
  encapsulatedAPDU[0] = 0x00; // NAD (NAD = 0x00 padrão)
  encapsulatedAPDU[1] = Ns; // PCB (PCB = 0b0X000000 com bit7 alternando)
  encapsulatedAPDU[2] = (byte)length; // LEN (Comprimento do APDU original)
  memcpy(encapsulatedAPDU + 3, apdu, length); // Copia o APDU original para o novo array
  encapsulatedAPDU[length + 3] = calc_LRC(encapsulatedAPDU, length+3); // LRC (Longitudinal Redundancy Check)
  
  Ns ^= 0x40; // Alterna o bit Ns (Send-sequence bit) para o próximo envio

  return encapsulatedAPDU; // Retorna o APDU encapsulado
    
}  

//Recebe um array de bytes, juntamente com seu tamanho, envia e retorna o apdu de resposta
String sendAPDU(byte* apdu, size_t length) {
  String response = ""; // Variável para armazenar a resposta

  digitalWrite(SOFT_SERIAL_PIN, HIGH);  // Garante que ao trocar o pino de Rx para Tx, o pino continue em nível alto
  pinMode(SOFT_SERIAL_PIN, OUTPUT);     // Define o pino como saída
  softSerial.begin(PWM_FREQUENCY1*Di/Fi, SWSERIAL_8E1,-1,SOFT_SERIAL_PIN); //Define o pino como Tx
  pin_flag();
  softSerial.write(apdu, length);
  softSerial.begin(PWM_FREQUENCY1*Di/Fi, SWSERIAL_8E1,SOFT_SERIAL_PIN,-1); //Define o pino como Rx de volta
  delay(400);
  
  // Lê a resposta da softSerial
  while(softSerial.available()) {
      char c = softSerial.read();
      // Formata o byte recebido como hexadecimal com dois dígitos
      if (c < 0x10) {
        response += "0"; // Adiciona zero à esquerda se necessário
      }
      response += String(c,HEX);
  }

  return response; // Retorna a resposta recebida
}

// Apenas imprime a string recebida colocando um espaço a cada 2 dígitos, para facilitar a leitura
void printAPDU(String apdu) {
  if (apdu != "") {
    Serial.print("Response Received: ");
    for (size_t i = 0; i < apdu.length(); i += 2) {
      String byteString = apdu.substring(i, i + 2); // Extrai dois caracteres
      byteString.toUpperCase(); // Converte para maiúsculas
      Serial.print(byteString); // Imprime os dois caracteres
      Serial.print(" "); // Adiciona um espaço após cada par
    }
    Serial.println(); // Finaliza a linha
  }
}

void powerDownSmartCard(){
  //PowerDown ISO7816 sequence
  sendSPI(0x85);              // Reset = Low
  delay(10);
  sendSPI(0x81);              // Clk = Low
  delay(90);
  configureVoltage(0);        // Card_VCC = Low
}

String resetSmartCard(){
  pin_flag();
  configureVoltage(18); //Liga smartCard - Define tensão do smart card: 18=1.8V
  delay(200);
  
  sendSPI(0x85);  // CLKdiv = 1 - Liga o clock do smartcard
  softSerial.begin(PWM_FREQUENCY1/372, SWSERIAL_8E1,SOFT_SERIAL_PIN,-1); // Inicializa a comunicação serial emulada como receptor, com baud rate proporcional à frequência do PWM
  delay(1000);
  sendSPI(0x95);  // CLKdiv = 1    RESET=1 - Reseta o smartcard
  delay(150);

  //Leitura de ATR
  String atr = "";
  if (softSerial.available()) {
    while(softSerial.available()) {
      char c = softSerial.read();
      if (c < 0x10) {
        atr += "0"; // Adiciona zero à esquerda se necessário
      }
      atr += String(c,HEX);
    }
    Serial.print("ATR Received: ");
    for (size_t i = 0; i < atr.length(); i += 2) {
      String byteString = atr.substring(i, i + 2); // Extrai dois caracteres
      byteString.toUpperCase(); // Converte para maiúsculas
      Serial.print(byteString); // Imprime os dois caracteres
      Serial.print(" "); // Adiciona um espaço após cada par
    }
    Serial.println(); // Finaliza a linha
    
    // Envio de PPS (Protocol and parameters selection)
    // Protocol T=1 ; Fi = 512; Di = 32; For maximum baud rate at specified clock
    byte parameter_set[] = {0xFF, 0x11, 0x96, 0x78};  // Envia o comando de configuração de parâmetros de comunicação
    digitalWrite(SOFT_SERIAL_PIN, HIGH);  // Garante que ao trocar o pino de Rx para Tx, o pino continue em nível alto
    pinMode(SOFT_SERIAL_PIN, OUTPUT);     // Define o pino como saída
    softSerial.begin(PWM_FREQUENCY1/372, SWSERIAL_8E1,-1,SOFT_SERIAL_PIN); //Define o pino como Tx
    softSerial.write(parameter_set, sizeof(parameter_set)); // Envia o comando de configuração
    softSerial.begin(PWM_FREQUENCY1/372, SWSERIAL_8E1,SOFT_SERIAL_PIN,-1); //Define o pino como Rx de volta
    delay(400);
    pin_flag();
    softSerial.begin(PWM_FREQUENCY1*Di/Fi, SWSERIAL_8E1,SOFT_SERIAL_PIN,-1);

    //Envia um IFS Request e imprime a resposta
    byte S_block_apdu[] = {0x00, 0xC1, 0x01, 0xFE, 0x00};   // IFS Request setando max 254 blocos/apdu (0xFE)
    S_block_apdu[4] = calc_LRC(S_block_apdu, sizeof(S_block_apdu)-1); // Calcula o LRC para o S-Block
    String response0 = sendAPDU(S_block_apdu, sizeof(S_block_apdu));
    Serial.print("IFS ");
    printAPDU(response0);
  }else{
    Serial.println("ERROR: No ATR received.");
  }

  return atr; // Retorna o ATR recebido
}

//******************************************INÍCIO SETUP****************************************************//
void setup() {
  //Inicializa pinos
  pinMode(INT_NCN,INPUT);
  pinMode(CLK_NCN,OUTPUT);
  pinMode(CS,OUTPUT);
  pinMode(FLAG_PIN,OUTPUT);

  //Inicializa pinos
  digitalWrite(CS, LOW); // Raise CS/SS line to end communication
  digitalWrite(CLK_NCN, LOW);
  digitalWrite(FLAG_PIN, LOW);

  Serial.begin(115200);
  
  // Configuração do canal PWM (LEDC)
  ledcSetup(0, PWM_FREQUENCY1, PWM_RESOLUTION);
  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(CLK_NCN, 0);
  // Ativa o clock com duty cycle 50%
  enableFreeRunningClock(1);

  // Associar a função de interrupção ao pino INT
  attachInterrupt(digitalPinToInterrupt(INT_NCN), intInterrupt, FALLING);
  
  SPI.begin(CLK_SPI,MISO,MOSI,CS);  /* begin SPI */

  Serial.println("Initiate");
  Serial.println();
  
  powerDownSmartCard(); //Desliga o smartcard para garantir que ele esteja desligado antes de iniciar
  delay(2000);
  atrString = resetSmartCard(); // Reseta o smartcard e lê o ATR
  //**************FIM DAS CONFIGURAÇÕES, E RESET DO SMARTCARD É REALIZADO************ */

  delay(500);

}


void loop() {
  //uint8_t received_SPI = sendSPI(0x95); //Faz um envio mantendo o estado do SPI, para receber o estado do NCN6001
  //Serial.print("   Received SPI: 0x");
  //Serial.println(received_SPI,HEX);

  //Aguarda o envio de um APDU pelo terminal serial, transmite ao smartcard e imprime a resposta
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Lê a entrada do terminal até o caractere de nova linha
    input.trim(); // Remove espaços ou caracteres extras no início/fim da string
    
    // Se for enviada a palavra "reset" no terminal, não será enviado APDU, e o smartcard será resetado
    if(input == "reset"){
      Serial.println("Resetting smartcard...");
      if((sendSPI(0x95) & 0x01) == 0){ // Verifica se o estado atual do gateway é "desligado"
        pin_flag();
        powerDownSmartCard();   // Executa sequência de powerdown obrigatória
        delay(2000);
      }
      atrString = resetSmartCard(); // Reseta o smartcard e lê o ATR
      return;
    }
    // Se for enviada a palavra "pdown" no terminal, não será enviado APDU, e o smartcard será desligado
    if(input == "pdown"){
      Serial.println("Powering down smartcard...");
      powerDownSmartCard(); 
      return;
    }

    // I-block type encapsulation - Esperado receber o conteúdo de dados do bloco
    if(input.charAt(0) == 'i'){
      input.remove(0, 1);               // Remove o primeiro caractere da string input
      if (input.length() % 2 != 0) {
        Serial.println("Erro: APDU inválida. Certifique-se de que o comprimento seja par.");
        return;
      }  
      // Converte a string em um array de bytes
      size_t length = input.length() / 2;
      byte apdu[length];
      for (size_t i = 0; i < length; i++) {
        String byteString = input.substring(i * 2, i * 2 + 2); // Extrai dois caracteres
        apdu[i] = (byte)strtol(byteString.c_str(), nullptr, 16); // Converte para byte
      }

      // Envia a APDU e imprime a resposta
      Serial.print("Sending I-Block: ");
      byte* apdu_encapsulated = encapsulateIBlock(apdu, length); // Envia a APDU encapsulada
      for (size_t i = 0; i < length + 4; i++) { // length + 4 porque o encapsulamento adiciona 4 bytes
        if (apdu_encapsulated[i] < 0x10) {
          Serial.print("0"); // Adiciona o zero à esquerda
        }
        Serial.print(apdu_encapsulated[i], HEX); // Imprime cada byte em hexadecimal
        Serial.print(" "); // Adiciona um espaço entre os bytes
      }
      Serial.println(); // Finaliza a linha
    
      soft_serial_received = sendAPDU(apdu_encapsulated, length+4); // Envia o APDU encapsulado e recebe a resposta
      delete[] apdu_encapsulated; // Libera a memória alocada para o APDU encapsulado
      printAPDU(soft_serial_received);
      return;
    }

    // S-block type encapsulation
    if(input.charAt(0) == 's'){
      // not implemented yet
      return;
    }

    // R-block type encapsulation - Esperado receber o N(R) bit
    if(input.charAt(0) == 'r'){
      byte nr = (input.charAt(1) & 0x01) << 4;
      nr |= 0x80;
      //Envia um R-block - Envia o N(R) bit recebido
      byte R_block_apdu[4] = {0x00, nr, 0x00, 0x00};   
      R_block_apdu[3] = calc_LRC(R_block_apdu, sizeof(R_block_apdu)-1); // Calcula o LRC para o R-Block
      String response0 = sendAPDU(R_block_apdu, sizeof(R_block_apdu));
      printAPDU(response0);
      return;
    }

    // Caso nenhum dos comandos acima seja enviado, o programa assume que o usuário está enviando um APDU raw
    if (input.length() % 2 != 0) {
      Serial.println("Erro: APDU inválida. Certifique-se de que o comprimento seja par.");
      return;
    }
    // Converte a string em um array de bytes
    size_t length = input.length() / 2;
    byte apdu[length];
    for (size_t i = 0; i < length; i++) {
      String byteString = input.substring(i * 2, i * 2 + 2); // Extrai dois caracteres
      apdu[i] = (byte)strtol(byteString.c_str(), nullptr, 16); // Converte para byte
    }
    // Envia a APDU e imprime a resposta
    soft_serial_received = sendAPDU(apdu, length);
    printAPDU(soft_serial_received);
    
  }

  delay(50);
}

