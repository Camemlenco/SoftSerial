/*
-Interface entre microcontrolador ESP32 e SmartCards utilizando o gateway NCN6001-

Desenvolvido por: Gabriel de Lelis
Data: 08/04/2025
Versão: 3.0

Este programa tem o intuito de ler o ATR de SmartCards e trocar APDUs por meio do terminal serial. Utiliza o CI NCN6001 como gateway e um pino I/O
qualquer para comunicação bidirecional emulando o funcionamento de uma UART half-duplex.
*/

//******INCLUDES*********************************************************************** */
#include <main.h>
#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <atr.h>

//******DEFINES************************************************************************ */
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

//******GLOBAL VARIABLES********************************************************************************** */
String atrString = "";              // Variável para armazenar o ATR
String soft_serial_received = "";   // Variável global para armazenar os dados recebidos na softSerial
uint8_t Ns = 0x00;                     // Send-sequence bit, alterna entre 0x00 e 0x40
ATR_TypeDef atr_struct;                    // Answer To Reset structure 
uint8_t response_buffer[255];          // Buffer para armazenar a resposta do APDU

SoftwareSerial softSerial(-1,-1);  // Inicializa UART emulada sem pinos definidos inicialmente

//******CUSTOM FUNCTIONS********************************************************************************************* */

/**
  * @brief  Função para habilitar o clock de smartcard para o NCN6001
  * @param  comando: Ativa ao receber 1 e desativa ao receber 0
  * @retval None
*/
void enableFreeRunningClock(bool comando){
  if(comando == 0)
    ledcWrite(0, 0); // Duty cycle de 0% 
  else
    ledcWrite(0, 7); // Duty cycle de ~50% 
}

// Função que será chamada na interrupção, ao se inserir ou remover o cartão 
void IRAM_ATTR intInterrupt() {
  
}

/**
  * @brief  Função para enviar dados via SPI e receber a resposta
  * @param  dataToSend: Recebe um byte para enviar
  * @retval miso_data: Byte recebido via SPI
*/
uint8_t sendSPI(uint8_t dataToSend){
  uint8_t  miso_data;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(CS, LOW);  // Lower CS/SS line to select the device
  miso_data = SPI.transfer(dataToSend);  // Send and receive data
  //digitalWrite(CS, HIGH); // Raise CS/SS line to end communication
  SPI.endTransaction();

  return miso_data;
}

/**
  * @brief  Configura a tensão do smart card
  * @param  voltage: Recebe como parâmetro: -> 18=1.8V    33=3.3V     50=5.0V   0=0V
  * @retval None
*/
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

/**
  * @brief  Liga e desliga rapidamente um pino de flag para fins de debug 
  * @retval None
*/
void pin_flag(){
  digitalWrite(FLAG_PIN, HIGH);
  digitalWrite(FLAG_PIN, LOW);
}

/**
  * @brief  Calcula o LRC (Longitudinal Redundancy Check) de um array de bytes
  * @param  data: Ponteiro para o array de bytes
  * @param  length: Tamanho do array de bytes
  * @retval lrc: Retorna o LRC calculado
*/
uint8_t calc_LRC(uint8_t* data, size_t length) {
  uint8_t lrc = 0; // Inicializa o LRC como 0
  for (size_t i = 0; i < length; i++) {
    lrc ^= data[i]; // Calcula o LRC usando XOR bit a bit
  }
  return lrc; // Retorna o LRC calculado
}

/**
  * @brief  Função para encapsular I-block types
  * @param  data: Ponteiro para o array de bytes
  * @param  length: Tamanho do array de bytes 
  * @retval encapsulatedAPDU: Retorna o APDU encapsulado
*/
uint8_t* encapsulateIBlock(uint8_t* apdu, size_t length) {
  uint8_t* encapsulatedAPDU = new uint8_t[length + 4]; // Aloca memória para o APDU encapsulado
  encapsulatedAPDU[0] = 0x00; // NAD (NAD = 0x00 padrão)
  encapsulatedAPDU[1] = Ns; // PCB (PCB = 0b0X000000 com bit7 alternando)
  encapsulatedAPDU[2] = (uint8_t)length; // LEN (Comprimento do APDU original)
  memcpy(encapsulatedAPDU + 3, apdu, length); // Copia o APDU original para o novo array
  encapsulatedAPDU[length + 3] = calc_LRC(encapsulatedAPDU, length+3); // LRC (Longitudinal Redundancy Check)
  
  Ns ^= 0x40; // Alterna o bit Ns (Send-sequence bit) para o próximo envio

  return encapsulatedAPDU; // Retorna o APDU encapsulado
    
}  

/**
  * @brief  Envia um APDU e retorna a resposta em String e em vetor de bytes
  * @param  data: Ponteiro para o array de bytes
  * @param  length: Tamanho do array de bytes
  * @param  response_buffer: Ponteiro para o buffer onde a resposta será armazenada
  * @param  response_buffer_size: Tamanho do buffer de resposta
  * @retval response: Retorna a resposta recebida como uma String
*/
String sendAPDU(uint8_t* apdu, size_t length, uint8_t* response_uint, size_t response_uint_size) {
  String response = ""; // Variável para armazenar a resposta

  digitalWrite(SOFT_SERIAL_PIN, HIGH);  // Garante que ao trocar o pino de Rx para Tx, o pino continue em nível alto
  pinMode(SOFT_SERIAL_PIN, OUTPUT);     // Define o pino como saída
  softSerial.begin(PWM_FREQUENCY1*Di/Fi, SWSERIAL_8E1,-1,SOFT_SERIAL_PIN); //Define o pino como Tx
  pin_flag();
  softSerial.write(apdu, length);
  softSerial.begin(PWM_FREQUENCY1*Di/Fi, SWSERIAL_8E1,SOFT_SERIAL_PIN,-1); //Define o pino como Rx de volta
  delay(300);
  
  memset(response_uint, 0, response_uint_size); // Limpa o buffer de resposta
  uint8_t index = 0;
  // Lê a resposta da softSerial
  while(softSerial.available()) {
      uint8_t c = softSerial.read();
      if(index < response_uint_size) {
        response_buffer[index++] = c; // Armazena o byte lido no buffer
      }else{
        break; // Evita overflow do buffer
      }

      // Formata o byte recebido como hexadecimal com dois dígitos
      if (c < 0x10) {
        response += "0"; // Adiciona zero à esquerda se necessário
      }
      response += String(c,HEX);
  }

  return response; // Retorna a resposta recebida
}

/**
  * @brief  Imprime a string recebida colocando um espaço a cada 2 dígitos, para facilitar a leitura
  * @param  apdu: String contendo os dados a serem impressos
  * @retval None
*/
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

/**
  * @brief  Desliga o smartcard seguindo a sequência obrigatória do ISO7816
  * @retval None
*/
void powerDownSmartCard(){
  //PowerDown ISO7816 sequence
  sendSPI(0x85);              // Reset = Low
  delay(10);
  sendSPI(0x81);              // Clk = Low
  delay(90);
  configureVoltage(0);        // Card_VCC = Low
}

/**
  * @brief  Reseta o smartcard e lê o ATR
  * @param  atr: ponteiro para matriz de uint8_t onde será armazenado o ATR
  * @param  maxLen: tamanho máximo da matriz atr
  * @retval length: quantidade de bytes lidos no ATR
*/
uint32_t resetSmartCard(uint8_t* atr, size_t maxLen){
  pin_flag();
  configureVoltage(18); //Liga smartCard - Define tensão do smart card: 18=1.8V
  delay(200);

  sendSPI(0x85);  // CLKdiv = 1 - Liga o clock do smartcard
  softSerial.begin(PWM_FREQUENCY1/372, SWSERIAL_8E1,SOFT_SERIAL_PIN,-1); // Inicializa a comunicação serial emulada como receptor, com baud rate proporcional à frequência do PWM
  delay(1000);
  sendSPI(0x95);  // CLKdiv = 1    RESET=1 - Reseta o smartcard
  delay(150);

  //Leitura de ATR
  size_t atrLen = 0;
  if (softSerial.available()) {
    while(softSerial.available() && atrLen < maxLen) {
      atr[atrLen++] = (uint8_t)softSerial.read();
    }
    Serial.print("ATR Received: ");
    for (size_t i = 0; i < atrLen; i++) {
      if (atr[i] < 0x10) Serial.print("0");
      Serial.print(atr[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("ERROR: No ATR received.");
  }

  return atrLen; // Retorna o número de bytes lidos
}


/**
  * @brief  Monta, envia, e valida o PPS request para protocolo T=1
  * @param  p_atr: ponteiro para a estrutura do ATR
  * @retval Resultado da operação (1=OK) (0=Erro)
  */
uint8_t pps_request(ATR_TypeDef* p_atr){
  uint8_t PPS_buffer[4] = {0xFF, 0x11, 0x45, 0xAA}; // Estrutura base do PPS

  if (p_atr->ib[0][ATR_INTERFACE_BYTE_TA].present){
    /* If the card supports values different from default values */
    if (p_atr->ib[0][ATR_INTERFACE_BYTE_TA].value != 0x11)
    {
      /* PPS1 */
      PPS_buffer[2] = p_atr->ib[0][ATR_INTERFACE_BYTE_TA].value;
    }
  }
  PPS_buffer[3] = calc_LRC(PPS_buffer, sizeof(PPS_buffer)-1);

  sendAPDU(PPS_buffer, sizeof(PPS_buffer), response_buffer, sizeof(response_buffer));

  // Verifica se a resposta é igual ao PPS enviado
  if (memcmp(PPS_buffer, response_buffer, sizeof(PPS_buffer)) == 0) {
    Serial.println("PPS accepted by card.");
    return 1; // PPS aceito
  } else {
    Serial.println("PPS rejected by card.");
    return 0; // PPS rejeitado
  }
}

uint8_t ifs_request(ATR_TypeDef* p_atr){
    //Envia um IFS Request e imprime a resposta
    uint8_t S_block_apdu[] = {0x00, 0xC1, 0x01, 0xFE, 0x00};
    S_block_apdu[4] = calc_LRC(S_block_apdu, sizeof(S_block_apdu)-1);
    String response0 = sendAPDU(S_block_apdu, sizeof(S_block_apdu), response_buffer, sizeof(response_buffer));
    Serial.print("IFS ");
    printAPDU(response0);
  
  
  uint8_t IFS_buffer[4] = {0xFF, 0x11, 0x45, 0xAA}; // Estrutura base do PPS

  memset(response_buffer, 0, sizeof(response_buffer)); // Limpa o buffer de resposta
  sendAPDU(IFS_buffer, sizeof(IFS_buffer), response_buffer, sizeof(response_buffer));

  // Verifica se a resposta é igual ao IFS enviado
  if (memcmp(IFS_buffer, response_buffer, sizeof(IFS_buffer)) == 0) {
    Serial.println("IFS accepted by card.");
    return 1; 
  } else {
    Serial.println("IFS rejected by card.");
    return 0; 
  }
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

  Serial.begin(115200); // Inicializa comunicação serial com o terminal de interface usuário
  
  // Configuração do canal PWM (LEDC)
  ledcSetup(0, PWM_FREQUENCY1, PWM_RESOLUTION);
  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(CLK_NCN, 0);
  // Ativa o clock com duty cycle 50%
  enableFreeRunningClock(1);

  // Associar a função de interrupção ao pino INT
  attachInterrupt(digitalPinToInterrupt(INT_NCN), intInterrupt, FALLING);
  
  SPI.begin(CLK_SPI,MISO,MOSI,CS);  // Begin SPI 

  Serial.println("Initiate");
  Serial.println();
  
  powerDownSmartCard(); //Desliga o smartcard para garantir que ele esteja desligado antes de iniciar
  delay(1000);
  uint8_t atrBuffer[50];
  uint32_t atr_size = resetSmartCard(atrBuffer, sizeof(atrBuffer)); // Reseta o smartcard e lê o ATR

  uint8_t decodeATRResult = ATR_Decode(&atr_struct, atrBuffer, atr_size);

  if(decodeATRResult == ATR_MALFORMED){
    // Artificial ATR for debugging    
    uint8_t atr_artificial[] = { 0x3B, 0xFD, 0x96, 0x00, 0x00, 0x80, 0x31, 0xFE, 0x45, 0x53, 0x4C, 0x4A, 0x35, 0x32, 0x47, 0x78, 0x78, 0x79, 0x79, 0x79, 0x7A, 0x52, 0x25};
		ATR_Decode(&atr_struct, atr_artificial, sizeof(atr_artificial)); //Considera o ATR artificial caso ocorra erro no ATR real(APENAS PARA DEBUG)
	}

  pps_request(&atr_struct); // Envia o PPS request
  ifs_request(&atr_struct); // Envia o IFS request

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
      uint8_t atrBuffer[50];
      size_t atr_size = resetSmartCard(atrBuffer, sizeof(atrBuffer)); // Reseta o smartcard e lê o ATR
      uint8_t decodeATRResult = ATR_Decode(&atr_struct, atrBuffer, atr_size);

      uint8_t atr_artificial[] = { // Artificial ATR for debugging
        0x3B, 0xFD, 0x96, 0x00, 0x00, 0x80, 0x31, 0xFE, 0x45, 0x53, 0x4C, 0x4A,
        0x35, 0x32, 0x47, 0x78, 0x78, 0x79, 0x79, 0x79, 0x7A, 0x52, 0x25};
      if(decodeATRResult == ATR_MALFORMED){
        ATR_Decode(&atr_struct, atr_artificial, sizeof(atr_artificial)); //Considera o ATR artificial caso ocorra erro no ATR real(APENAS PARA DEBUG)
      }
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
      uint8_t apdu[length];
      for (size_t i = 0; i < length; i++) {
        String byteString = input.substring(i * 2, i * 2 + 2); // Extrai dois caracteres
        apdu[i] = (uint8_t)strtol(byteString.c_str(), nullptr, 16); // Converte para byte
      }

      // Envia a APDU e imprime a resposta
      Serial.print("Sending I-Block: ");
      uint8_t* apdu_encapsulated = encapsulateIBlock(apdu, length); // Envia a APDU encapsulada
      for (size_t i = 0; i < length + 4; i++) { // length + 4 porque o encapsulamento adiciona 4 bytes
        if (apdu_encapsulated[i] < 0x10) {
          Serial.print("0"); // Adiciona o zero à esquerda
        }
        Serial.print(apdu_encapsulated[i], HEX); // Imprime cada byte em hexadecimal
        Serial.print(" "); // Adiciona um espaço entre os bytes
      }
      Serial.println(); // Finaliza a linha
    
      soft_serial_received = sendAPDU(apdu_encapsulated, length+4, response_buffer, sizeof(response_buffer)); // Envia o APDU encapsulado e recebe a resposta
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
      uint8_t nr = (input.charAt(1) & 0x01) << 4;
      nr |= 0x80;
      //Envia um R-block - Envia o N(R) bit recebido
      uint8_t R_block_apdu[4] = {0x00, nr, 0x00, 0x00};   
      R_block_apdu[3] = calc_LRC(R_block_apdu, sizeof(R_block_apdu)-1); // Calcula o LRC para o R-Block
      String response0 = sendAPDU(R_block_apdu, sizeof(R_block_apdu),response_buffer, sizeof(response_buffer));
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
    uint8_t apdu[length];
    for (size_t i = 0; i < length; i++) {
      String byteString = input.substring(i * 2, i * 2 + 2); // Extrai dois caracteres
      apdu[i] = (uint8_t)strtol(byteString.c_str(), nullptr, 16); // Converte para byte
    }
    // Envia a APDU e imprime a resposta
    soft_serial_received = sendAPDU(apdu, length, response_buffer, sizeof(response_buffer));
    printAPDU(soft_serial_received);
    
  }

  delay(50);
}

