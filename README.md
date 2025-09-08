# esp32_pki_over_spi

-Interface entre microcontrolador ESP32 e SmartCards utilizando o gateway NCN6001-

Este programa tem o intuito de ler o ATR de SmartCards e trocar APDUs por meio do terminal serial. Utiliza o CI NCN6001 como gateway e um pino I/O
genérico para comunicação bidirecional emulando o funcionamento de uma UART half-duplex.