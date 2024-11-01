const int lampPin = 7;  // Defina o pino onde a lâmpada (ou LED) está conectada

void setup() {
  Serial.begin(9600);  // Inicializa a comunicação serial
  pinMode(lampPin, OUTPUT);  // Define o pino como saída
  digitalWrite(lampPin, LOW);  // Garante que a lâmpada esteja desligada inicialmente
}

void loop() {
  if (Serial.available() > 0) {  // Verifica se há dados disponíveis para ler
    char command = Serial.read();  // Lê o comando enviado pelo Python

    if (command == '1') {  // Se o comando for '1'
      digitalWrite(lampPin, HIGH);  // Liga a lâmpada
    } 
    else if (command == '0') {  // Se o comando for '0'
      digitalWrite(lampPin, LOW);  // Desliga a lâmpada
    }
  }
}


