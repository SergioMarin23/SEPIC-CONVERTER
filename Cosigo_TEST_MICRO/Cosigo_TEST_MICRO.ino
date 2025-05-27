// Definición del pin de testeo
#define TEST_MICRO_PIN  4  // Pin de prueba

void setup() {
  // Inicializa el pin como salida
  Serial.begin(115200);
  pinMode(TEST_MICRO_PIN, OUTPUT);
}

void loop() {
  // Encender el pin
  digitalWrite(TEST_MICRO_PIN, HIGH);
  delay(500);
  Serial.print("LED ON");   // Esperar 500 ms

  // Apagar el pinu
  digitalWrite(TEST_MICRO_PIN, LOW);
  delay(500);  // Esperar 500 ms
  Serial.print("LED OFF"); 
}