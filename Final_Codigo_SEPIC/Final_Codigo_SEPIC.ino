#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pines
#define PWM_OUT_PIN     6
#define PV_SENSE_PIN    10
#define Vout_SENSE_PIN  2
#define TEST_MICRO_PIN  4
#define SDA_PIN         8
#define SCL_PIN         9

// Constantes
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C

#define ANALOG_RESOLUTION 12
const float MAX_ADC = pow(2, ANALOG_RESOLUTION) - 1;

const float Vref_ADC_out = 24.0;
const float Vref_ADC_in_PV = 21.0;
const float Vref = 12.0;  // Voltaje deseado en salida

// Variables globales
float Vout_VALUE = 0.0;
float Vin_PV_VALUE = 0.0;

float D1 = 0.2;           // Duty inicial
int duty1 = 0;            // Valor PWM final a escribir

// PWM
const int PWM_RESOLUTION = 9;
const int FREQ_PWM = 100000;
const int PWM_MAX = pow(2, PWM_RESOLUTION) - 1;
const int LED_CHANNEL = 0;

//Parámetros PID
const float Kp = 0.4;    // Ganancia proporcional
const float Ki = 0.05;   // Ganancia integral
const float Kd = 0.01;   // Tiempo derivativo
const float Ts = 1.0 / FREQ_PWM;    // Tiempo de muestreo en segundos

// Coeficientes PID discretos
float q0, q1, q2;
float e_k = 0, e_k_1 = 0, e_k_2 = 0;  // Errores e(k), e(k-1), e(k-2)
float u_k = 0, u_k_1 = 0;             // Duty actual y anterior

// Pantalla
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  //Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializar pantalla OLED
  /*
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Error al iniciar pantalla OLED"));
    while (true); // Detener si falla
  }
  display.clearDisplay();
  display.display();
  */

  //INICIALIZAR PINES
  pinMode(PV_SENSE_PIN, INPUT);
  pinMode(Vout_SENSE_PIN, INPUT);
  pinMode(PWM_OUT_PIN, OUTPUT);
  pinMode(TEST_MICRO_PIN, OUTPUT);

  //INICIALIZAR ADC
  analogReadResolution(ANALOG_RESOLUTION);

  //INICIALIZAR PWM
  ledcAttach(PWM_OUT_PIN, FREQ_PWM, PWM_RESOLUTION);

  // Cálculo de coeficientes PID discretos

  q0 = Kp + (Ki * Ts) / 2 + (Kd / Ts);
  q1 = -Kp + (Ki * Ts) / 2 - (2 * Kd) / Ts;
  q2 = Kd / Ts;
  

  //VALIDAR MICRO
  digitalWrite(TEST_MICRO_PIN, HIGH);
}

void loop() {
  digitalWrite(TEST_MICRO_PIN, HIGH);

  // Leer ADC
  Vin_PV_VALUE = analogRead(PV_SENSE_PIN);    // Panel
  Vout_VALUE = analogRead(Vout_SENSE_PIN);    // Salida

  // Convertir a voltaje real
  Vin_PV_VALUE = Vin_PV_VALUE * (Vref_ADC_in_PV / MAX_ADC);
  Vout_VALUE = Vout_VALUE * (Vref_ADC_out / MAX_ADC);

  // === Control ===
  float error = Vref - Vout_VALUE;
  D1 += 0.02*error + 0.4;
  D1 = constrain(D1, 0.03, 0.95);  // Limitar duty a rango válido
  duty1 = D1 * PWM_MAX;

  // === Control PID incremental (desactivado) ===
  /*
  e_k = error;
  u_k = u_k_1 + q0 * e_k + q1 * e_k_1 + q2 * e_k_2;

  // Saturar u(k) dentro de los límites del PWM
  u_k = constrain(u_k, 0, 0.95 * PWM_MAX);
  duty1 = (int)u_k;

  // Actualizar variables para próxima iteración
  e_k_2 = e_k_1;
  e_k_1 = e_k;
  u_k_1 = u_k;
  */ 

  // Aplicar señal PWM
  ledcWrite(PWM_OUT_PIN, duty1);

  // Mostrar en la pantalla OLED
  //displayValues();

  // Imprimir por serial
  Serial.print("Vout: ");
  Serial.print(Vout_VALUE, 2);
  Serial.print(" V | PV: ");
  Serial.print(Vin_PV_VALUE, 2);
  Serial.print(" V | PWM: ");
  Serial.println(duty1);

  delay(100);
}


//Funciones extra

void displayValues() {
  // Mostrar en la pantalla OLED
  display.clearDisplay();
  display.setTextColor(WHITE);

  // Mostrar voltaje sensado de PV
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Volt. Panel: ");
  display.print(Vin_PV_VALUE, 1);
  display.println(" V");

  // Mostrar voltaje sensado de Salida
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.print("Volt. Salida: ");
  display.print(Vout_VALUE, 1);
  display.println(" V");

  // Mostrar el valor de PWM
  display.setCursor(0, 30);
  display.print("PWM: ");
  display.print(duty1);
  display.print(" / ");
  display.println(PWM_MAX);

  // Mostrar en la pantalla
  display.display();           
}
