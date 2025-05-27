// === PINES ===
int Vout_Sense_Pin = 35;
int PWM_out = 25;

// === PWM ===
int pwm_value = 0;
const int FreqPWM = 100000;
const int resolucionPWM = 9;
const int PWM_MAX = pow(2, resolucionPWM) - 1;

// === ADC ===
const int ANALOG_RESOLUTION = 12;
const float Max_ADC = pow(2, ANALOG_RESOLUTION) - 1;
const float Vmax_ADC_escala = 24.0;

// === Control ===
float Vref = 12.0;
float Vout = 0.0;
float error = 0.0;

// === Protección ===
const float VOUT_MIN_UMBRAL = 0.1;
bool control_activo = false;

// === PID Incremental 1===
float Kc = 0.02;
float Ti = 0.1;
float Td = 0.001;
float Ts = 1.0 / FreqPWM;

// === PID Incremental 2===
/*
float Kp = 0.00096454;
float Ki = 192.9072;
float Kd = 0.0;
*/

// Coeficientes PID
float q0, q1, q2;
float e_k = 0, e_k_1 = 0, e_k_2 = 0;
float u_k = 0, u_k_1 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(Vout_Sense_Pin, INPUT);
  pinMode(PWM_out, OUTPUT);
  analogReadResolution(ANALOG_RESOLUTION);

  // Configurar PWM
  ledcAttach(PWM_out, FreqPWM, resolucionPWM);

  // Calcular coeficientes PID incrementales modo 1
  /*
  q0 = Kc * (1 + Ts/(2*Ti) + Td/Ts);
  q1 = -Kc * (1 - Ts/(2*Ti) + 2*Td/Ts);
  q2 = Kc * Td/Ts;
  */

  // Calcular coeficientes PID incrementales modo 2
  /*
  q0 = Kp + (Ki*Ts)/2 + Kd/Ts;
  q1 = -Kp + (Ki*Ts)/2 - (2*Kd)/Ts;
  q2 = Kd/Ts; 
  */

  // Impulso inicial de arranque
  pwm_value = (int)(0.2*PWM_MAX);
  ledcWrite(PWM_out, pwm_value);
  Serial.println(">>> PWM inicial aplicado para arranque <<<");
  delay(500);  // Espera para permitir que el voltaje suba

  control_activo = true;  // Habilitar control
}

void loop() {
  // Leer voltaje
  float adc_value = analogRead(Vout_Sense_Pin);
  Vout = ((adc_value * Vmax_ADC_escala) / Max_ADC);

  // Si el voltaje es muy bajo, aplicar PWM de arranque fijo
  if (!control_activo || Vout < VOUT_MIN_UMBRAL) {
    pwm_value = (int)(0.2 * PWM_MAX);  // Duty fijo para mantener actividad
    ledcWrite(PWM_out, pwm_value);
    Serial.print("⚠️ Vout bajo ("); Serial.print(Vout, 2);
    Serial.println(" V). Aplicando PWM base.");
    delay(100);
    return;
  }

  // === Calculo del error ===
  error = Vref - Vout;

  // === CONTROL PID INCREMENTAL ===
  /*
  e_k = error;
  float delta_U = q0*e_k + q1*e_k_1 + q2*e_k_2;

  if(delta_U > 0.3*PWM_MAX){
    delta_U = 2*(0.2 * PWM_MAX);    
  }

  if (u_k > 0.95 * PWM_MAX) {
    u_k = 0.95 * PWM_MAX;
  } else if (u_k < 0.03 * PWM_MAX) {
    u_k = 0.03 * PWM_MAX;
  }

  pwm_value = (int)u_k;

  // Actualizar historial
  e_k_2 = e_k_1;
  e_k_1 = e_k;
  u_k_1 = u_k;
  */

  // === Control proporcional alternativo ===
  
  float D1 = 0.02*error + 0.4;
  D1 = constrain(D1, 0.03, 0.95);
  pwm_value = (int)(D1 * PWM_MAX);
  

  // Modificación de ciclo útil del PWM
  pwm_value = constrain(pwm_value, 0, PWM_MAX);
  ledcWrite(PWM_out, pwm_value);

  // Mostrar por serial
  int pwm_percent = map(pwm_value, 0, PWM_MAX, 0, 100);
  Serial.print("Vout: "); Serial.print(Vout, 2);
  Serial.print(" V | Error: "); Serial.print(error, 2);
  Serial.print(" V | PWM: "); Serial.print(pwm_value);
  Serial.print(" | Duty: "); Serial.print(pwm_percent);
  Serial.println(" %");

  delay(100);
}
