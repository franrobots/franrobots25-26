#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Endereço que você usa:
#ifndef BNO055_ADDR
#define BNO055_ADDR 0x28
#endif

class BNO055 {
private:
  // Evento do Adafruit Unified Sensor
  sensors_event_t event;

  // Constrói o driver oficial: (sensorID, address, wire)
  Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);

  // Sentinela para "sem offset explícito"
  static constexpr int16_t NO_OFF = INT16_MIN;

  // Offsets (em graus inteiros) capturados no resetCoordinatesValues()
  int16_t offset_y = 0;  // para inclinação (roll)
  int16_t offset_x = 0;  // para yaw/heading

  // ---------- Helpers ----------
  static int16_t roundDeg(float v) {
    // converte float -> int16 mantendo 0..360 wrap
    long d = lroundf(v);                    // arredonda
    d %= 360; if (d < 0) d += 360;          // 0..359
    return (int16_t)d;
  }

  static int16_t shortestSignedDelta(int16_t valueDeg, int16_t offsetDeg) {
    // retorna diferença no MENOR ARCO, em [-180, +180]
    int16_t v = valueDeg - offsetDeg;
    while (v > 180)  v -= 360;
    while (v < -180) v += 360;
    return v;
  }

  // Converte valor absoluto em graus para delta (menor arco) já com offset
  static int16_t convertGyro(int16_t valueAbsDeg, int16_t offsetDeg) {
    int16_t d = shortestSignedDelta(valueAbsDeg, offsetDeg);
    // mantém sinal; se d==0 retorna 0
    return d;
  }

public:
  // Armazena 4 alvos (N, L, S, O) relativos ao heading atual após resetCoordinatesValues()
  int16_t coordinatesValues[4] = {0, 0, 0, 0};

  BNO055() {}
  ~BNO055() {}

  void begin() {
    Serial.print("BNO STARTING SETUP...");
    bool ok = bno.begin(OPERATION_MODE_IMUPLUS);  // modo IMU (gyro+acc)
    if (ok) {
      bno.setExtCrystalUse(true);                 // melhora estabilidade se o módulo tiver XTAL
      Serial.println(" BNO SETUP COMPLETE!");
    } else {
      Serial.println(" BNO DO NOT WORKING!");
    }
    delay(100);
  }

  // Define offsets a partir da orientação atual e popula 4 alvos a cada 90°
  // 'compass' permite rotacionar o vetor [N,L,S,O] se você quiser trocar a referência
  void resetCoordinatesValues(int8_t compass = -1) {
    bno.getEvent(&event);
    if (compass < 0) {
      compass  = 0;
      offset_x = roundDeg(event.orientation.x);  // heading
      offset_y = roundDeg(event.orientation.y);  // roll
    }

    // Base = heading atual; gera N, L, S, O a cada 90°
    int16_t base = getYawAngle(); // já retorna delta (0 aqui, pois offset foi setado)
    for (uint8_t i = 0; i < 4; i++) {
      int16_t value = base + 90 * i;   // alvo absoluto em relação ao heading atual
      // normaliza para 0..359
      while (value >= 360) value -= 360;
      while (value < 0)    value += 360;
      // converte para delta relativo ao offset_x
      coordinatesValues[(i + compass) % 4] = convertGyro(value, 0); // 0 pois 'value' já é relativo ao base
    }
  }

  // Retorna YAW (heading) como delta relativo ao offset_x (se offset==NO_OFF, usa só offset_x;
  // se fornecer 'offset', soma ao offset_x — útil pra comparar com um alvo predefinido)
  int16_t getYawAngle(int16_t offset = NO_OFF) {
    bno.getEvent(&event);
    int16_t x = roundDeg(event.orientation.x); // heading absoluto 0..359
    int16_t off = (offset == NO_OFF) ? offset_x : (int16_t)(offset_x + offset);
    return convertGyro(x, off);
  }

  // Inclinação (roll) como delta relativo ao offset_y; invertido para manter sua convenção
  int16_t getInclinationAngle(int16_t offset = NO_OFF) {
    bno.getEvent(&event);
    int16_t y = roundDeg(event.orientation.y); // roll absoluto 0..359
    int16_t off = (offset == NO_OFF) ? offset_y : (int16_t)(offset_y + offset);
    return (int16_t)(-convertGyro(y, off));
  }

  // Retorna o ângulo (delta) até a coordenada cardinal mais próxima
  // choose=false => retorna delta (graus) até o alvo mais próximo (sinal indica lado)
  // choose=true  => retorna o índice do alvo [0..3] mais próximo
  int16_t getAngleToNearmostCoordinate(bool choose = false) {
    int16_t deltas[4];
    for (uint8_t i = 0; i < 4; i++) {
      deltas[i] = getYawAngle(coordinatesValues[i]); // delta para cada alvo
    }
    uint16_t minAbs = (uint16_t)abs(deltas[0]);
    uint8_t idx = 0;
    for (uint8_t i = 1; i < 4; i++) {
      uint16_t a = (uint16_t)abs(deltas[i]);
      if (a < minAbs) { minAbs = a; idx = i; }
    }
    return choose ? idx : (int16_t)(-deltas[idx]); // mantém sua convenção de sinal invertido
  }

  // Checa calibração; mantém sua lógica (gyro==3)
  bool isWorking() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(sys); Serial.print(", ");
    Serial.print(gyro); Serial.print(", ");
    Serial.print(accel); Serial.print(", ");
    Serial.println(mag);
    return (gyro == 3);
  }
};

// Instância global compatível com seu código atual
BNO055 gyro;
