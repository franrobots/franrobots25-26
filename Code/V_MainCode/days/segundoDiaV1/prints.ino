void printVector(bool* vector, uint8_t len) {
  //SerialBT.print(vector[0]);
  for (uint8_t i = 1; i < len; i++) {
    //SerialBT.print(" | ");
    //SerialBT.print(vector[i]);
  }
  //SerialBT.println();
}

void printVector(int* vector, uint8_t len) {
  //SerialBT.print(vector[0]);
  for (uint8_t i = 1; i < len; i++) {
    //SerialBT.print("  ");
    //SerialBT.print(vector[i]);
  }
  ////SerialBT.println();
}
/*
void printVector(int16_t* vector, uint8_t len) {
  //SerialBT.print(vector[0]);
  for (uint8_t i = 1; i < len; i++) {
    //SerialBT.print(" | ");
    //SerialBT.print(vector[i]);
  }
  //SerialBT.println();
}
*/

void printVector(uint16_t* vector, uint8_t len) {
  //SerialBT.print(vector[0]);
  for (uint8_t i = 1; i < len; i++) {
    //SerialBT.print(" | ");
    //SerialBT.print(vector[i]);
  }
  //SerialBT.println();
}

void printVector(int16_t* vector, uint8_t len) {
  Serial.print(vector[0]);
  for (uint8_t i = 1; i < len; i++) {
    Serial.print(" | ");
    Serial.print(vector[i]);
  }
  Serial.println();
}

// Leitura pura do GY
void printSensorsPure() {
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(read_sensors_pure(i)), Serial.print("  ");
    //Serial.print(read_sensors_pure(i)), Serial.print("  ");
  }
  Serial.println(" ");
  //Serial.println(" ");
}

// GY with offset
void printSensors() {
  for (uint8_t i = 0; i < 8; i++) {
    SerialBT.print(read_sensors(i)), SerialBT.print("  ");
  }
  SerialBT.println("");
}

// Reflectance plate 
void printReflectance() {
  readColorSensors();
  SerialBT.print("C9: ");   SerialBT.print(sensor_values[0]);
  SerialBT.print(" | R: "); SerialBT.print(sensor_values[1]);
  SerialBT.print(" | G: "); SerialBT.print(sensor_values[2]);
  SerialBT.print(" | B: "); SerialBT.println(sensor_values[3]);
  SerialBT.println(); // Quebra de linha após os dois valores

  // readColorSensors();
  // Serial.print("C9: ");   Serial.print(sensor_values[0]);
  // Serial.print(" | R: "); Serial.print(sensor_values[1]);
  // Serial.print(" | G: "); Serial.print(sensor_values[2]);
  // Serial.print(" | B: "); Serial.println(sensor_values[3]);
  // Serial.println(); // Quebra de linha após os dois valores
}

// Robot encoder
void printEncoder() {
  Serial.print("Distância percorrida: ");
  Serial.print(Encoder());
  Serial.println(" cm");
}

// Sensor BNO
void printGyro() {
  //Serial.printf(" Yaw Angle %d | Inclination Angle %d", gyro.getYawAngle(), gyro.getInclinationAngle());
  Serial.print(gyro.getYawAngle());
  //Serial.println(gyro.getInclinationAngle());
  Serial.println("  ");
}  
// Cameras 
void printCam() {
  VictimData vd;
  if (xQueueReceive(cameraQueue, &vd, 0) == pdTRUE) {
    if (vd.esq != 0 || vd.dir != 0) {
      Serial.printf("CÂMERA ESQ -> Vítima %d | CÂMERA DIR -> Vítima %d\n", vd.esq, vd.dir);
      Serial.println("---------------------------");
    }
  }
  // Delay opcional se esta função estiver sendo chamada em loop
  vTaskDelay(pdMS_TO_TICKS(100));
}

void printRefletanceResult() {
  SerialBT.print(getColor() == "black");
  SerialBT.print(getColor() == "blue");
  SerialBT.println(getColor() == "grey");
}
