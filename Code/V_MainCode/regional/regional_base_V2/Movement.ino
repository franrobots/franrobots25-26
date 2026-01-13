
// Pressionado = LOW (pinos com INPUT_PULLUP)
static inline bool readSwitchDebounced(uint8_t pin) {
  uint8_t hits = 0;
  for (int i = 0; i < 3; ++i) {
    hits += (digitalRead(pin) == HIGH);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return (hits >= 2); // maioriaHIGH
}

bool moveSwitch() {
  const bool leftPressed  = readSwitchDebounced(SWITCHLEFT);
  const bool rightPressed = readSwitchDebounced(SWITCHRIGHT);
  if(!noSwitch){
  if (leftPressed && !rightPressed) {
    Serial.println("Choque esquerdo");

    stopTank();
    //gyro.resetCoordinatesValues(position);
    moveTank(-100, -100, false);
    vTaskDelay(pdMS_TO_TICKS(50));
    //stopTank();
    axisCurve(10);
    vTaskDelay(pdMS_TO_TICKS(10));
    moveTank(100, 100, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    //gyro.resetCoordinatesValues(position);
    //alignTile();
    return true;

  } else if (rightPressed && !leftPressed) {
    Serial.println("Choque direito");

    stopTank();
    //gyro.resetCoordinatesValues(position);
    moveTank(-100, -100, false);
    vTaskDelay(pdMS_TO_TICKS(50));
    //stopTank();
    axisCurve(-10);
    vTaskDelay(pdMS_TO_TICKS(10));
    moveTank(100, 100, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    //gyro.resetCoordinatesValues(position); 
    //alignTile();
    return true;
  }
  return false; // redundante, mas explícito
  }
  return false; // para o if do switch
}

inline int8_t sgn_i(long x) { return (x > 0) - (x < 0); } // -1, 0, +1
  
void moveTile()
  {
    bool walkByCoordinate = true;
    Point ponto_atual = robot.getActualPoint();
    //SerialBT.println(robot.pointToIndex(ponto_atual));
    if (no)
      robot.addVisitNodesTimes(robot.pointToIndex(ponto_atual));
    if (no){
      if (getColor() == "blue"){
        robot.blueNodes.append(robot.pointToIndex(ponto_atual));
        stopTank();
        //blink_led(5, 3, true);
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
      if (getColor() == "grey"){
        //blink_led(2, 4, false);
        checkpoint = ponto_atual;
      }
    }
    uint8_t decision = path_decision();
    if (decision != 0){
      if (no) alignGyro();
      if (no) alignTile();
      switch (decision){
      case 1:
        //if (no) camera_Identify();
        if (no) turn_right();
        break;
      case 2:
        //if (no) camera_Identify();
        if (no) turn_left();
        break;
      case 3:
        if (no) moveTank(150, 150, false);
        if (no) vTaskDelay(pdMS_TO_TICKS(800)); //800
        if (no) stopTank();
        if (no) vTaskDelay(pdMS_TO_TICKS(200));
        if (no) gyro.resetCoordinatesValues(position);
        if (no) moveTank(-150, -150, false);
        if (no) vTaskDelay(pdMS_TO_TICKS(250)); //600
        if (no) turn_left();
        if (no) turn_left();
        break;
      case 4: // Caso especifico para buraco na frente do robô e duas paredes
        if (no) turn_left();
        if (no) turn_left();
        break;
    }
    walkByCoordinate = false;
    closerToWall = true; 
  }

    if (closerToWall) {
      if (no) axisCurve(getNextTileAngle());
      walkByCoordinate = false;
    }   
    
    
  if (walkByEncoder(30, walkByCoordinate)) { // Trocar valor de gap entre os ladrilhos (valor de movimentação)
    if (no) moveTank(-255, -255, false);
    if (no) vTaskDelay(pdMS_TO_TICKS(800)); // Delay de ré do ladrilho preto // 650
    if (no) stopTank();
    uint16_t black_index = robot.getRelativeNeighborhood(position).getByIndex(0);
    if (no) robot.blackNodes.append(black_index);
  } else {
    if (no) {
      if (position == 0) robot.move("up");
      else if (position == 1) robot.move("right");
      else if (position == 2) robot.move("down");
      else if (position == 3) robot.move("left");
      //tile_count++;
    }
  }
}

bool camera_Identify(){
    VictimData vd;
    if (cameraIgnore){//contrl variavel para quando retornar
    if (xQueueReceive(cameraQueue, &vd, 0) == pdTRUE) {
      stopTank();
      if(vd.esq > 0 ) {
        release_kits(convert_victim_code(vd.esq), false);  // false = lado esquerdo
      }
      if(vd.dir > 0){
        release_kits(convert_victim_code(vd.dir), true);  // true = lado direito
      }
      return true;
    }
    }
    return false;
}

bool walkByEncoder(long cm, bool flag) {
  lastencoder = Encoder(); // ZERA no começo
  int16_t off_angle = flag ? gyro.coordinatesValues[gyro.getAngleToNearmostCoordinate(1)] : gyro.getYawAngle();
  uint8_t speed = 255; // 255
  while (Encoder() - lastencoder < cm ) {
    if (moveSwitch()) {moveSwitch();
    }else{
   if (camera_Identify()) continue;
    int16_t target_angle = gyro.getYawAngle(off_angle);
    pdControl(-target_angle, speed, 16.0, 0, MAX_PWM);
    if (getColor() == "black") return true;
    if (!no) break;
    
    taskYIELD();
    }
  }
  stopTank();
  lastencoder = Encoder(); //zera encoder
  return false;
}

uint8_t path_decision() {
  closerToWall = false;
  bool possibilities[4] = { 0, 0, 0, 1 };  // vetor para identificar os possiveis caminhos sem parede
  uint8_t index_vector[] = { 0, 6, 1 };    //frente, direita, esquerda
  uint8_t index_vector2[] = { 7, 5, 2 };   //frente, direita, esquerda
  for (uint8_t i = 0; i < 3; i++) {
    int sensor_value = read_sensors(index_vector[i]);
    int sensor_value2 = read_sensors(index_vector2[i]);
    if (sensor_value > NEAR_WALL || sensor_value2 > NEAR_WALL) {
      possibilities[i] = 1;
    } else if (i != 0) {
      if (sensor_value > 60) closerToWall = true; // 90
    }
  }
  //SerialBT.print.print("possibilities: "), printVector(possibilities, 4);
  uint8_t vetor_priority[4] = { 0, 1, 3, 2 };  //front, right, left, back //0 = front, 1 = right, 2 = back, 3 = left // define a ordem de prioridade
  //ele toma a decisão baseado no ladrilho menos visitado
  uint16_t vector_indexes[4];
  for (uint8_t i = 0; i < 4; i++) vector_indexes[i] = robot.getRelativeNeighborhood(position).getByIndex(vetor_priority[i]);
  //printVector(vector_indexes, 4);
  int8_t vector_decision[] = { 0, 0, 0, 0 };
  for (uint8_t i = 0; i < 4; i++) vector_decision[i] = robot.getVisitNodesTimes(vector_indexes[i]);
  //printVector(vector_decision, 4);
  uint8_t min_value = 90;  
  uint8_t min_index = 3;
  for (int8_t i = 2; i >= 0; i--) {
    if (possibilities[i] && !robot.blackNodes.contains(vector_indexes[i])) {
      if (vector_decision[i] <= min_value) {
        min_index = i;
        min_value = vector_decision[i];
      }
    }
  }
  globalIndex = vector_indexes[min_index];
  //SerialBT.print.println(robot.blackNodes.contains(vector_indexes[0]));
  if(min_index == 3 && robot.blackNodes.contains(vector_indexes[0])){
    return 4;
  }
  //SerialBT.println(min_index);
  return min_index;
}

void analog_write(uint8_t channel, uint8_t value) {
  ledcWrite(channel, value < 255 ? value : 255);
}

int16_t motorValueCorrection(int value) {
  return (abs(value) < MIN_PWM) ? -2 * MIN_PWM + value : value;
}

void moveTank(int left_value, int right_value, bool correctionFlag) {
  //camera_Identify();
  if (correctionFlag) {
    left_value = motorValueCorrection(left_value);
    right_value = motorValueCorrection(right_value);
  }
  if (left_value < 0) {
    analog_write(BACK_1, 0); // 0 → 2 → troca com 3 → vira 1
    analog_write(BACK_3, 0); // 1 → 3
    analog_write(FORWARD_1, min(abs(left_value), MAX_PWM));
    analog_write(FORWARD_3, min(abs(left_value), MAX_PWM));
  } else {
    analog_write(BACK_1, min(abs(left_value), MAX_PWM));
    analog_write(BACK_3, min(abs(left_value), MAX_PWM));
    analog_write(FORWARD_1, 0);
    analog_write(FORWARD_3, 0);
  }
  // Motores da direita (motores 2 → 0, 3 → 2)
  if (right_value < 0) {
    analog_write(BACK_0, 0); // 2 → 0
    analog_write(BACK_2, 0); // 3 → 1 → troca com 0 → vira 2
    analog_write(FORWARD_0, min(abs(right_value), MAX_PWM));
    analog_write(FORWARD_2, min(abs(right_value), MAX_PWM));
  } else {
    analog_write(BACK_0, min(abs(right_value), MAX_PWM));
    analog_write(BACK_2, min(abs(right_value), MAX_PWM));
    analog_write(FORWARD_0, 0);
    analog_write(FORWARD_2, 0);
  }
}

void stopTank() {
  for (uint8_t i = 0; i < motor_length; i++) {
    analog_write(motor_vector[i], 0);
  }
}

void pdControl(int16_t error, int16_t userPowerDefault, float userKp, float userKd, uint8_t correctionLimit) {
  float pd = userKp * error + userKd * (error - lastError);  // Compute PD Control value
  lastError = error;                                         // Update last error value (global variable)
  int16_t leftPower = userPowerDefault + pd;                 // Compute the leftPower (positive logic)
  int16_t rightPower = userPowerDefault - pd;                // Compute the rightPower (negative logic)
  leftPower = abs(leftPower) <= correctionLimit ? leftPower : int(leftPower / abs(leftPower)) * correctionLimit;
  rightPower = abs(rightPower) <= correctionLimit ? rightPower : int(rightPower / abs(rightPower)) * correctionLimit;
  //Serial//SerialBT.print.print(error), Serial//SerialBT.print.print("   "), Serial//SerialBT.print.print(leftPower), Serial//SerialBT.print.print("   "), Serial//SerialBT.print.println(rightPower);
  moveTank(leftPower, rightPower, 0);
}

void pdControlCurve(int16_t error, float userKp, float userKd, uint8_t correctionLimit) {
  float pd = userKp * error + userKd * (error - lastError);
  lastError = error;
  int16_t leftPower  = (int16_t)pd;    // positivo
  int16_t rightPower = (int16_t)(-pd); // negativo
  // injeta MIN_PWM no sentido do sinal, só se houver sinal (sgn != 0)
  int8_t sl = sgn_i(leftPower);
  int8_t sr = sgn_i(rightPower);
  if (sl != 0) leftPower  = (int16_t)(sl * MIN_PWM + leftPower);
  if (sr != 0) rightPower = (int16_t)(sr * MIN_PWM + rightPower);
  // saturação simétrica sem x/abs(x)
  if (abs(leftPower)  > correctionLimit) leftPower  = (int16_t)(sgn_i(leftPower)  * correctionLimit);
  if (abs(rightPower) > correctionLimit) rightPower = (int16_t)(sgn_i(rightPower) * correctionLimit);
  moveTank(leftPower, rightPower, 0);
}

void alignTile() {
  camera_Identify();
  const uint8_t correctionLimit = 160;   // 160
  const float   local_kp = 25.0f; // 25
  // frente(7) e trás(3)
  const int idx_front_back[2] = {7, 3};
  int v_front_back[2] = { read_sensors(idx_front_back[0]), read_sensors(idx_front_back[1]) };
  // frente(0) e trás(4) "de referência" para sanity check
  const int idx_ref_fb[2] = {0, 4};
  int v_ref_fb[2] = { read_sensors(idx_ref_fb[0]), read_sensors(idx_ref_fb[1]) };
  // qual lado está mais próximo (menor distância): front (-1) ou back (+1)
  // (mantém sua convenção: frente -1, trás +1)
  int8_t sign = (v_front_back[0] < v_front_back[1]) ? -1 : +1;
  // sanity check: se a diferença entre leituras front-frontRef (ou back-backRef) for grande, aborta
  if (sign == -1) { // frente
    if (abs(v_front_back[0] - v_ref_fb[0]) > 70) return; // 70..100
  } else {           // trás
    if (abs(v_front_back[1] - v_ref_fb[1]) > 70) return;
  }
  const float n_tile_math = (sign == -1) ? (v_front_back[0] / 300.0f) : (v_front_back[1] / 300.0f);
  uint8_t n_tile = (uint8_t)n_tile_math + ((n_tile_math - (int)n_tile_math) < 0.6f ? 0 : 1);
  if (n_tile > 2) n_tile = 2;
  const uint8_t chosen = (sign == +1) ? 3 : 7;
  int err0 = (int)sensors_target_value[n_tile] - read_sensors(chosen);
  int8_t sign_prev = sgn_i(err0);
  int speed = 0;
  uint8_t flips = 0;
  if (n_tile < 2) { // mantém seu filtro de proximidade
    for (;;) {
      int error = sign * ((int)sensors_target_value[n_tile] - read_sensors(chosen));
      speed = (int)(local_kp * error);
      int8_t s = sgn_i(speed);
      if (s != 0 && s != sign_prev) {
        flips++;
        sign_prev = s;
      }
      if (abs(speed) > correctionLimit) speed = sgn_i(speed) * correctionLimit;
      moveTank(speed, speed, false);
      if (flips > 2) break;
      if (!no)       break;
      // vTaskDelay(1);
    }
  }
  stopTank();
}

int getNextTileAngle() {
  int angle;
  uint16_t middle_wall_value = 150; // 100
  uint16_t d1 = 70;
  int vector[2] = { 1, 5 };  // esquerda, direita
  for (uint8_t i = 0; i < 2; i++) vector[i] = read_sensors(vector[i]);
  //printVector(vector, 2);
  int choosen_sensors;
  bool side;
  if (vector[0] > vector[1]) {
    choosen_sensors = vector[1];
    side = false;  // false for right sensor
  } else {
    choosen_sensors = vector[0];
    side = true;
  }                                      // true for left sensor
  if (choosen_sensors > 8000) return 0;  // faz ignorar o movimento se o sensor ler uma medida errada
  uint8_t n_tile = int(choosen_sensors / 300.0);
  if (n_tile >= 3) return 0; // 2
  // SerialBT.print(n_tile), SerialBT.print(" | "), SerialBT.println(choosen_sensors);
  float ref = (n_tile * 300 + middle_wall_value - (d1 + choosen_sensors)) / 300.0; // Fazer define dps
  float arctang = atan(ref);
  int arctan = int(arctang * 180 / 3.14159265359);
  //Serial.print("Ref= "), Serial.print(ref), Serial.print("  |  "), Serial.print(arctang), Serial.print("  |  "), Serial.println(arctan);
  if (side) angle = (choosen_sensors < (n_tile + 1) * 300) ? arctan : -arctan;
  else angle = (choosen_sensors < (n_tile + 1) * 300) ? -arctan : arctan;
  //Serial.print(choosen_sensors), Serial.print(", Angle ----> "), Serial.print(angle), Serial.print(" | "), Serial.println(arctan);
  return abs(angle) < 20 ? angle : 0;
}

void axisCurve(int8_t angle, float userKp, uint8_t correctionLimit) {
  //Serial.println(angle);
  uint8_t count = 0;
  int16_t off_angle = gyro.getYawAngle();
  //Serial.println(off_angle);
  while (count <= 20) { // 17 
    if (!no) break;
    int16_t error = angle - gyro.getYawAngle(off_angle);
    //Serial.println(error);
    if (error == 0) {
      count++;
    } else {
      count = 0;
      pdControlCurve(error, userKp, 0, correctionLimit);
    }
  }
  stopTank();
}

void turn_right() {
  camera_Identify();
  alignGyro();
  alignTile();
  int8_t index = (position + 1) % 4;
  int16_t angle = gyro.convertGyro(gyro.coordinatesValues[index] - gyro.getYawAngle(), 0);
  axisCurve(angle);
  alignGyro();
  //update_position(1);
  position = gyro.getAngleToNearmostCoordinate(1);
  alignTile();
}

void turn_left() {
  camera_Identify();
  alignGyro();
  alignTile();
  int8_t index = (position - 1 < 0 ? 3 : position - 1) % 4;
  int16_t angle = gyro.convertGyro(gyro.coordinatesValues[index] - gyro.getYawAngle(), 0);
  axisCurve(angle);
  alignGyro();
  //update_position(0);
  position = gyro.getAngleToNearmostCoordinate(1);
  alignTile();
}

void alignGyro() {
  axisCurve(gyro.getAngleToNearmostCoordinate());
  stopTank();
}

void update_position(bool direction) {  // 1 right, 0 left
  position = direction ? position + 1 : position - 1;
  position = position > 3 ? 0 : position < 0 ? 3 : position;  //0 north, 1 east, 2 south, 3 west
}

void return2init() {
  List teste = robot.getInstructions();
  blink_led(5, 4, true);
  for (uint8_t i = 0; i < teste.len(); i++) {
    if (getColor() == "blue") {
      //blink_led(5, 3, true);
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
    bool turnFlag = false;
    switch (teste.getByIndex(i)) {
      case 1:
        turn_right();
        turnFlag = true;
        break;
      case 2:
        turn_left();
        turn_left();
        turnFlag = true;
        break;
      case 3:
        turn_left();
        turnFlag = true;
        break;
    }
    if (turnFlag) {
      axisCurve(getNextTileAngle());
    }
    walkByEncoder(30, true); // Mesmo valor do encoder de cima, de def 
  }
  stopTank();
  blink_led(10, 4, true);
  vTaskDelay(pdMS_TO_TICKS(20000));
}

void testeServo(){
  servo.write(90);
  delay(5000);
  servo.write(55);
  delay(1000);
  servo.write(90);
  delay(1000);
  servo.write(120);
  delay(1000);
}
