#include <Arduino.h> //
#include <Adafruit_Sensor.h> //
#include <Adafruit_BNO055.h> //
//#include "Adafruit_VL53L0X.h" //

#include <BluetoothSerial.h> //

#include <MazeFranRobots.h> //
#include <Adafruit_NeoPixel.h> //
#include <ESP32Servo.h> //
#include <OpenMVCamera.h> //
#include <utility/imumaths.h> //
#include <Wire.h> //
#include <freertos/task.h> //
#include <VL53L0X.h> //
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Motor da Frente - Esquerda - 0
#define FORWARD_0 10
#define BACK_0 11
#define LEFT_MOTOR_1_0 32
#define LEFT_MOTOR_2_0 33
// Motor de Trás - Esquerda - 1
#define FORWARD_1 2
#define BACK_1 3
#define LEFT_MOTOR_1_1 27  
#define LEFT_MOTOR_2_1 16  
// Motor de Trás - Direita - 2
#define FORWARD_2 4
#define BACK_2 5
#define RIGHT_MOTOR_1_2 17 
#define RIGHT_MOTOR_2_2 12 
// Motor da Frente - Direita - 3
#define FORWARD_3 6
#define BACK_3 7
#define RIGHT_MOTOR_1_3 26
#define RIGHT_MOTOR_2_3 25
// Velocity to control motors
#define MIN_PWM 100
#define MAX_PWM 255
#define TURN_SPEED_DEFAULT 230
#define SERVO_PIN 5

#define SWITCHRIGHT 13
#define SWITCHLEFT 2

#define KIT_CENTER 90
#define KIT_LEFT 55
#define KIT_RIGHT 125

#define NEAR_WALL 180

#define LEDS_PIN 23

#define SENSOR_ldrR 34
#define SENSOR_ldrB 39
#define SENSOR_ldrG 36
#define SENSOR_c9 35

#define N_LEDS 6
#define BUTTON 4 //| 12
#define TCAADDR 0x70
#define BNO055_ADDR 0x28
#define GY_ADDR 0x29

#define CAM_LEFT 0x11
#define CAM_RIGHT 0x13

#define SDA_PIN 21
#define SCL_PIN 22
#define ENCODER_PIN 19

#define noSwitch false // true sem  switch - false com switch 
// --------------------- BNO Class -------------------- // 
class BNO055 {
private:
  sensors_event_t event;
  Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);

  int16_t offset_y = NULL;
  int16_t offset_x = NULL;
public:
  int16_t coordinatesValues[4] = { NULL, NULL, NULL, NULL };
  BNO055() {}
  ~BNO055() {}

  void begin() {
    Serial.print("BNO STARTING SETUP...");
    !bno.begin(OPERATION_MODE_IMUPLUS) ? Serial.println(" BNO DO NOT WORKING!") : Serial.println(" BNO SETUP COMPLETE!");
    delay(100);
  }

  void resetCoordinatesValues(int8_t compass = -1) {
    bno.getEvent(&event);
    if (compass < 0) {
      compass = 0;
      offset_x = event.orientation.x;
      offset_y = event.orientation.y;
    }
    for (uint8_t i = 0; i < 4; i++) {
      int16_t value = getYawAngle() + 90 * i;
      value = value <= 360 ? value : value - 360;
      coordinatesValues[(i + compass) % 4] = convertGyro(value, 0);
    }
  }

  int16_t getYawAngle(int16_t offset = NULL) {
    bno.getEvent(&event);
    offset = offset == NULL ? offset_x : offset + offset_x;
    return convertGyro(event.orientation.x, offset);
  }

  int16_t getInclinationAngle(int16_t offset = NULL) {
    bno.getEvent(&event);
    offset = offset == NULL ? offset_y : offset + offset_y;
    return -convertGyro(event.orientation.y, offset);
  }

  int16_t convertGyro(int16_t value, int16_t offset) {
  int16_t d = (int16_t)(value - offset);
  // normaliza para [-180, +180] sem dividir
  while (d > 180)  d -= 360;
  while (d < -180) d += 360;
  return d;
  }


  int16_t getAngleToNearmostCoordinate(bool choose = false) {
    int angle;
    int vector[] = { 0, 0, 0, 0 };
    for (uint8_t i = 0; i < 4; i++) vector[i] = getYawAngle(coordinatesValues[i]);
    uint16_t min_value = min(abs(vector[0]), min(abs(vector[1]), min(abs(vector[2]), abs(vector[3]))));
    uint16_t index;
    for (index = 0; index < 4; index++) {
      if (abs(vector[index]) == min_value) break;
    }
    return choose ? index : -vector[index];
  }

  bool isWorking() {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(sys), Serial.print(", "), Serial.print(gyro), Serial.print(", "), Serial.print(accel), Serial.print(", "), Serial.println(mag);
    if (gyro == 3) return true;
    return false;
  }
};
BNO055 gyro;

// ----------- Encoder and Motor -------------- 
const uint8_t motor_length = 8;
const uint8_t motor_vector[motor_length] = { FORWARD_0, BACK_0, FORWARD_1, BACK_1, FORWARD_2, BACK_2, FORWARD_3, BACK_3 };

float lastencoder = 0;

volatile uint32_t pulseCount = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderISR() {
  pulseCount++;
}

float Encoder() {
  //int incAngle = -gyro.getInclinationAngle();
 // bool cmValue = incAngle > 10 ? true : false;
  uint32_t cnt;
  portENTER_CRITICAL(&mux);
  cnt = pulseCount;
  portEXIT_CRITICAL(&mux);
    return cnt * 0.00630f;  // 0.00590 roda grande
}

// ---------------- General ------------------ //
OpenMVCamera camEsquerda(CAM_LEFT);
OpenMVCamera camDireita(CAM_RIGHT);

Adafruit_NeoPixel led = Adafruit_NeoPixel(N_LEDS, LEDS_PIN, NEO_GRB + NEO_KHZ800);
Point ponto = Point(10, 10);
MazeRunner robot = MazeRunner(20, 20, ponto);
Servo servo;
Point checkpoint = ponto;

sensors_event_t event;

struct VictimData {
  uint8_t esq;
  uint8_t dir;
};
static QueueHandle_t cameraQueue;
TaskHandle_t TaskOnCore0;  // Variável para armazenar o identificador da tarefa task
TaskHandle_t TaskOnCore1;  // Variável para armazenar o identificador da tarefa task

// ---------------- Reflect plate ------------- //

const uint8_t sensor_length = 5;
const uint8_t sensor_vector[sensor_length] = { SENSOR_c9, SENSOR_ldrR, SENSOR_ldrG, SENSOR_ldrB };
int16_t sensor_values[sensor_length];

// ---------------- GY Sensors ---------------- //
const uint8_t N_SENSORS = 8;
VL53L0X sensors[N_SENSORS];

int dist_sensors_offsets[8] = {30, 52,  37,  47,  62, 122,  50,  40};
//int dist_sensors_offsets[8] = {30, 52,  37,  47,  62, 122,  50,  43};
uint16_t sensors_target_value[] = {30, 330, 630, 930};

BluetoothSerial SerialBT;

bool no = 1;
uint8_t tile_count = 0;
int offX = 0;
int originalOffX[4] = { 0, 0, 0, 0 };
int lastError = 0;
int8_t position = 0;
uint16_t globalIndex = robot.pointToIndex(ponto);
bool bnoFlag = 0;
bool closerToWall = false;
uint8_t victimCounter = 0;
bool wasInclined = false;
bool cameraIgnore = true;
float zeroEncoder = 0;
int incAngle = gyro.getInclinationAngle();
unsigned long incTimeBegin = 0;
bool incDetected = false;

void beginLed();
void analog_write(uint8_t, uint8_t);
int16_t motorValueCorrection(int);
void moveTank(int, int, bool);
void stopTank();
void pdControl(int16_t, int16_t, float, float, uint8_t);
int read_sensors(uint8_t);
int read_sensors_pure(uint8_t);
void tcaselect(uint8_t);
void calibrate_dist_sensors();
void init_gy();
void moveTile();
uint8_t path_decision();
void axisCurve(int8_t, float userKp = 9.0, uint8_t correctionLimit = 255);
void printEncoder();
void printCam();
void printSensorsPure();
void printVector(int16_t* vector, uint8_t len);

void led_clear();
void return2init();
void begin_gy();
void alignTile();
void moveTile();
bool walkByEncoder(long int, bool);
const char *getColor();
bool moveSwitch();
void alignGyro();
void blink_led(uint8_t, uint8_t, bool);
void printGyro();
void printSensors();
void printReflectance();
bool camera_Identify();
void testeServo();
void taskOnCore0(void *pvParameters);
void taskOnCore1(void *pvParameters);
void taskCamera(void *pvParameters);

void setup() {
  analogReadResolution(10);
  ledcSetup(FORWARD_0, 5000, 8);
  ledcSetup(BACK_0, 5000, 8);
  ledcSetup(FORWARD_1, 5000, 8);
  ledcSetup(BACK_1, 5000, 8);
  ledcSetup(FORWARD_2, 5000, 8);
  ledcSetup(BACK_2, 5000, 8);
  ledcSetup(FORWARD_3, 5000, 8);
  ledcSetup(BACK_3, 5000, 8);
  ledcAttachPin(LEFT_MOTOR_1_0, FORWARD_0);
  ledcAttachPin(LEFT_MOTOR_2_0, BACK_0);
  ledcAttachPin(LEFT_MOTOR_1_1, FORWARD_1);
  ledcAttachPin(LEFT_MOTOR_2_1, BACK_1);
  ledcAttachPin(RIGHT_MOTOR_1_2, FORWARD_2);
  ledcAttachPin(RIGHT_MOTOR_2_2, BACK_2);
  ledcAttachPin(RIGHT_MOTOR_1_3, FORWARD_3);
  ledcAttachPin(RIGHT_MOTOR_2_3, BACK_3);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(800000); // 400.000
  SerialBT.begin("Fran Robots");
  stopTank();
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  pinMode(SWITCHLEFT, INPUT_PULLUP);
  pinMode(SWITCHRIGHT, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.begin(115200);
  for (uint8_t i = 0; i < sensor_length; i++) pinMode(sensor_vector[i], INPUT);
  led.begin();
  led_clear();
  gyro.begin();
  begin_gy();
  beginLed();
  gyro.resetCoordinatesValues();
  servo.attach(SERVO_PIN);
  servo.write(KIT_CENTER);
  Serial.println("Setup ready");
  Serial.println("Que Deus abençoe o round!");

cameraQueue = xQueueCreate(1, sizeof(VictimData));
if (!cameraQueue) {Serial.println("Erro criando cameraQueue");}

  xTaskCreatePinnedToCore(taskOnCore0, "TaskOnCore0", 12000, NULL, 2, &TaskOnCore0, 0);
  xTaskCreatePinnedToCore(taskOnCore1, "TaskOnCore1", 12000, NULL, 1, &TaskOnCore1, 1);
  xTaskCreatePinnedToCore(taskCamera, "taskCamera", 4096, NULL, 2, NULL, 1);
  disableCore0WDT();
  disableCore1WDT();
  delay(500);
  Serial.println("END SETUP");
}

void taskCamera(void *pvParameters) {
VictimData buf;
  for (;;) {
    CameraData bufesquerda = camEsquerda.ler();
    CameraData bufdireita = camDireita.ler();
    if (bufdireita.vitima != 0 || bufesquerda.vitima != 0) {
        buf.esq = bufesquerda.vitima;
        buf.dir = bufdireita.vitima;
      xQueueOverwrite(cameraQueue, &buf);
    }
    vTaskDelay(pdMS_TO_TICKS(140));  // exemplo: 500 ms entre leituras
  }
}

void taskOnCore0(void *pvParameters) {
  unsigned long timeToReturn = millis();
  unsigned long timeLimit = 5 * 60 * 1000;  // 8 minutos
  for (;;) {
    if (!bnoFlag) {
      position = gyro.getAngleToNearmostCoordinate(1);
      gyro.resetCoordinatesValues();
      //vectorReorder(gyro.coordinatesValues, position);
      bnoFlag = 1;
      no = 1;
      //alignTile();
    }
    if (no) {
      position = gyro.getAngleToNearmostCoordinate(1);
    if (millis() - timeToReturn > timeLimit) { 
      cameraIgnore = false; 
      return2init();
    }else{
        //moveTile();
        //beginLed();
        //getNextTileAngle();
        //walkByEncoder(30, true);
        //delay(10000);
        //alignTile();
        printReflectance();
        //SerialBT.println(getColor());
        //moveTank(100, 100, true);
        //blink_led(5, 3, true);
        //printCam();
        //delay(10000);
        //testeServo();
        //printGyro();
        //Encoder();
        //printEncoder();
        //printSensorsPure();
        //printSensors();
        //printVector();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));  // cede o Core 0 para outras tasks
  } // Final for 
} // Final core 0

void taskOnCore1(void *pvParameters) {
  for (;;) {
    if (digitalRead(BUTTON)) {
      while (eTaskGetState(TaskOnCore0) != eSuspended) {
        vTaskSuspend(TaskOnCore0);
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      no = 0;
      bnoFlag = 0;
      stopTank();
      uint16_t last_index = robot.getVisitNodes().len() - 1;
      Point atual = robot.getActualPoint();
      if (robot.pointToIndex(atual) == robot.getVisitNodes().getByIndex(last_index) && robot.getVisitNodes().getByIndex(last_index) != robot.pointToIndex(ponto)) {
        robot.delVisitNodesByIndex(last_index);
      }
      robot.setActualPoint(checkpoint);
      led_clear();
      while (digitalRead(BUTTON)) {
         vTaskDelay(pdMS_TO_TICKS(100));
      }
      vTaskResume(TaskOnCore0);
      vTaskDelay(pdMS_TO_TICKS(10));  // polling leve
    } 
  } // Final for 
} // Final core 0

void loop() {
  vTaskDelete(NULL);
}
