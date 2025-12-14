/*
   ESP32 micro-ROS: Versão FINAL - MAPPING ROBOT
   - Features: Slip Ring + Sensor Hall + Multitasking
   - Calibração: Razão 4.7405 (Pontos visíveis em 360)
   - Offset: 45 graus Anti-Horário aplicado
*/

#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "Bonezegei_ULN2003_Stepper.h"
#include <MPU9250_WE.h> 

// --- BIBLIOTECAS PARA EVITAR REINICIALIZAÇÃO (BROWNOUT) ---
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =================== CONFIGURAÇÕES ===================
#define DIAMETRO 6.8
#define N_FUROS 20
#define MPU_ADDR 0x68
float ODOM_CORRECTION = 0.47; 
float razao = 4.7405; // Ajuste fino para os pontos fecharem 360 graus

// =================== PINOS ===================
const int motorA1 = 18; const int motorA2 = 19;
const int motorB1 = 23; const int motorB2 = 27;

// SENSOR HALL (Ligue no D4)
const int PIN_HALL_SENSOR = 4; 
// LED AZUL DA PLACA (Para Debug Visual)
const int PIN_LED_DEBUG = 2;

const int PWM_FREQ = 1000; const int PWM_RES_BITS = 8;
const int CH_A1 = 0; const int CH_A2 = 1;
const int CH_B1 = 2; const int CH_B2 = 3;

// =================== VARIÁVEIS COMPARTILHADAS ===================
// Posição (Protegidas por Mutex)
volatile float pub_x = 0;
volatile float pub_y = 0;
volatile float pub_theta = 0;
volatile float pub_vx = 0;
volatile float pub_wz = 0;

SemaphoreHandle_t dataMutex; 

// Scan e Sincronia
volatile bool scan_ready = false;
float ranges_array[24]; 
volatile int step_index = 0;

// Flag do Sensor Hall
volatile bool hall_triggered = false;

// Encoders
volatile long pulsos_direita = 0;
volatile long pulsos_esquerda = 0;
void IRAM_ATTR isr_direita() { pulsos_direita++; }
void IRAM_ATTR isr_esquerda(){ pulsos_esquerda++; }

// Interrupção do Hall (Dispara quando o ímã passa)
void IRAM_ATTR isr_hall() { hall_triggered = true; }

// Controle Motor
volatile int targetPwmL = 0;
volatile int targetPwmR = 0;

// Objetos
MPU9250_WE mpu(&Wire, MPU_ADDR);
Bonezegei_ULN2003_Stepper Stepper(32, 26, 25, 33);
VL53L0X tof;

// ROS
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t alloc;
rclc_executor_t executor;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t pub_tf;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_buffer[1];
rcl_publisher_t pub_scan;
sensor_msgs__msg__LaserScan scan_msg;

// Lidar
#define ALFA 15
#define HORARIO 1 
const float passos_rev = 4096;
const float grau_por_passo_motor = 360.0f * razao / passos_rev;
int n_passos = (int)(ALFA / grau_por_passo_motor);

// WiFi
char ssid[] = "Dub";                              
char pass[] = "123456789";
char agent_ip[] = "10.202.237.102";
uint32_t agent_port = 8888;

// =================== FUNÇÕES AUXILIARES ===================
geometry_msgs__msg__Quaternion yaw_to_quat(float yaw){
  geometry_msgs__msg__Quaternion q;
  q.x = 0; q.y = 0; q.z = sin(yaw/2); q.w = cos(yaw/2);
  return q;
}

void applyMotorPWM(int pL, int pR){
  pL = constrain(pL, -255, 255);
  pR = constrain(pR, -255, 255);
  if(pL >= 0){ ledcWrite(CH_A1, 0); ledcWrite(CH_A2, pL); }
  else       { ledcWrite(CH_A1, -pL); ledcWrite(CH_A2, 0); }
  if(pR >= 0){ ledcWrite(CH_B1, pR); ledcWrite(CH_B2, 0); }
  else       { ledcWrite(CH_B1, 0); ledcWrite(CH_B2, -pR); }
}

void cmd_vel_callback(const void * msg){
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist*)msg;
  unsigned long now = millis();
  extern unsigned long lastCmdMillis; 
  lastCmdMillis = now;

  float v = m->linear.x;
  float w = m->angular.z * 4.0;
  float left  = v - w * 0.11 / 2;
  float right = v + w * 0.11 / 2;
  int pwmL = left * 400.0;
  int pwmR = right * 400.0;
   
  if(abs(pwmL)<120 && pwmL!=0) pwmL = pwmL>0?120:-120;
  if(abs(pwmR)<120 && pwmR!=0) pwmR = pwmR>0?120:-120;
   
  targetPwmL = pwmL;
  targetPwmR = pwmR;
}

unsigned long lastCmdMillis = 0;
const unsigned long DEADMAN_MS = 300;

// ==========================================================
// TAREFA 1: ODOMETRIA (Prioridade Alta - 100Hz Fixos)
// ==========================================================
void TaskOdometria(void *pvParameters) {
  const float wheel_circ_m = (DIAMETRO/100.0)*PI;
  long lastL = 0, lastR = 0;
  unsigned long lastTime = micros();
  float local_yaw = 0, local_x = 0, local_y = 0;

  // Garante loop exato de 10ms (100Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); 

  for (;;) {
    applyMotorPWM(targetPwmL, targetPwmR);

    // --- CÁLCULO ODOMETRIA ---
    unsigned long now_micros = micros();
    float dt = (now_micros - lastTime) / 1000000.0; 
    lastTime = now_micros;

    if(dt > 0.0) {
        float gyroZ = mpu.getGyrValues().z;
        if(abs(gyroZ) < 0.1) gyroZ = 0; 
        local_yaw += gyroZ * dt;
        if(local_yaw > 180) local_yaw -= 360;
        if(local_yaw < -180) local_yaw += 360;

        long currL = pulsos_direita;
        long currR = pulsos_esquerda;
        float dL = ((currL - lastL) * wheel_circ_m / N_FUROS) * ODOM_CORRECTION;
        float dR = ((currR - lastR) * wheel_circ_m / N_FUROS) * ODOM_CORRECTION;
        lastL = currL; lastR = currR;

        if(targetPwmL < 0) dL = -dL;
        if(targetPwmR < 0) dR = -dR;

        float dc = (dL + dR) / 2.0;
        float theta_rad = local_yaw * PI / 180.0;
        local_x += dc * cos(theta_rad);
        local_y += dc * sin(theta_rad);

        float vx = dc / dt;
        float wz = gyroZ * (PI / 180.0);

        // Salva nas variáveis globais
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        pub_x = local_x;
        pub_y = local_y;
        pub_theta = theta_rad;
        pub_vx = vx; 
        pub_wz = wz; 
        xSemaphoreGive(dataMutex);
    }
    
    // Libera o processador até o próximo ciclo de 10ms
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==========================================================
// TAREFA 2: LIDAR + HALL + OFFSET 45 GRAUS (ANTI-HORARIO)
// ==========================================================
void TaskLidar(void *pvParameters) {
  for (;;) {
    
    // 1. SINCRONIA: Sensor Hall Disparou
    if (hall_triggered) {
        // Debug LED
        digitalWrite(PIN_LED_DEBUG, HIGH);
        
        // Força publicação imediata
        scan_ready = true; 
        
        // --- AJUSTE DE ROTAÇÃO (OFFSET) ---
        // 1 passo = 15 graus.
        // Queremos 45 graus Anti-Horário -> Voltamos 3 passos (Reset = 21).
        // Se precisar mudar: 0=Frente, 6=Dir, 12=Tras, 18=Esq, 21=Esq+Frente
        
        step_index = 21;  
        
        hall_triggered = false;
        
        vTaskDelay(pdMS_TO_TICKS(5));
        digitalWrite(PIN_LED_DEBUG, LOW);
    }

    // 2. Move Motor (Horário)
    Stepper.step(HORARIO, n_passos);
    vTaskDelay(pdMS_TO_TICKS(15)); 

    // 3. Lê Distância
    float dist_cm = tof.readRangeSingleMillimeters() / 10.0;
    
    // 4. CORREÇÃO DE SIMETRIA (INVERSÃO PARA ROS)
    // Motor HW = Horário | ROS = Anti-Horário
    int ros_index = (24 - 1) - step_index;

    // Tratamento Circular:
    // Como começamos em 21, o índice pode ficar negativo na fórmula acima
    while (ros_index < 0) ros_index += 24;
    while (ros_index > 23) ros_index -= 24;

    if(dist_cm > 4 && dist_cm < 400) {
         ranges_array[ros_index] = dist_cm / 100.0;
    } else {
         ranges_array[ros_index] = scan_msg.range_max;
    }

    step_index++;
    
    // Backup de segurança (caso o Hall falhe ou demore)
    // Limite alto (48) para deixar o Hall controlar, mas resetar se travar
    if(step_index >= 48){ 
        step_index = 0; 
        scan_ready = true;
    }

    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// =================== SETUP ===================
void setup(){
  // --- DESATIVA DETECTOR DE BROWNOUT ---
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);

  // Configuração LED Debug
  pinMode(PIN_LED_DEBUG, OUTPUT);
  digitalWrite(PIN_LED_DEBUG, LOW);

  pinMode(motorA1,OUTPUT); pinMode(motorA2,OUTPUT);
  pinMode(motorB1,OUTPUT); pinMode(motorB2,OUTPUT);
  ledcSetup(CH_A1,PWM_FREQ,PWM_RES_BITS); ledcSetup(CH_A2,PWM_FREQ,PWM_RES_BITS);
  ledcSetup(CH_B1,PWM_FREQ,PWM_RES_BITS); ledcSetup(CH_B2,PWM_FREQ,PWM_RES_BITS);
  ledcAttachPin(motorA1,CH_A1); ledcAttachPin(motorA2,CH_A2);
  ledcAttachPin(motorB1,CH_B1); ledcAttachPin(motorB2,CH_B2);

  pinMode(34,INPUT); pinMode(35,INPUT);
  attachInterrupt(34,isr_direita,RISING);
  attachInterrupt(35,isr_esquerda,RISING);

  // --- CONFIGURAÇÃO DO SENSOR HALL ---
  pinMode(PIN_HALL_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_SENSOR), isr_hall, FALLING);

  Wire.begin(21,22);
  mpu.init(); mpu.autoOffsets(); mpu.enableGyrDLPF();
  mpu.setGyrRange(MPU9250_GYRO_RANGE_2000); mpu.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu.setSampleRateDivider(5);

  tof.init(); tof.setMeasurementTimingBudget(20000);
  Stepper.begin(); Stepper.setSpeed(5); 

  set_microros_wifi_transports(ssid,pass,agent_ip,agent_port);
  delay(1000);

  alloc = rcl_get_default_allocator();
  rclc_support_init(&support,0,NULL,&alloc);
  rclc_node_init_default(&node,"esp32_multitask_node","",&support);

  rclc_subscription_init_default(&sub_cmd_vel,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"/cmd_vel");
  rclc_publisher_init_default(&pub_odom,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"/odom");
  rclc_publisher_init_default(&pub_tf,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs,msg,TFMessage),"/tf");
   
  sensor_msgs__msg__LaserScan__init(&scan_msg);
  scan_msg.ranges.capacity = 24;
  scan_msg.ranges.size = 24;
  scan_msg.ranges.data = ranges_array;
  // Configuração Fixa do Scan
  scan_msg.angle_min = 0;
  scan_msg.angle_max = 2 * PI;
  scan_msg.angle_increment = ALFA * PI/180.0;
  scan_msg.range_min = 0.05;
  scan_msg.range_max = 4.0;
  
  rclc_publisher_init_default(&pub_scan,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,LaserScan),"/scan");

  rclc_executor_init(&executor,&support.context,1,&alloc);
  rclc_executor_add_subscription(&executor,&sub_cmd_vel,&cmd_vel_msg,&cmd_vel_callback,ON_NEW_DATA);

  tf_msg.transforms.capacity = 1;
  tf_msg.transforms.size = 1;
  tf_msg.transforms.data = tf_buffer;

  dataMutex = xSemaphoreCreateMutex();
   
  // === CRIAÇÃO DAS TASKS SEPARADAS ===
  // Core 0, Prioridade 2 (Alta) -> Odometria
  xTaskCreatePinnedToCore(TaskOdometria, "Odom", 4096, NULL, 2, NULL, 0);
  
  // Core 0, Prioridade 1 (Baixa) -> Lidar e Hall
  xTaskCreatePinnedToCore(TaskLidar, "Lidar", 4096, NULL, 1, NULL, 0);

  Serial.println("Sistema Final: Hall + Offset 45deg + Multitask");
}

// =================== LOOP (Core 1) ===================
void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  unsigned long now = millis();
   
  if(now - lastCmdMillis > DEADMAN_MS) {
    targetPwmL = 0; targetPwmR = 0;
  }

  static unsigned long lastPub = 0;
  if (now - lastPub >= 50) {
    float x, y, th, vx, wz;
    
    // Pega dados seguros da TaskOdometria
    if(xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        x = pub_x; y = pub_y; th = pub_theta;
        vx = pub_vx; wz = pub_wz;
        xSemaphoreGive(dataMutex);

        odom_msg.header.stamp.sec = now/1000;
        odom_msg.header.stamp.nanosec = (now%1000)*1000000;
        odom_msg.header.frame_id.data = (char*)"odom";
        odom_msg.child_frame_id.data  = (char*)"base_link";
        
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation = yaw_to_quat(th);
        
        odom_msg.pose.covariance[0] = 0.01; odom_msg.pose.covariance[7] = 0.01;
        odom_msg.pose.covariance[14] = 10000.0; odom_msg.pose.covariance[35] = 0.01;
        
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.angular.z = wz;
        odom_msg.twist.covariance[0] = 0.01; odom_msg.twist.covariance[35] = 0.01;
        
        rcl_publish(&pub_odom, &odom_msg, NULL);

        // TF
        geometry_msgs__msg__TransformStamped *t = &tf_buffer[0];
        t->header = odom_msg.header;
        t->header.frame_id.data = (char*)"odom";
        t->child_frame_id.data  = (char*)"base_link";
        t->transform.translation.x = x;
        t->transform.translation.y = y;
        t->transform.rotation = odom_msg.pose.pose.orientation;
        rcl_publish(&pub_tf, &tf_msg, NULL);
    }
    lastPub = now;
  }

  if(scan_ready){
    scan_ready = false;
    scan_msg.header.stamp.sec = now/1000;
    scan_msg.header.stamp.nanosec = (now%1000)*1000000;
    scan_msg.header.frame_id.data = (char*)"lidar_link";
    rcl_publish(&pub_scan,&scan_msg,NULL);
  }
}
