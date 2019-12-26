
//acelarómetro
#include <Wire.h>
#include <SPI.h>
#include <SparkFun_MMA8452Q.h>
MMA8452Q accel;

//Sensor temperatura e humidade
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Antena RFID
#include <MFRC522.h>
#define SS_PIN 10
#define RST_PIN 9
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;
byte nuidPICC[4];
String nuidMaster = "75177789";

// -- PIN's --------
#define JOY_X_PIN A0
#define THROTLE_PIN A1
#define TEMP_PIN 4
#define BUZZER_PIN 3
#define LED_PIN 7
// ---- End PIN's ------

//Outros defines
#define DELAY 5
#define SIM_DELAY 5
#define MAX_ROLL 90
#define MIN_ROLL -90
#define MAX_PITCH 45
#define MIN_PITCH -45
#define PYTHON_OUT false

// Variaveis para simulação de voo
double pitch = 0;
double roll = 0.0;
double heading = 0.0; // 0 a 359 degrees
int altitude = 0; //feet
int ground_distance = 0; //Feet
double air_speed = 0.0; //Knots
int vertical_speed = 0; //feet per minute
double engine_load = 0.0;
// --------------

//variaveis do volante
short int yaw_value = 0;
short int pitch_value = 0;
short int roll_value = 0;
double throttle = 0.0;

short int max_yaw_value = -1025;
short int max_pitch_value = -1025;
short int max_roll_value = -1025;

short int min_yaw_value = 1025;
short int min_pitch_value = 1025;
short int min_roll_value = 1025;

short int iddle_yaw_value = 0;
short int iddle_pitch_value = 0;
short int iddle_roll_value = 0;
//--------------------

// Outras Variaveis ---
float temperature = 0.0;
float humidity = 0.0;
bool locked = true;
short int print_tick = 0;
short int ang_tick = 0;
short int temp_tick = 0;
short int accel_tick = 0;
short int joystick_tick = 0;
short int throttle_tick = 0;
short int rfid_tick = 0;
short int sim_tick = 0;
short int calib_tick = 0;
short int tick = 0;
short int dummy_int = 0;
double dummy_double = 0;
String dummy_string = "";

// --------------------

void setup() {

  digitalWrite(LED_PIN, HIGH);
  Serial.begin(9600);
  accel.init();
  dht.begin();
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TEMP_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522 

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  while (!Serial);
  
  tone(BUZZER_PIN, 5000, 100);
  digitalWrite(LED_PIN, LOW);

}

void loop() {

  addTick();

  if (passed(rfid_tick, 500))
    handleRFID();

  if (locked) {

    calibrate();
    
    if (passed(calib_tick, 500))
      printCalibration();
    
  } else {
    
    if (passed(accel_tick, 20))
      readAccel();

    if (passed(throttle_tick, 20))
      readThrottle();

    if (passed(joystick_tick, 20))
      readJoystick();

    if (passed(temp_tick, 5000))
      readTemp();

    if (passed(sim_tick, 50))
      processSimulation();

    if (PYTHON_OUT) {
      
      if (passed(ang_tick, 50))
        printAngles();
        
    } else {
      
      if (passed(print_tick, 500))
        printVariables();
        
    }
      
  }
  
  delay(DELAY);
}

void addTick() {
  temp_tick++;
  accel_tick++;
  joystick_tick++;
  throttle_tick++;
  rfid_tick++;
  print_tick++;
  sim_tick++;
  calib_tick++;
  ang_tick++;
}

bool passed(int tick_var, int ms) {
  if (tick_var * DELAY >= ms)
    return true;
  else
    return false;
}

void printAngles() {

  ang_tick = 0;
  Serial.print("!ANG:");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(heading);
  
}

void printVariables() {
  print_tick = 0;
  Serial.print("[");
  Serial.print(pitch_value);
  Serial.print("\t| ");
  Serial.print(roll_value);
  Serial.print("\t| ");
  Serial.print(yaw_value);
  Serial.print("\t][ ");
  Serial.print(throttle);
  Serial.print("\t] \tCONTROLS");
  Serial.println();

  Serial.print("[");
  Serial.print(pitch);
  Serial.print("\t| ");
  Serial.print(roll);
  Serial.print("\t| ");
  Serial.print(heading);
  Serial.print("\t][ ");
  Serial.print(air_speed);
  Serial.print("\t] [");
  Serial.print(ground_distance);
  Serial.print("\t]");
  Serial.println(" AIRPLANE");
  Serial.println("----------------------------------------------");
  
}

void printCalibration() {

  calib_tick = 0;
  Serial.print("[");
  Serial.print(min_pitch_value);
  Serial.print("\t| ");
  Serial.print(min_roll_value);
  Serial.print("\t| ");
  Serial.print(min_yaw_value);
  Serial.print("\t] MIN");
  Serial.println();

  Serial.print("[");
  Serial.print(iddle_pitch_value);
  Serial.print("\t| ");
  Serial.print(iddle_roll_value);
  Serial.print("\t| ");
  Serial.print(iddle_yaw_value);
  Serial.print("\t] IDDLE");
  Serial.println();

  Serial.print("[");
  Serial.print(max_pitch_value);
  Serial.print("\t| ");
  Serial.print(max_roll_value);
  Serial.print("\t| ");
  Serial.print(max_yaw_value);
  Serial.print("\t] MAX");
  Serial.println();

  Serial.println("----------------------------------");
}

void readTemp() {

  temp_tick = 1;
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
}

void readAccel() {

  accel_tick = 0;
  accel.read();
  pitch_value = accel.x;
  roll_value = accel.y;
 
}

void readThrottle() {

  throttle_tick = 0;
  dummy_double = analogRead(THROTLE_PIN);

  throttle = (dummy_double / 1022.00);

  if (throttle < 0.01)
    throttle = 0.0;
  else if (throttle > 0.99)
    throttle = 1.0;
}

void readJoystick () {
  joystick_tick = 0;
  yaw_value = analogRead(JOY_X_PIN);
}

void handleRFID() {

  rfid_tick = 0;
  
  // Look for new cards
  if ( ! rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if ( ! rfid.PICC_ReadCardSerial())
    return;

  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println("RFID Triggered!");

  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }

  if (air_speed <= 1 && ground_distance <= 0.1) {

    if (rfid.uid.uidByte[0] == nuidPICC[0] || 
    rfid.uid.uidByte[1] == nuidPICC[1] || 
    rfid.uid.uidByte[2] == nuidPICC[2] || 
    rfid.uid.uidByte[3] == nuidPICC[3] )
      return;
    
    dummy_string = "";
    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
      dummy_string.concat(String(nuidPICC[i] < 0x10 ? " 0" : " "));
      dummy_string.concat(String(nuidPICC[i], HEX));
    }

    dummy_string.replace(" ", "");

    if (nuidMaster.equals(dummy_string)) {

      if (locked) {
        locked = false;
        Serial.println("Plane unlocked!");
        tone(BUZZER_PIN, 5000, 300);

        Serial.print("Iddle Pitch: ");
        Serial.println(iddle_pitch_value);
        
      } else {
        locked = true;
        Serial.println("Plane locked!");
        
      }
      
    } else {
      tone(BUZZER_PIN, 200, 300);
      Serial.println("Chave invalida!");
    }
    
  } else {
    Serial.println("Não pode bloquear ou desbloquear o avião enquanto este está a voar.");
  }
  
  // Halt PICC
  rfid.PICC_HaltA();

  // Stop encryption on PCD
  rfid.PCD_StopCrypto1();
  
}

void processSimulation() {
  
  sim_tick = 0;
  int local_pitch = 0;
  int local_roll = 0;
  int local_yaw = 0;

  //Load pitch
  if (pitch_value > max_pitch_value)
    local_pitch = max_pitch_value;
  else if (pitch_value < min_pitch_value)
    local_pitch = min_pitch_value;
  else
    local_pitch = pitch_value;

  local_pitch -= iddle_pitch_value;

  //load Roll
  if (roll_value > max_roll_value)
    local_roll = max_roll_value;
  else if (roll_value < min_roll_value)
    local_roll = min_roll_value;
  else
    local_roll = roll_value;

  local_roll -= iddle_roll_value;


  //Load yaw
  if (yaw_value > max_yaw_value)
    local_yaw = max_yaw_value;
  else if (yaw_value < min_yaw_value)
    local_yaw = min_yaw_value;
  else
    local_yaw = yaw_value;

  local_yaw -= iddle_yaw_value;

  //Process pitch
  if (local_pitch > 10 && air_speed > 120) {
    pitch += ((((local_pitch * 100) / max_pitch_value) * SIM_DELAY) * 0.01) * 0.1;
  } else if (local_pitch < -10 && air_speed > 120) {
    pitch -= ((((local_pitch * 100) / min_pitch_value) * SIM_DELAY) * 0.01) * 0.1;
  } else {

    if (pitch < -0.1)
      pitch += 0.02 * SIM_DELAY;
    else if (pitch > 0.1)
      pitch -= 0.02 * SIM_DELAY;
      
  }

  if (pitch > MAX_PITCH)
    pitch = MAX_PITCH;
  else if (pitch < MIN_PITCH)
    pitch = MIN_PITCH;

  //process ROLL
  if (local_roll > 10 && air_speed > 120) {
    roll += (((local_roll / 10) * SIM_DELAY) * 0.01);
  } else if (local_roll < -10 && air_speed > 120) {
    roll += (((local_roll / 10) * SIM_DELAY) * 0.01);
  } else {
    if (roll < -0.1)
      roll += 0.01 * SIM_DELAY;
    else if (roll > 0.1)
      roll -= 0.01 * SIM_DELAY;
  }

  if (roll > MAX_ROLL)
    roll = MAX_ROLL;
  else if (roll < MIN_ROLL)
    roll = MIN_ROLL;

  //Process HEADING using YAW
  if (ground_distance == 0) {
    
    dummy_double = heading;

    if (air_speed > 2 && air_speed < 15) {
      dummy_double += (((local_yaw / 10) * SIM_DELAY) * air_speed) * 0.01;
    } else if (air_speed >= 15){
      dummy_double += (((local_yaw / 10) * SIM_DELAY) / air_speed) * 0.01;
    }
    
  } else {
    dummy_double += (((local_yaw / 10) * SIM_DELAY) / air_speed) * 0.01;
  }

  //handle speed
  if (ground_distance <= 0.1) {

    if (throttle > 0.10) {
      air_speed += 0.001 + (throttle) - (air_speed * 0.002);
    }
    
    air_speed -= SIM_DELAY * 0.03;
    
  } else if (ground_distance > 0.1) {

    if (throttle > 0.10) {
      air_speed += 0.001 + (throttle) - (air_speed * 0.002) - (pitch * 0.05) - (roll * 0.01);
    }
    
    air_speed -= SIM_DELAY * 0.02;
  }

  if (air_speed < 0)
    air_speed = 0.0;
  else if (air_speed > 330) {
      air_speed--;
  }

  //fix heading
  if (heading > 360) {
    heading -= 360;
  } else if (heading < 0) {
    heading += 360;
  }

  //Handle altitude
  if ((pitch > 0.2 || pitch < -0.2) && air_speed > 140) {
    ground_distance += ((local_pitch / 10) * 0.01) * SIM_DELAY;
  }
  
}

void calibrate() {
  
  readAccel();
  readJoystick();
  
  if (yaw_value < min_yaw_value) min_yaw_value = yaw_value;
  if (pitch_value < min_pitch_value) min_pitch_value = pitch_value;
  if (roll_value < min_roll_value) min_roll_value = roll_value;

  if (yaw_value > max_yaw_value) max_yaw_value = yaw_value;
  if (pitch_value > max_pitch_value) max_pitch_value = pitch_value;
  if (roll_value > max_roll_value) max_roll_value = roll_value;

  iddle_yaw_value = yaw_value;
  iddle_pitch_value = pitch_value;
  iddle_roll_value = roll_value;
  
}

/**
 * Helper routine to dump a byte array as hex values to Serial. 
 */
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
 * Helper routine to dump a byte array as dec values to Serial.
 */
void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], DEC);
  }
}

