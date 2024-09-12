// new code 20 06 2024
// note: 0: sac ko day, 1: pin, 2: sac co day
const int relay[2] = {2,3}; // relay loa , sac
const float max_value_voltage[2] = {12.6, 12.55}; 
const int analog_pin[2] = {A0, A1}; // a1: sac co day, a2: pin
const long interval = 1000; // 1000 -> 4s

volatile bool reset_flag_sac = false;

const float max_value_analog[2] = {530, 470}; 
const float offset_value_analog[2] = {-45, -330}; 

const float threshold_voltage_khongday = 8.0;
volatile float voltage_sackhongday=0;
volatile bool flag_koday = false;

const float threshold_voltage_coday = 10.0;
volatile float voltage_saccoday=0;
volatile bool flag_coday = false;
const int count_T = 3000;
int count_delay = 0;

volatile long sumVoltage = 0; 
volatile int readingsCount = 0;
unsigned long previousMillis = 0;
bool break_flag = false;
void setup() {
  Serial.begin(38400);
  delay(5);
  pinMode(max_value_voltage[0],INPUT);
  pinMode(max_value_voltage[1],INPUT);
  pinMode(max_value_voltage[2],INPUT);
  pinMode(relay[0],OUTPUT);
  pinMode(relay[1],OUTPUT);
  digitalWrite(relay[0],0);
  digitalWrite(relay[1],0);
  // buzzer(1);
  set_up_timer1();
  Serial.println("SET UP DONE . . .");
}

ISR (TIMER1_OVF_vect) {
  TCNT1 = 65000;
  // PIN
  sumVoltage += map(analogRead(analog_pin[1]),0,max_value_analog[1]/100, 0, max_value_voltage[1]) + offset_value_analog[1]; 
  readingsCount++;
  
//  voltage_sackhongday = (map(analogRead(analog_pin[2]),0,max_value_analog[2]/100, 0, max_value_voltage[2])+ offset_value_analog[2])/100;
//  flag_koday = flag_sackoday(voltage_sackhongday);
//
  voltage_saccoday = (map(analogRead(analog_pin[0]),0,max_value_analog[0]/100, 0, max_value_voltage[0])+ offset_value_analog[0])/100;
  flag_coday = flag_saccoday(voltage_saccoday);
}


void loop() {
  // ------------------------------->start ham xuat gia tri pin
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (readingsCount >= 50) {
      noInterrupts(); 
      float averageVoltage = sumVoltage / readingsCount; 
      averageVoltage /= 100; 
      if(averageVoltage > max_value_voltage[0]) averageVoltage = max_value_voltage[0];
      sumVoltage = 0; readingsCount = 0;
      interrupts();
      Serial.print(String(averageVoltage)); //----------------> print gia tri pin
      Serial.print("       ");
      Serial.println(voltage_saccoday);
      // Serial.print(analogRead(analog_pin[1]));
      // Serial.print("       ");
      // Serial.println(analogRead(analog_pin[0]));
    }
  }
  // ------------------------------->end ham xuat gia tri pin
  

  // new code 27_06
  if (!break_flag)
  {
    if(flag_coday && !break_flag && !reset_flag_sac){
        Serial.println("A");
        buzzer(1);
        digitalWrite(relay[1], 1);
      break_flag = true;
    }
  }
  
  if (reset_flag_sac) {
    //Serial.println("da reset");
    break_flag = false;
    count_delay=0;
    digitalWrite(relay[0], 0);
    digitalWrite(relay[1], 0);
    delay(2000);
    reset_flag_sac = false;
  }
  
  
}

void serialEvent() {
  String recv_data = "";
  if (Serial.available() > 0) {
    recv_data = Serial.readStringUntil('\n');
    if (recv_data == "reset"){
      recv_data = "";
      reset_flag_sac = true;
    }
  }
}
    
bool flag_sackoday(float voltage_sackoday){
  if (voltage_sackhongday > threshold_voltage_khongday) return true;
  else return false;
}
bool flag_saccoday(float voltage_saccoday){
  if (voltage_saccoday > threshold_voltage_coday) return true;
  else return false;
}
void buzzer(int times) {
  for(int counter=0;counter<times;counter++) {

    digitalWrite(relay[0], 1);
    delay(1000);
        digitalWrite(relay[0], 0);
    delay(1000);
  }
}
void set_up_timer1(){
  cli(); 
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCCR1B |= (1 << CS12);   
  TCNT1 = 0;
  TIMSK1 = (1 << TOIE1);            
  sei();
}
