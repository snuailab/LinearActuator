/* 라이브러리 include */
#include <HCMotor.h>

/* 모터 회전수 당 STEP 수 */
#define STEPPERREV 1000

/* 아두이노 PWM 주기 100us = 10000 pulse*/
#define PWMFREQUNCY 10000

/* 모터의 최대속도(mm/s)  */
#define MOTOR_MAX_VELOCITY_MMPERS 600

/* 모터의 최대 가속도(mm/s^2)  */
#define MOTOR_MAX_ACCELERATION_MMPERS2 300


/* 레일의 회전당 이송거리 */
#define MMPERREV 60

/* 레일의 길이 880mm*/
#define RAIL_LENGTH_MM 880

/* 레일의 데드존(충돌위험), 바디길이 60mm */
#define BODY_RANGE 60
#define RAIL_DEADZONE_RANGE 60
#define RAIL_DEADZONE_MIN RAIL_DEADZONE_RANGE
#define RAIL_DEADZONE_MAX RAIL_LENGTH_MM-(RAIL_DEADZONE_RANGE+BODY_RANGE)

/* 모터드라이버 연결핀 */
#define DIR_PIN 8   //스텝모터드라이버 DIR 연결핀
#define CLK_PIN 9   //스텝모터드라이버 CLK 연결핀
#define EN_PIN 7    //스텝모터드라이버 CLK 연결핀

/* 리미트스위치 연결핀 */
#define LLIMIT_PIN 2
#define RLIMIT_PIN 3

/* 조이스틱 아날로그 연결핀 */
#define VRx_PIN A0
#define CSW_PIN A2

/* 테스트 연결핀 */
#define TESTER_PIN 6
#define TESTER2_PIN 5

/* 통신 검증키 */
#define COM_VALIDATION_KEY 0x0F

//SF 변수
enum MOTORSTATE {    // 열거형 정의
    IDLE    = 0,         // 초깃값 할당
    INIT    = 1,
    ESCAPE  = 2,
    SAFETY  = 3,
    PRERUN  = 4,
    RUN     = 5,
    ERROR   = 6
};
volatile int prerun_count = 0;

/* HCMotor 라이브러리 인스턴스 생성 */
HCMotor HCMotor;
volatile int Speed = 1000;

volatile int MotorState = 0;// idle:0, init :1, run:2
volatile int Pre_MotorState = 0;

//안전속도
volatile int safety_speed = 5;
//탈출속도
volatile int escape_speed = 50;
//데드존여부
int is_deadzone = 1;


//스위치상태변수
volatile int SwitchState = 0;//left_stop:1, right_stop:2, all_stop:3, default:0
volatile int Pre_SwitchState = 0;

/* profile control variable */
int profile_state = 0;
int profile_pos_from = 0;
int profile_pos_to = 0;
float profile_pos_distance = 0;
float profile_time_total = 0;
float profile_time_start = 0;
float profile_t_acc = 0;
float profile_t_dec = 0;
float profile_t_max = 0;
float profile_acc_max = 0;
float profile_vel_max = 0;

// 조이스틱의 아날로그 변수
int vrx_voltage = 0;
int pre_vrx_voltage = 0;
int csw_voltage = 0;
int pre_csw_voltage = 0;

/* Timer variable */
unsigned long currentMillis = 0;//현재시간
unsigned long previousMillis = 0;//이전시간
const long interval = 5;  //빈도시간

/* serial communication transmit variable */
volatile int MotorPosition = 0;
volatile int CommandToPosition = 0;
volatile int CommandVelocity = 0;
volatile int CommandAcceleration = 0;
volatile int CommandMove = 0;


byte tx_buffer[64];
byte rx_buffer[64];


// 리미트스위치-> 인터럽트에서 호출함.
void stop_switch(){
  digitalWrite(EN_PIN, HIGH);
  MotorState = MOTORSTATE::ESCAPE;
}

//SF에서 after 변수
unsigned long init_time_ms[10] = {0,};
int after_state_array[10] = {0,};
bool after(int index ,unsigned long interval_ms){
  if(after_state_array[index]==0){
    after_state_array[index]=1;
    init_time_ms[index] = currentMillis;
  }
  else if(after_state_array[index]==1){
    if((init_time_ms[index] + interval_ms) <= currentMillis){
      after_state_array[index] = 0;
      return true;
    }
  }
  return false;
}


/* SI 단위 속도를 모터의 step 속도로 변환 */
int convert_speed(int velocity_mmpersecond){
  if(velocity_mmpersecond == 0){
    return 0;
  }
  return (MMPERREV*(PWMFREQUNCY/STEPPERREV))/velocity_mmpersecond;
}

/* SI 단위 거리를 모터의 step 이동거리로 변환 */
int convert_distance(int lenth_mm){
  return (int)((float)STEPPERREV)*((float)lenth_mm/(float)MMPERREV);
}

// 상대적으로 모터 제어
void motor_position_controller(int position_mm, int velocity_mmpers)
{
  digitalWrite(EN_PIN, LOW);
  HCMotor.DutyCycle(0, convert_speed(abs(velocity_mmpers)));
  HCMotor.Direction(0, velocity_mmpers < 0 ? REVERSE : FORWARD );
  HCMotor.Steps(0,convert_distance(position_mm));
}


// 프로파일 모터제어 설정
bool set_profile_controller(int pos_s, int pos_f, int max_velocity, int max_acceleration)
{
  /*http://engineeronadisk.com/book_modeling/motiona3.html*/
  if((profile_state==0) && ((profile_pos_from != pos_s) || (profile_pos_to != pos_f)) ){
    profile_pos_from = pos_s;
    profile_pos_to   = pos_f;
    
    profile_vel_max = max_velocity;
    profile_acc_max = max_acceleration;

    profile_t_acc = (float)profile_vel_max/(float)profile_acc_max;
    profile_t_dec = profile_t_acc;

    profile_pos_distance = pos_f-pos_s;
    profile_t_max = (abs(profile_pos_distance)/profile_vel_max) - (abs(profile_t_acc)/2.0f) - (abs(profile_t_dec)/2.0f);
    
    profile_time_total = profile_t_acc+profile_t_max+profile_t_dec;

    profile_state = 1;
    return true;
  }
  return false;
}

// 프로파일 모터 이동
void profile_move(unsigned long time_ms,float* pred_x,int* state)
{
  float time_sec = time_ms/1000.0f;
  float now_x = 0.0f;
  float now_velocity = 0.0f;
  float final_time = profile_time_start+profile_time_total;
  float dir = profile_pos_distance < 0.0f ? -1.0f : 1.0f;
  int mode = 0;
  if(profile_state==1){
    digitalWrite(EN_PIN, LOW);
    HCMotor.DutyCycle(0, convert_speed(abs(0)));
    HCMotor.Direction(0, profile_pos_distance < 0 ? REVERSE : FORWARD );
    HCMotor.Steps(0,convert_distance((int)abs(profile_pos_distance)));
    profile_time_start = time_sec;
    profile_state = 2;
  }

  if(profile_state==2)
  {
    // inc
    if(time_sec < profile_time_start+profile_t_acc){
      mode = 1;
      now_velocity = profile_acc_max * (time_sec-profile_time_start);
      HCMotor.DutyCycle(0, convert_speed(abs((int)now_velocity)));
      now_x = 0.5f * now_velocity * (time_sec-profile_time_start);
    }
    // steady
    else if(time_sec < (profile_time_start+profile_t_acc+profile_t_max)){
      mode = 2;
      now_velocity = profile_vel_max;
      
      HCMotor.DutyCycle(0, convert_speed(abs((int)now_velocity)));

      now_x = (0.5f * profile_vel_max * profile_t_acc) 
            + (now_velocity * (time_sec-(profile_time_start+profile_t_acc)));
    }
    // dec
    else if(time_sec < final_time){
      mode = 3;

      float dt = (time_sec-(profile_time_start+profile_t_acc+profile_t_max));
      now_velocity = profile_vel_max - (profile_acc_max * (dt));
      
      HCMotor.DutyCycle(0, convert_speed(abs((int)now_velocity)));

      now_x = (0.5f * profile_vel_max * profile_t_acc) 
            + (profile_vel_max * profile_t_max)
            + ((profile_vel_max - 0.5 * (profile_vel_max-now_velocity))*dt);
    }
    else{ // end
      mode = 4;
      HCMotor.DutyCycle(0, convert_speed(abs(0)));
      now_x = (0.5f * profile_vel_max * profile_t_acc)
              + (profile_vel_max * profile_t_max)
              + (0.5f * profile_vel_max * profile_t_dec);  
      profile_state = 0;
    }
    
  }
  else{
    mode = 5;
    HCMotor.DutyCycle(0, convert_speed(abs(0)));
    now_x = (float)profile_pos_to;
    *pred_x = ((float)profile_pos_from)+(dir*now_x);
    *state = mode;
    return;
  }

  *pred_x = ((float)profile_pos_from)+(dir*now_x);
  *state = mode;
  MotorPosition = (int)(*pred_x);
}


void motor_control(){
  if((MotorState==0)&&(Pre_MotorState!=0)){
    //if()
    //digitalWrite(EN_PIN, HIGH);
  }
}

int check_switch(){
  return ((!digitalRead(LLIMIT_PIN))<<0)|((!digitalRead(RLIMIT_PIN))<<1);
}


void escape_move(){
  if( SwitchState == 1)
  {
    motor_position_controller(RAIL_DEADZONE_RANGE, escape_speed);
    MotorPosition = RAIL_DEADZONE_MIN;
    is_deadzone = 1;
  }
  else if( SwitchState == 2)
  {
    motor_position_controller(RAIL_DEADZONE_RANGE,-escape_speed);
    MotorPosition = RAIL_DEADZONE_MAX;
    is_deadzone = 2;
  }
  else{
    motor_position_controller(0,0);
  }
}

void safety_move(){
  if( is_deadzone == 1)
  {
    motor_position_controller(RAIL_DEADZONE_RANGE, escape_speed);
    MotorPosition = RAIL_DEADZONE_MIN;
  }

  else if( is_deadzone == 2)
  {
    motor_position_controller(RAIL_DEADZONE_RANGE,-escape_speed);
    MotorPosition = RAIL_DEADZONE_MAX;
  }
}


void find_position(){
  if( SwitchState == 0){
    motor_position_controller(safety_speed, -safety_speed);
  }
}

void run_motor_mode(){
  if(CommandMove==0){
    if(vrx_voltage < 250 && pre_vrx_voltage > 250)
    {
      motor_position_controller(60, -60);
        
    }
    else if(vrx_voltage > 750 && pre_vrx_voltage < 750)
    {
      motor_position_controller(60, 60);
    }
  }
  else if(CommandMove==3){

    set_profile_controller(MotorPosition,CommandToPosition,CommandVelocity,CommandAcceleration);

  }
  else if(CommandMove==4)
  {
    float pred_x=0;
    int state = 0;

    profile_move(currentMillis,&pred_x,&state);
    if(state == 5){
      CommandMove = 0;
      MotorPosition = CommandToPosition;
    }

  }
  

  
}


void process(){
  vrx_voltage = (int)analogRead(VRx_PIN);
  csw_voltage = (int)analogRead(CSW_PIN);
  
  SwitchState = check_switch();
  

  switch(MotorState){
    
    case MOTORSTATE::IDLE:{//idle
        if(csw_voltage==0 && pre_csw_voltage!=0){
          MotorState = MOTORSTATE::INIT;
        }else{
          digitalWrite(EN_PIN, HIGH);
        }
        break;
      }
    case MOTORSTATE::INIT:{//init
        find_position();
        break;
      }
    case MOTORSTATE::ESCAPE:{//escape
        escape_move();
        if(after(1,100)){
          MotorState = MOTORSTATE::SAFETY;
          digitalWrite(EN_PIN, LOW);
          safety_move();
        }
        break;
      }
    case MOTORSTATE::SAFETY:{//safety
      if(after(0,(1000*(int)(RAIL_DEADZONE_RANGE/escape_speed))+1000)){
          if(SwitchState == 0){
            MotorState = MOTORSTATE::PRERUN;
            is_deadzone = 0;
            prerun_count = 0;
            //Serial.println("safety");
          }
          else{
            MotorState = MOTORSTATE::IDLE;
          }
      }
      break;
    }
    case MOTORSTATE::PRERUN:{
      float pred_x=0;
      int state = 0;
      if(prerun_count == 0){
        if(after(0,500)){
          set_profile_controller(RAIL_DEADZONE_MIN,RAIL_DEADZONE_MAX,120,240);
          prerun_count = 1;
        }
      }
      else if(prerun_count == 1){
        profile_move(currentMillis,&pred_x,&state);
        if(state == 5){
          prerun_count = 2;
        }
      }
      else if(prerun_count == 2){
        if(after(0,500)){
          set_profile_controller(RAIL_DEADZONE_MAX,RAIL_DEADZONE_MIN,120,240);
          prerun_count = 3;
        }
        
      }
      else if(prerun_count == 3){
        profile_move(currentMillis,&pred_x,&state);
        if(state == 5){
          prerun_count = 4;
          MotorPosition = RAIL_DEADZONE_MIN;
        }


      }
      else if(prerun_count == 4){
        MotorState = MOTORSTATE::RUN;
      }
      break;
      
    }
    case MOTORSTATE::RUN://run
    {
      run_motor_mode();
      break;

    }  
    case MOTORSTATE::ERROR:
      
      
      break;
  };

  //Serial.println(MotorState);
  
  Pre_SwitchState = SwitchState;
  pre_vrx_voltage = vrx_voltage;
  pre_csw_voltage = csw_voltage;
}

void comunication(){
  if(Serial.available() > 0){
    String receive_data = Serial.readString(); 
    receive_data.getBytes(rx_buffer,receive_data.length()+1);
    
    if(rx_buffer[0] == COM_VALIDATION_KEY){
      
      int read_length = rx_buffer[2];

      switch(rx_buffer[1]){
        
        case 'r':{//reset
          if(MotorState != MOTORSTATE::IDLE){
            MotorState = MOTORSTATE::IDLE;
          }
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 0;
          tx_buffer[3] = '\r';
          tx_buffer[4] = '\n';
          Serial.write(tx_buffer,5+1);
          break;
        }

        case 'i':{//init
          if(MotorState!=MOTORSTATE::INIT){
            MotorState = MOTORSTATE::INIT;
          }
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 0;
          tx_buffer[3] = '\r';
          tx_buffer[4] = '\n';
          Serial.write(tx_buffer,5+1);
          break;
        }

        case 'p':{//set profile(trajectory-planning) position
          if(MotorState==(int)MOTORSTATE::RUN){
            int h_data1 = rx_buffer[4];
            int l_data1 = rx_buffer[3];
            int to_x    = (h_data1 << 8 | l_data1);
            to_x  = constrain(to_x,RAIL_DEADZONE_MIN,RAIL_DEADZONE_MAX);
            CommandToPosition = to_x;
            if(CommandMove == 0){
              CommandMove = 1;
            }
          }
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 1;
          tx_buffer[3] = CommandMove==1?1:0;
          tx_buffer[4] = '\r';
          tx_buffer[5] = '\n';
          Serial.write(tx_buffer,6+1);
          break;
        }

        case 'v':{//set profile(trajectory-planning) velocity
          if(MotorState==(int)MOTORSTATE::RUN){
            int h_data1 = rx_buffer[4];
            int l_data1 = rx_buffer[3];
            int vel_x    = (h_data1 << 8 | l_data1);
            vel_x = constrain(vel_x,0,MOTOR_MAX_VELOCITY_MMPERS);
            CommandVelocity = vel_x;

            if(CommandMove == 1){
              CommandMove = 2;
            }
          }
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 1;
          tx_buffer[3] = CommandMove==2?1:0;
          tx_buffer[4] = '\r';
          tx_buffer[5] = '\n';
          Serial.write(tx_buffer,6+1);
          break;
        }

        case 'a':{//set profile(trajectory-planning) velocity
          if(MotorState==(int)MOTORSTATE::RUN){
            
            int h_data1 = rx_buffer[4];
            int l_data1 = rx_buffer[3];
            int acc_x    = (h_data1 << 8 | l_data1);
            acc_x = constrain(acc_x,0,MOTOR_MAX_ACCELERATION_MMPERS2);
            CommandAcceleration = acc_x;
            if(CommandMove == 2){
              CommandMove = 3;
            }
          }
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 1;
          tx_buffer[3] = CommandMove==3?1:0;
          tx_buffer[4] = '\r';
          tx_buffer[5] = '\n';
          Serial.write(tx_buffer,6+1);
          break;
        }



        case 'q':{//get profile(trajectory-planning)
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 8;
          *(int*)(tx_buffer+3) = CommandMove;
          *(int*)(tx_buffer+5) = CommandToPosition;
          *(int*)(tx_buffer+7) = CommandVelocity;
          *(int*)(tx_buffer+9) = CommandAcceleration;
          tx_buffer[11] = '\r';
          tx_buffer[12] = '\n';
          Serial.write(tx_buffer,13+1);
          break;
        }

        case 'm':{//move profile(trajectory-planning)
          if(MotorState==MOTORSTATE::RUN){
            if(CommandMove==3){
              CommandMove = 4;
            }
          }
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 1;
          tx_buffer[3] = CommandMove==4?1:0;
          tx_buffer[4] = '\r';
          tx_buffer[5] = '\n';
          Serial.write(tx_buffer,6+1);
          break;
        }

        case 'g':{//get infomation
          tx_buffer[0] = COM_VALIDATION_KEY;
          tx_buffer[1] = rx_buffer[1];
          tx_buffer[2] = 4;
          *(int*)(tx_buffer+3) = MotorState;
          *(int*)(tx_buffer+5) = MotorPosition;
          tx_buffer[7] = '\r';
          tx_buffer[8] = '\n';
          Serial.write(tx_buffer,9+1);
          break;
        }

      };
      
    }

    
  }
}



void setup() 
{
  /* 스텝모터 라이브러리 초기화 */
  HCMotor.Init();

  /* 스텝모터로 세팅*/
  HCMotor.attach(0, STEPPER, CLK_PIN, DIR_PIN);
  pinMode(EN_PIN, OUTPUT);

  /*테스트용 핀 세팅*/
  pinMode(TESTER_PIN, OUTPUT);
  pinMode(TESTER2_PIN, OUTPUT);

  /*리미트스위치 세팅*/
  pinMode(LLIMIT_PIN, INPUT_PULLUP); 
  pinMode(RLIMIT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LLIMIT_PIN), stop_switch, FALLING);
  attachInterrupt(digitalPinToInterrupt(RLIMIT_PIN), stop_switch, FALLING);

  /*시리얼통신*/
  Serial.begin(115200);
  Serial.setTimeout(int(interval/2));

  /*시리얼통신초기문자열*/
  Serial.println("DKKO-SNUAILAB-STEPMOTOR-DRIVER");

  /*모터초기화*/
  HCMotor.Steps(0,0);
  HCMotor.DutyCycle(0, 0);
  digitalWrite(EN_PIN,LOW);
}

void loop()
{
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      digitalWrite(TESTER_PIN,HIGH);
      process();
      comunication();
      //delay(100);
      digitalWrite(TESTER_PIN,LOW);
  }
}