// Open loop motor control example

#include <SimpleFOC.h>
#include "SPI.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"




// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);


HFIBLDCMotor motor = HFIBLDCMotor(1,19.7);
// PIDController P_angle{motor.P_angle.P,motor.P_angle.I,motor.P_angle.D,motor.P_angle.output_ramp,300};	//!< parameter determining the position PID configuration 
// PIDController PID_angle = PIDController(motor.P_angle.P,motor.P_angle.I,motor.P_angle.D,0,250);
// PIDController PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT};//!< parameter determining the velocity PID configuration


// LowPassFilter LPF_angle = LowPassFilter(1/(2000*_2PI));
// BLDCMotor motor = BLDCMotor(1,19.6);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA10, PB1, PA9 ,PB0 ,PA8 , PA7);


LowsideCurrentSense current_sense = LowsideCurrentSense(0.2f, 0.15f, PA4, PA5, PA6);
// MagneticSensorAS5048A sensor(PB6);

MagneticSensorSPI m_sensor = MagneticSensorSPI(MA730_SPI, PA13);
SPIClass SPI_2(PB15, PB14, PB13);



HallSensor h_sensor = HallSensor(PA0, PA1, PA2, 1);
// interrupt routine initialization
void doA(){h_sensor.handleA();}
void doB(){h_sensor.handleB();}
void doC(){h_sensor.handleC();}




// instantiate the commander
float target_voltage = 0;
float lpf_ang;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }
void onMotion(char* cmd){ command.motion(&motor,cmd); }
// void doHFI(char* cmd) { command.scalar(&hfi_status, cmd); }
// void HFIV(char* cmd) { command.scalar(&hfi_v, cmd); }




// void process_hfi(){motor.process_hfi();}

PhaseCurrent_s currents;
DQCurrent_s currentdq;
float ea;
float sa;
uint16_t i = 0;
// voltage set point variable
float target_angle = 0.0f;
float initial_angle;
float current_angle;
float prev_angle;
int motion_flag= 1;
float state_flag= 1;
float delta;
float h_sensor_velocity = 0;
float vq;


void doAngle(char* cmd) { command.scalar(&target_angle, cmd); }
// void lpfD(char* cmd) { command.scalar(&motor.LPF_current_d.Tf, cmd); }
// void lpfQ(char* cmd) { command.scalar(&motor.LPF_current_q.Tf, cmd); }
void pP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }
void pI(char* cmd) { command.scalar(&motor.PID_velocity.I, cmd); }
void pD(char* cmd) { command.scalar(&motor.PID_velocity.D, cmd); }
// void sF(char* cmd) { command.scalar(&state_flag, cmd); }
// void LPF(char* cmd) { command.scalar(&LPF_angle.Tf, cmd); }
// void OR(char* cmd) { command.scalar(&PID_angle.output_ramp, cmd); }

#define NSLEEP PA3
#define DRVOFF PB10
#define NFAULT PC13







float temp_q_setpoint;
void move();
void applyTorque();
// void newPID(float error);
  



void setup() {

  // driver config
  Serial.begin(115200);
   while (!Serial) {
    ; // Wait for Serial to be ready
  }
  Serial.println("USB serial connected");
  Serial.println("******************NEW RUN**************");
  pinMode(NSLEEP, OUTPUT);
  digitalWrite(NSLEEP, HIGH);
  pinMode(DRVOFF, OUTPUT);
  pinMode(NFAULT, INPUT);




  


  // power supply voltage [V]
  motor.useMonitoring(Serial);
  driver.voltage_power_supply = 24;
  motor.voltage_sensor_align = 5; 
  
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 24;
  driver.pwm_frequency = 20000;
  // driver.enable_active_high = false;
  driver.dead_zone = 0.004;


  

  driver.init();
  h_sensor.init();
  m_sensor.init(&SPI_2);

  // enable hall sensor hardware interrupts
  h_sensor.enableInterrupts(doA, doB, doC); 
  // current_sense.linkDriver(&driver);
  // motor.linkCurrentSense(&current_sense);
  motor.linkSensor(&h_sensor);
  // motor.linkSensor(&sensor);

 
  // link the motor and the driver
  motor.linkDriver(&driver);
  digitalWrite(DRVOFF, LOW);
  digitalWrite(NSLEEP, LOW);
  delayMicroseconds(30);
  digitalWrite(NSLEEP, HIGH);



  driver.enable();
  // current_sense.init();




  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  // motor.current_limit= 0.25f;   
  // PID_angle.P = 750.0f;
  // PID_angle.I = 6.0f;
  // PID_angle.D = 0.01f;
  // PID_angle.output_ramp = 0;
  // LPF_angle.Tf =  1/(25*_2PI);

  // motor.PID_velocity.P = 0.2f;
  // motor.PID_velocity.I = 0.10f;
  // motor.PID_velocity.D = 0.0f;
  // motor.PID_velocity.output_ramp = 5;
  // motor.LPF_velocity.Tf = 1/(20*_2PI); 
  

  // motor.LPF_current_d.Tf = 1/(2000*_2PI);
  // motor.LPF_current_q.Tf = 1/(2000*_2PI);
  
 
  // open loop control config
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000;
  

  // motor.Ld = 172.95e-6f;
  // motor.Lq = 232.75e-6f;
  // motor.hfi_v = 6;
  // motor.hfi_on = true;

  
  // motor.sensor_direction = CCW;
  // motor.zero_electric_angle = 3.67;


  // command.verbose = VerboseMode::user_friendly;



  // init motor hardware
  motor.init();
  
  
  motor.initFOC();


  // prev_angle = sensor.getAngle();

  // add target command T
  command.add('T', doTarget, "target voltage");
  command.add('G', doAngle, "target angle");
  // command.add('P', pP, " proportional");
  // command.add('I', pI, " intergral");
  // command.add('D', pD, " differential");
  // command.add('F', sF, " state flag");
  // command.add('L', LPF, " lpf");
  // command.add('O', OR, "Output ramp");







  // command.add('D', lpfD, "tf");
  // command.add('Q', lpfQ, "tf");




 


  // add target command T
  // command.add('T', doTarget, "target velocity");
  // command.add('L', doLimit, "voltage limit");
  // command.add('M',onMotion,"motion control");




  
  Serial.println("HFIMotor2 ready!");
  Serial.println("Set target velocity [rad/s]");
  

  _delay(1000);
  


}


uint32_t time_prev=0;
uint32_t time_p=0;

void loop() {

   
    // Read the incoming data into a buffer
    

    // Optionally process the received message (e.g., as a command)
    // Example: Setting motor target velocity
    // if (strncmp(receivedData, "velocity:", 9) == 0) {
    //     float velocity = atof(receivedData + 9); // Extract velocity value
    //     motor.target = velocity;                // Set motor target
    //     Serial.print("Set target velocity to: ");
    //     Serial.println(velocity);
    // }
    

  


  motor.loopFOC();
  // h_sensor.update();
  m_sensor.update();

  





  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  uint32_t time_now = micros();
  if (fabs(time_now-time_prev) > 1000){
    time_prev = time_now;
    if(target_voltage == 0.0f ){
      motor.PID_velocity.reset();
    }
    motor.move(target_voltage - h_sensor.vel);
    


    

    

    // Serial.println(digitalRead(PC13));
    // Serial.print(motor.enabled);
  }
  uint32_t time_n = micros();
  if (fabs(time_n-time_p) > 1000000){
    time_p = time_n;
    Serial.print("Angle:");
    Serial.println(h_sensor.getSensorAngle());
    Serial.print("Velocity:");
    Serial.println(h_sensor.getVelocity());
    // Serial.print("Shaft:");
    // Serial.println(motor.shaft_velocity);
    Serial.println("-------");

    Serial.print("Target:");
    Serial.println(target_voltage);
    Serial.print("V..Q:");
    Serial.println(motor.voltage.q);
    Serial.print("V..D:");
    Serial.println(motor.voltage.d);
    Serial.print("U.A:");
    Serial.println(motor.Ua);
    Serial.print("HFI:");
    Serial.println(motor.hfi_velocity);
    Serial.print("Temp:");
    Serial.println(motor.current_setpoint.q);
    // Serial.print("Hall:");
    // Serial.println(h_sensor.getVelocity());
    Serial.print("Shaft:");
    Serial.println(motor.shaft_velocity);
    Serial.println("-------");

    


  //   // char receivedData[128]; // Adjust size based on expected input
  //   // size_t dataIndex = 0;

  //   // while (Serial.available() && dataIndex < sizeof(receivedData) - 1) {
  //   //     receivedData[dataIndex++] = Serial.read();
  //   // }
  //   // receivedData[dataIndex] = '\0'; // Null-terminate the string

  //   // // Echo the received data back to the sender
  //   // Serial.print("Received: ");
  //   // Serial.println(receivedData);






  //   // ABCurrent_s s = current_sense.getABCurrents(current_sense.getPhaseCurrents());
  //   // Serial.print(s.alpha);
  //   // Serial.print(s.beta);
  }

  // // Motion control function
  // // velocity, position or voltage (defined in motor.controller)
  // // this function can be run at much lower frequency than loopFOC() function
  // // You can also use motor.move() and set the motor.target in the code
  
  // // user communication
  command.run();
  
  

  // // // has to be called before getAngle and getVelocity
  // h_sensor.update();
  // currentdq = current_sense.getFOCCurrents(_electricalAngle(motor.shaft_angle, motor.pole_pairs));


   
  // current_angle = h_sensor.getAngle();
  // lpf_ang = LPF_angle(current_angle);

  // delta = target_angle - lpf_ang;
  // if(state_flag){
  //   if(!motion_flag){
  //     move();
  //   }
  //   if(fabs(delta) >0.00035f){
  //   target_voltage = -1*(PID_angle(delta));
  //   i++;
  //   if(i>10000){
  //     PID_angle.reset();

  //     i = 0;
  //   }
  //   }else{
  //     target_voltage = 0;
  //   }
  // }else{
  //   if(motion_flag){
  //     applyTorque();
  //   }

  // }

}

  void applyTorque(){
    motor.move(0);
    target_voltage = 0;
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
    motion_flag = 0;
    return;
  }

  void move(){
    motor.move(0);
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::velocity_openloop;
    motion_flag = 1;
    return;
  }


/**
 * TODO:
 * Understand how the controller works 
 * Implement PID
 * Attach motor to Mech Jig and see how the torque works
 */


// void newPID(float error){
//       if(motor.motion_cnt++ < motor.motion_downsample) return;
//       motor.motion_cnt = 0; 
//       pangle = PID_angle(error);

//       // calculate velocity set point
//       temp_q_setpoint = motor.feed_forward_velocity + pangle;
//       temp_q_setpoint = _constrain(temp_q_setpoint,-motor.current_limit, motor.current_limit);
//       temp_q_setpoint = LPF_angle(temp_q_setpoint);
//       // calculate the torque command - sensor precision: this calculation is ok, but based on bad value from previous calculation
//       noInterrupts();
//       motor.current_setpoint.q = temp_q_setpoint;
//       interrupts();
//       // if torque controlled through voltage
//       if(motor.torque_controller == TorqueControlType::voltage){
//         // use voltage if phase-resistance not provided
//         if(!_isset(motor.phase_resistance))  motor.voltage.q = motor.current_setpoint.q;
//         else  motor.voltage.q =  _constrain( motor.current_setpoint.q*motor.phase_resistance + motor.voltage_bemf , -motor.voltage_limit, motor.voltage_limit);
//         // set d-component (lag compensation if known inductance)
//         if(!_isset(motor.phase_inductance)) motor.voltage.d = 0;
//         else motor.voltage.d = _constrain( -motor.current_setpoint.q*motor.shaft_velocity*motor.pole_pairs*motor.phase_inductance, -motor.voltage_limit, motor.voltage_limit);
//       }
//       return;
// }