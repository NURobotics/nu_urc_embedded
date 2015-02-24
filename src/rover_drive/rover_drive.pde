#include <ros.h>
#include <RoverDriveState.h>

void velocityJointCommandsCallback(const nu_urc_msgs::JointCommands& j_cmds);

using namespace nu_urc_embedded;

RoverDriveState rds;
ros::NodeHandle nh;
ros::Publisher js_publisher("/nu_urc_rover/joints/state", &(rds.js_));
ros::Subscriber<nu_urc_msgs::JointCommands> jc_subscriber("/nu_urc_rover/joints/cmds", &velocityJointCommandsCallback);

#ifdef __cplusplus
extern "C" {
#endif

// void __attribute__((interrupt(IPL7SOFT)))
void __ISR(_TIMER_4_VECTOR, IPL7SOFT) DrivePIDControl(void)
{
  // Initialize all of the PID control terms
  static double v_integral_errors[NUM_DRIVE_JOINTS];
  static double v_prev_errors[NUM_DRIVE_JOINTS];
  static double v_kp[NUM_DRIVE_JOINTS];
  static double v_ki[NUM_DRIVE_JOINTS];
  static double v_kd[NUM_DRIVE_JOINTS];
  static double v_gain[NUM_DRIVE_JOINTS];
  static double dt = 0.01, dt_inv = 100;
  
  double error, u, dedt;
  for(int i = 0; i < NUM_DRIVE_JOINTS; i++) {
    // Compute error terms
    error = rds.joint_control_[i].velocity_goal_ - rds.joint_control_[i].velocity_;
    dedt = (error - v_prev_errors[i])*dt_inv;
    v_integral_errors[i] += error*dt;
  
    // Deal with saturation
    // ...

    // Deal with integrator anti-windup
    // ...
    
    // Compute control signal
    u = rds.joint_control_[i].motor_.getDuty() + v_gain[i]*(v_kp[i]*error + v_ki[i]*v_integral_errors[i] + v_kd[i]*dedt);
    u = constrain(u, -1.0L, 1.0L);
    if(u < 0) {
      rds.joint_control_[i].motor_.setDirection(0);
      u = -u;
    }
    else rds.joint_control_[i].motor_.setDirection(1);
    
    rds.joint_control_[i].motor_.setDuty(u);
  }
  
  IFS0bits.T4IF = 0;                  // Clear Interrupt Flag
}

//void __attribute__((interrupt(IPL7SOFT)))  __attribute__((at_vector(26)))
void __ISR(_CHANGE_NOTICE_VECTOR, IPL7SRS) 
EncoderUpdate(void)
{
  // REPLACE ALL DIGITAL READS WITH PORT READINGS
  int value, dir;
  for(int i = 0; i < NUM_DRIVE_JOINTS; i++) {
    if(rds.joint_control_[i].encoder_.A_buffer_ !=
       (value = digitalRead(rds.joint_control_[i].encoder_.A_pin_))) {
      // Change in pin
      if(value == digitalRead(rds.joint_control_[i].encoder_.B_pin_)) dir = 1;
      else dir = -1;
      rds.joint_control_[i].encoder_.count_ += dir;
    }
  }
  
  IFS1bits.CNIF = 0;                    // Clear Interrupt Flag
}

#ifdef __cplusplus
}
#endif

void setup()
{
  
  // Configure Timer 4 for PID with interrupt
  T4CONbits.ON = 0;                     // Disable during configuration
  T4CONbits.TCKPS = 0b100;
  T4CONbits.TCS = 0;
  T4CONbits.TGATE = 0;
  PR4 = 49999;                          // Timer Frequency of 100Hz
  TMR4 = 0;                             // Reset Timer Counter
  
  // Configure 4 Change Notification for encoders
  CNEN |= (0b1111<<0);
  
  noInterrupts();
  
  // Timer 4 Interrupt
  IPC4bits.T4IP = 7;                    // Interrupt Priority 7
  IPC4bits.T4IS = 2;                    // Subpriority 3
  IFS0bits.T4IF = 0;                    // Clear Flag Status
  IEC0bits.T4IE = 1;                    // Interrupt Enable
  
  // CN Interrupt
  IPC6bits.CNIP = 7;
  IPC6bits.CNIS = 3;
  IFS1bits.CNIF = 0;
  IEC1bits.CNIE = 1;
  
  interrupts();
  
  T4CONbits.ON = 1;                     // Enable
  CNCONbits.ON = 1;
  
  // Configure Timer 2 for PWM
  PWM::setTimerFrequency(3, 1000.0);      // 1000 Hz for better resolution
  PWM::enableTimer(3);
    
  // Pin modes
  // STILL NEEDS SETTING
  
  // Configure the RoverDriveState pins
  rds.joint_control_[FL_WHEEL].motor_.reset(1,1,3);
  rds.joint_control_[FR_WHEEL].motor_.reset(2,1,3);
  rds.joint_control_[BL_WHEEL].motor_.reset(3,2,3);
  rds.joint_control_[BR_WHEEL].motor_.reset(4,2,3);
  
  // Configure ROS publishers and subscribers
  nh.initNode();
  nh.advertise(js_publisher);
  nh.subscribe(jc_subscriber);
}

void loop()
{
  js_publisher.publish(&(rds.js_));
  nh.spinOnce();
  delayMicroseconds(1000000);
}

void velocityJointCommandsCallback(const nu_urc_msgs::JointCommands& j_cmds)
{
  // Set the reference velocity in the RoverDriveState object
  // Get the size of the array
  int len = 0; while(j_cmds.names[len]) len++;
  for(int i = 0; i < len; i++) {
    if(!strcmp(j_cmds.names[i],"front_left_wheel")) rds.joint_control_[FL_WHEEL].velocity_goal_ = j_cmds.commands[i];
    else if(!strcmp(j_cmds.names[i],"front_right_wheel")) rds.joint_control_[FR_WHEEL].velocity_goal_ = j_cmds.commands[i];
    else if(!strcmp(j_cmds.names[i],"back_left_wheel")) rds.joint_control_[BL_WHEEL].velocity_goal_ = j_cmds.commands[i];
    else if(!strcmp(j_cmds.names[i],"back_right_wheel")) rds.joint_control_[BR_WHEEL].velocity_goal_ = j_cmds.commands[i];
  }
}
