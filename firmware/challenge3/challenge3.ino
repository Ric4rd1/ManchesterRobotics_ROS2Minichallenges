// Include micro ros and rcl libraries
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>  // include Float32 msg

// Instantiate Subscriber and msg
rcl_subscription_t pwm_subscriber;
std_msgs__msg__Float32 pwm_signal;      // Holds the motor pwm value with sign

// Instantiate micro ros executor and support classes
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Define pins
#define LED_PIN 2 // error indicating LED
#define PWM_PIN_IN1 18     // Driver In 1
#define PWM_PIN_IN2 19     // Driver In 2
// Define both PWM configurations
#define PWM_FRQ 5000   // PWM frequency (Hz)
#define PWM_RES 8      // PWM resolution (bits)
#define PWM_CHNL0 0     // PWM channel for in 1
#define PWM_CHNL1 1     // PWM channel for in 2
// Value limit for msg
#define MSG_MIN_VAL 0  // Minimum regulating signal value
#define MSG_MAX_VAL 1  // Maximum regulating signal value
// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Local variables
float value = 0.0;
int sign = 0;
float magnitude = 0.0;
float pwm_set_point = 0.0;

// Error callback 
void error_loop(){
  while(1){
    // Blink LED every 100 ms
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Subscription callback
void subscription_callback(const void * msgin)
{  
  // Type casting to Float32 and pointer assignment to pwm_signal
  const std_msgs__msg__Float32 * pwm_signal = (const std_msgs__msg__Float32 *)msgin;

  value = pwm_signal->data; // getting data value from metadata
  sign = (value < 0) ? -1 : 1; // retrieving sign
  magnitude = fabsf(value); // retrieving magnitude
  pwm_set_point = constrain(magnitude, MSG_MIN_VAL, MSG_MAX_VAL); // constraining the magnitude to [0,1]

  // Sending regulating pwm signal to pwm channels depending on the sign
  if (sign == 1){
    ledcWrite(PWM_CHNL0, (uint32_t)(pow(2, PWM_RES) * (pwm_set_point / 1.0))); // Map the pwm to 8 bits and send it to channel 0
    ledcWrite(PWM_CHNL1, 0); // set to 0 
  } else{
    ledcWrite(PWM_CHNL1, (uint32_t)(pow(2, PWM_RES) * (pwm_set_point / 1.0))); // Map the pwm to 8 bits and send it to channel 1
    ledcWrite(PWM_CHNL0, 0); // set to 0 
  }
}

void setup() {
  set_microros_transports(); // micro ros configuration
  
  // Configure PWM
  // Set pins as output
  pinMode(PWM_PIN_IN1, OUTPUT); 
  pinMode(PWM_PIN_IN2, OUTPUT);
  // Configure pwm channel
  ledcSetup(PWM_CHNL0, PWM_FRQ, PWM_RES); 
  ledcSetup(PWM_CHNL1, PWM_FRQ, PWM_RES);
  // Assign pins
  ledcAttachPin(PWM_PIN_IN1, PWM_CHNL0);
  ledcAttachPin(PWM_PIN_IN2, PWM_CHNL1);
  
  delay(2000); // Wait for ros configuration to finish

  allocator = rcl_get_default_allocator(); // assign memory allocator

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &pwm_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_signal, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Executor spin
}
