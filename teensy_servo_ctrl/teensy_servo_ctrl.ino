#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/time.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/joint_state.h>

#include "STSServoDriver.h"


// Wrap functions in this to check if ROS is still running throughout
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

#pragma GCC diagnostic ignored "-Wunused-result" 


STSServoDriver servos;
uint8_t num_servos = 3;
byte servo_ids[] = {1, 2, 3};
// The arm numbering is as follows
//   Front aka Battery
// ===||     1
//    ||===  3
// ===||     2
int64_t max_pos[] = {6750, 9250, 8500};
int64_t min_pos[] = {700, 3000, 1000};

constexpr unsigned int TIMER_TIMEOUT = RCL_S_TO_NS(1.0 / 250.0); // 1/f_pub

// Define 
//rcl_publisher_t publisher;
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState servo_in_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool micro_ros_init_successful;

// States for reconnecting if needed
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


// Callback function for the timer, publish touch msgs
/*void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    // Get current timestamp
    int64_t now = rmw_uros_epoch_nanos();

    servo_out_msg.header.stamp.sec = RCL_NS_TO_S(now);
    servo_out_msg.header.stamp.nanosec = now - RCL_S_TO_NS(servo_out_msg.header.stamp.sec);

    for(uint8_t i = 0; i < num_servos; i++)
    { 
      servo_out_msg.position.data[i] = (double)(
        (servos.getCurrentPosition(servo_ids[i])) //-min_pos[i]) / (max_pos[i] - min_pos[i])
      );
    }

    // Publish
    RCSOFTCHECK(rcl_publish(&publisher, &servo_out_msg, NULL));
  }
}*/

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void js_subscription_callback(const void *msgin)
{
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;

  for (uint8_t i = 0; i < num_servos; i++)
  {
    // Clip position command to be in [0.0, 1.0]
    double pos = clamp(msg->position.data[i], 0.0, 1.0);
    int target_ticks = min_pos[i] + pos * (max_pos[i] - min_pos[i]);
    // Set target position on servo
    servos.setTargetPosition(
      servo_ids[i], // ID
      target_ticks,  // Position in Ticks
      4000 // Speed in Ticks/s
    );
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  //create init_options
  while(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  // create node
  while(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  // create publishers
  /*while(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/feely_drone/out/servo_states") != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }*/

  // create subscribers
  while(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/feely_drone/in/servo_states") != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  // create timer,
  /*while(rclc_timer_init_default(
    &timer,
    &support,
    TIMER_TIMEOUT,
    timer_callback) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }*/

  // create executor
  while(rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  /*while(rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }*/

  while(rclc_executor_add_subscription(&executor, &subscriber, &servo_in_msg, &js_subscription_callback, ON_NEW_DATA) != RCL_RET_OK)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void) rcl_subscription_fini(&subscriber, &node);
  //(void) rcl_publisher_fini(&publisher, &node);
  //(void) rcl_timer_fini(&timer);
  (void) rclc_executor_fini(&executor);
  (void) rcl_node_fini(&node);
  (void) rclc_support_fini(&support);
}

void setup() {
  
  // Init MicroRos
  set_microros_transports();
  
  // Init user com LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(500);

  // Set State
  state = WAITING_AGENT;
  
  // Init joint message
  servo_in_msg.position.size = num_servos;
  servo_in_msg.position.capacity =  num_servos;
  servo_in_msg.position.data = (double *) malloc(num_servos * sizeof(double));

  servo_in_msg.name.size =  num_servos;
  servo_in_msg.name.capacity =  num_servos;
  servo_in_msg.name.data = (rosidl_runtime_c__String*) malloc(num_servos * sizeof(rosidl_runtime_c__String)); 

  for(uint8_t i = 0; i < num_servos; i++)
  {
    servo_in_msg.name.data[i].capacity = 5;
    servo_in_msg.name.data[i].data = (char *) malloc(8 * sizeof(char));
    servo_in_msg.name.data[i].size = strlen(servo_in_msg.name.data[i].data);
  }

  // Configure Serial5 for half-duplex
  Serial5.begin(1000000);

  // Since the serial port is taken by the servo, we can't easily send debug messages, so
  // we use the on-board led instead.
  if (!servos.init(&Serial5, 1000000))
  {
    // Failed to get a ping reply, turn on the led.
    //digitalWrite(13, HIGH);
  }

  // Reset all servos to position mode: servos have three modes (position, velocity, step position).
  // Position is the default mode so this shouldn't be needed but it's here just to make sure
  // (depending on what you've run before, the servos could be in a different mode)
  servos.setMode(0xFE, STSMode::POSITION); // 0xFE is broadcast address and applies to all servos.
}

void loop() {

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        // Synchronize if not yet synchronized and the agent is available
        if(!rmw_uros_epoch_synchronized() 
          && rmw_uros_ping_agent(100, 10) == RMW_RET_OK)
        {    
          rmw_uros_sync_session(100);
        }
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_BUILTIN, 1);
  } else {
    digitalWrite(LED_BUILTIN, 0);
  }

}