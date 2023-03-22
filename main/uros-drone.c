#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "imu.h"
#include "copter.h"
#include "bus_config.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define CONFIG_MICRO_ROS_APP_STACK 4000
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 1
#define G_TO_MS2 9.80665

rcl_subscription_t subscriber;
// rcl_subscription_t state_estimator_subscriber;
rcl_publisher_t mpu9250_imu_publisher;

std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 recv_msg;

extern mpu9250_dev_t mpu9250;

sensor_msgs__msg__Imu imu;
// sensor_msgs__msg__Imu recv_imu_data;
static const char *TAG = "app_main";

void mpu9260_imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL){
		esp_err_t ret = imu_mpu9250_acquire_acce();
		ret += imu_mpu9250_acquire_gyro();

		if (ret == ESP_OK){
			gettimeofday(mpu9250.timer, NULL);
			
			imu.header.stamp.sec = mpu9250.timer->tv_sec;
			imu.header.stamp.nanosec = mpu9250.timer->tv_usec/1000;
			imu.linear_acceleration.x = mpu9250.mpu6500->acce_values.acce_x;
			imu.linear_acceleration.y = mpu9250.mpu6500->acce_values.acce_y;
			imu.linear_acceleration.z = mpu9250.mpu6500->acce_values.acce_z;
			imu.angular_velocity.x = mpu9250.mpu6500->gyro_values.gyro_x;
			imu.angular_velocity.y = mpu9250.mpu6500->gyro_values.gyro_y;
			imu.angular_velocity.z = mpu9250.mpu6500->gyro_values.gyro_z;
			RCSOFTCHECK(rcl_publish(&mpu9250_imu_publisher, &imu, NULL));
		}
	}
}

void state_estimator_callback(const void * msgin)
{
	// const sensor_msgs__msg__Imu *imu_msg = (const sensor_msgs__msg__Imu *) msgin;
	ESP_LOGI(TAG, "Called state estimator subscriber callback");
}

void subscriber_callback(const void * msgin)
{
	const std_msgs__msg__Int32 *throttle = (const std_msgs__msg__Int32 *) msgin;
	ESP_LOGI(TAG, "Received throttle data %d", throttle->data);
	if (throttle->data == -9){
		ESP_LOGI(TAG, "Motor disarm command received");
		disarm_motors();
	} else if (throttle->data == -10){
		ESP_LOGI(TAG, "Motor arm command received");
		arm_motors();
	} else {
		float data = (float) throttle->data;
		esp_err_t ret = set_throttle_copter(data);
		if (ret != ESP_OK){
			ESP_LOGE(TAG, "Failed to set throttle value %f", data);
		} else {
			ESP_LOGI(TAG, "Successfully set throttle value %f", data);
		}
	}
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	// RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

	// Auto discover micro-ROS agent
	RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, CONFIG_NODE_NAME, CONFIG_NODE_NAMESPACE, &support));
	
	//publisher with best effort transport
	RCCHECK(rclc_publisher_init_best_effort(
		&mpu9250_imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"sensors/imu"));

	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"throttle"));

	// create timer,
	rcl_timer_t timer3;
	const unsigned int timer3_timeout = 10;

	RCCHECK(rclc_timer_init_default(
		&timer3,
		&support,
		RCL_MS_TO_NS(timer3_timeout),
		mpu9260_imu_timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer3));
	
	RCCHECK(rclc_executor_add_subscription(
		&executor, 
		&subscriber, 
		&recv_msg,
		&subscriber_callback,
		ON_NEW_DATA));


	msg.data = 0;

	imu.header.frame_id.data = "turtle_01";
	imu.header.frame_id.size = 9;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rclc_executor_fini(&executor));
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	// RCCHECK(rcl_subscription_fini(&state_estimator_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&mpu9250_imu_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	esp_err_t ret = i2c_bus_init(ACTUATOR_BUS);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "Failed to initialize Actuator Bus");
		while (true)
		{
			sleep(1000);
		}
		
	}
	ret = i2c_bus_init(SENSOR_BUS);
	if (ret != ESP_OK){
		ESP_LOGE(TAG, "Failed to initialize Sensor Bus");
		while (true)
		{
			sleep(1000);
		}
		
	}
	ret = mpu9250_create();
	if (ret != ESP_OK){
		while (true){
			sleep(1000);
		}
	}

	init_vehicle();

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);

}