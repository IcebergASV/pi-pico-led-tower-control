#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>
    
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;

const uint RED_LED = 2;
const uint YELLOW_LED = 3;
const uint GREEN_LED = 4;
const uint ESTOP_INPUT = 1;


bool guided_mode = false;

bool current_flowing = true;

rcl_subscription_t led_subscriber;
std_msgs__msg__Bool sub_msg;

// Subscription callback: updates the LED state based on the incoming bool message.
void led_control_callback(const void *msg_in)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;

    guided_mode = msg->data;

    printf("LED set to: %s\n", msg->data ? "ON" : "OFF");
}

int main()
{
    // Set custom transport for micro-ROS.
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize the LED GPIO.
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1); // Turn LED on initially.

    // Initialize the Tower GPIO.
    gpio_init(RED_LED);
    gpio_set_dir(RED_LED, GPIO_OUT);
    gpio_put(RED_LED, 0); // Turn GPIO off initially.

    gpio_init(YELLOW_LED);
    gpio_set_dir(YELLOW_LED, GPIO_OUT);
    gpio_put(YELLOW_LED, 0); // Turn GPIO off initially.

    gpio_init(GREEN_LED);
    gpio_set_dir(GREEN_LED, GPIO_OUT);
    gpio_put(GREEN_LED, 0); // Turn GPIO off initially.

    // Initialize the Input GPIO.
    gpio_init(ESTOP_INPUT);
    gpio_set_dir(ESTOP_INPUT, !GPIO_OUT);


    // Initialize micro-ROS support structures.
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    // Wait for the micro-ROS agent.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        printf("Failed to ping micro-ROS agent. Exiting.\n");
        return ret;
    }

    // Create the node.
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("Error initializing node: %d\n", ret);
        return ret;
    }

    // Initialize the subscription on the "guided_status" topic.
    ret = rclc_subscription_init_default(
        &led_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "guided_status"
    );

    if (ret != RCL_RET_OK) {
        printf("Error initializing subscription: %d\n", ret);
        return ret;
    }


    // Initialize an executor with capacity for 2 handles (timer and subscription).
    rclc_executor_t executor;
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Error initializing executor: %d\n", ret);
        return ret;
    }

    // Add the subscription to the executor.
    ret = rclc_executor_add_subscription(&executor, &led_subscriber, &sub_msg, led_control_callback, ALWAYS);
    if (ret != RCL_RET_OK) {
        printf("Error adding subscription to executor: %d\n", ret);
        return ret;
    }

    // Main loop: spin the executor to process callbacks.
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        current_flowing = gpio_get(ESTOP_INPUT);

        if(!current_flowing) //E-Stop is pressed
        {
            printf("LED RED IS ON");
            gpio_put(YELLOW_LED, 0); // Turn Yellow GPIO off.
            gpio_put(GREEN_LED, 0); // Turn Green GPIO off.
            gpio_put(RED_LED, 1); // Turn Red GPIO on.
        }
        else if(!guided_mode) //RC Mode
        {
            printf("LED YELLOW IS ON");
            gpio_put(RED_LED, 0); // Turn Red GPIO off.
            gpio_put(GREEN_LED, 0); // Turn Green GPIO off.
            gpio_put(YELLOW_LED, 1); // Turn Yellow GPIO on.
        }
        else //If current is flowing and it is not in RC mode, then it is in autonomous mode (guided mode)
        {
            printf("LED GREEN IS ON");
            gpio_put(RED_LED, 0); // Turn Red GPIO off.
            gpio_put(YELLOW_LED, 0); // Turn Yellow GPIO off.
            gpio_put(GREEN_LED, 1); // Turn Green GPIO on.
        }
        
    }

    // Cleanup (unreachable in this example)
    rcl_subscription_fini(&led_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}
