// Boucle COMMUNE a tous les controleurs (voir controllers/README.md).
//
//   sub /cmd_vel (geometry_msgs/Twist)  -> kin_apply_twist()  [cinematique du robot]
//   timer a CONTROL_PERIOD_MS           -> kin_update()       [capteurs, odometrie, actionneurs]
//                                       -> pub /odom (nav_msgs/Odometry) 1 cycle sur ODOM_PUBLISH_DIV
//   IMU                                 -> pub /imu/data_raw (sensor_msgs/Imu)
//
// Ce fichier ne sait PAS comment le robot roule. Tout ce qui depend de la
// mecanique (diffdrive, Ackermann) est derriere l'interface kin.h, et UNE
// seule implementation est compilee par controleur (main/CMakeLists.txt du
// dossier du controleur).
//
// Deadman : kin_apply_twist(0, 0) si aucun cmd_vel depuis CMD_VEL_TIMEOUT_MS.
// Chaque cinematique sait ce que "zero" veut dire pour elle : moteurs coupes,
// et en Ackermann roues droites en plus.

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
#include <uros_network_interfaces.h>
#endif
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
#include "driver/uart.h"
#include "esp32_serial_transport.h"
#endif

#include "config.h"
#include "imu.h"
#include "kin.h"

static const char *TAG = ROBOT_NODE_NAME;

#define RCCHECK(fn)                                                          \
    do {                                                                     \
        rcl_ret_t rc_ = (fn);                                                \
        if (rc_ != RCL_RET_OK) {                                             \
            ESP_LOGE(TAG, "rcl error %d ligne %d", (int)rc_, __LINE__);      \
            kin_stop();                                                      \
            vTaskDelete(NULL);                                               \
        }                                                                    \
    } while (0)

#define RCSOFTCHECK(fn)                                                      \
    do {                                                                     \
        rcl_ret_t rc_ = (fn);                                                \
        if (rc_ != RCL_RET_OK) {                                             \
            ESP_LOGW(TAG, "rcl warn %d ligne %d", (int)rc_, __LINE__);       \
        }                                                                    \
    } while (0)

static rcl_subscription_t s_sub_cmd_vel;
static rcl_publisher_t s_pub_odom;
static rcl_publisher_t s_pub_imu;
static geometry_msgs__msg__Twist s_cmd_vel_msg;
static nav_msgs__msg__Odometry s_odom_msg;
static sensor_msgs__msg__Imu s_imu_msg;
static bool s_imu_present;

// Date du dernier cmd_vel : ecrite par le callback, lue par le timer de
// controle (meme executor -> pas de concurrence).
static TickType_t s_last_cmd_tick;
static kin_odom_t s_odom;

static void ros_string_set(rosidl_runtime_c__String *str, const char *literal)
{
    str->data = (char *)literal;
    str->size = strlen(literal);
    str->capacity = str->size + 1;
}

static void odom_msg_init(void)
{
    memset(&s_odom_msg, 0, sizeof(s_odom_msg));
    ros_string_set(&s_odom_msg.header.frame_id, "odom");
    ros_string_set(&s_odom_msg.child_frame_id, "base_link");
    s_odom_msg.pose.pose.orientation.w = 1.0;

    // Covariances non nulles : indispensables pour l'EKF côté hôte.
    // x, y et yaw observés ; z, roll, pitch inobservables (valeur énorme).
    static const double pose_cov[6]  = { 1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2 };
    static const double twist_cov[6] = { 1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2 };
    for (int i = 0; i < 6; i++) {
        s_odom_msg.pose.covariance[i * 7] = pose_cov[i];
        s_odom_msg.twist.covariance[i * 7] = twist_cov[i];
    }
}

static void imu_msg_init(void)
{
    memset(&s_imu_msg, 0, sizeof(s_imu_msg));
    ros_string_set(&s_imu_msg.header.frame_id, "imu_link");

    // Pas d'estimation d'orientation embarquée : convention ROS = -1
    s_imu_msg.orientation_covariance[0] = -1.0;

    // Covariances ANNONCEES SELON LE CAPTEUR REELLEMENT DETECTE : c'est avec
    // elles que l'EKF pondere la mesure. Les sous-estimer ferait trop confiance
    // a un gyro bruite (cap qui derive), les surestimer reviendrait a ignorer
    // le gyro et a retomber sur l'odometrie des roues, sensible au patinage.
    // Le L3G4200D du GY-801 est nettement plus bruite que l'ICM-42688 :
    // ~0.03 deg/s/racine(Hz) contre ~0.0028, soit un ecart-type ~10x plus eleve
    // donc une variance ~100x superieure.
    float gyro_var, accel_var;
    if (imu_model() == IMU_MODEL_GY801) {
        gyro_var  = 2.5e-3;
        accel_var = 2.5e-2;    // ADXL345 : ~4 mg/racine(Hz)
    } else {
        gyro_var  = 2.5e-5;    // ICM-42688 en low-noise
        accel_var = 2.5e-3;
    }
    for (int i = 0; i < 3; i++) {
        s_imu_msg.angular_velocity_covariance[i * 4] = gyro_var;
        s_imu_msg.linear_acceleration_covariance[i * 4] = accel_var;
    }
}

static void stamp_now(builtin_interfaces__msg__Time *stamp)
{
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        stamp->sec = (int32_t)(ns / 1000000000LL);
        stamp->nanosec = (uint32_t)(ns % 1000000000LL);
    }
}

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    imu_sample_t sample;
    if (!imu_read(&sample)) {
        return;
    }

    stamp_now(&s_imu_msg.header.stamp);
    s_imu_msg.angular_velocity.x = sample.gx;
    s_imu_msg.angular_velocity.y = sample.gy;
    s_imu_msg.angular_velocity.z = sample.gz;
    s_imu_msg.linear_acceleration.x = sample.ax;
    s_imu_msg.linear_acceleration.y = sample.ay;
    s_imu_msg.linear_acceleration.z = sample.az;

    RCSOFTCHECK(rcl_publish(&s_pub_imu, &s_imu_msg, NULL));
}

static void cmd_vel_callback(const void *msg_in)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    // La cinematique du robot traduit (v, w) en consignes d'actionneurs ; si
    // la demande est impossible pour cette mecanique, elle le journalise.
    kin_apply_twist((float)msg->linear.x, (float)msg->angular.z);
    s_last_cmd_tick = xTaskGetTickCount();
}

static void odom_publish(void)
{
    stamp_now(&s_odom_msg.header.stamp);
    s_odom_msg.pose.pose.position.x = s_odom.x;
    s_odom_msg.pose.pose.position.y = s_odom.y;
    s_odom_msg.pose.pose.orientation.z = sinf(s_odom.theta * 0.5f);
    s_odom_msg.pose.pose.orientation.w = cosf(s_odom.theta * 0.5f);
    s_odom_msg.twist.twist.linear.x = s_odom.v;
    s_odom_msg.twist.twist.angular.z = s_odom.w;

    RCSOFTCHECK(rcl_publish(&s_pub_odom, &s_odom_msg, NULL));
}

static void control_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    float dt = (float)last_call_time / 1e9f;
    if (dt <= 0.0f || dt > 0.5f) {
        dt = CONTROL_PERIOD_MS / 1000.0f;
    }

    // Deadman : agent déconnecté ou teleop arrêté → consigne nulle
    if ((xTaskGetTickCount() - s_last_cmd_tick) > pdMS_TO_TICKS(CMD_VEL_TIMEOUT_MS)) {
        kin_apply_twist(0.0f, 0.0f);
    }

    kin_update(dt, &s_odom);

    // La boucle tourne à 50 Hz (fluide) mais on ne publie /odom qu'1 cycle sur
    // ODOM_PUBLISH_DIV (-> 10 Hz) pour rester sous la limite du série.
    static int publish_div = 0;
    if (++publish_div >= ODOM_PUBLISH_DIV) {
        publish_div = 0;
        odom_publish();
    }
}

static void micro_ros_task(void *arg)
{
    (void)arg;
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#if defined(RMW_UXRCE_TRANSPORT_UDP)
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    // Attente de l'agent : le robot peut booter avant le SBC
    rclc_support_t support;
    while (rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                          &allocator) != RCL_RET_OK) {
        ESP_LOGW(TAG, "micro-ros-agent injoignable, nouvel essai dans 1 s...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "connecté au micro-ros-agent");

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, ROBOT_NODE_NAME, "", &support));

    RCCHECK(rclc_subscription_init_default(
        &s_sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    RCCHECK(rclc_publisher_init_default(
        &s_pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

    rcl_timer_t control_timer;
    RCCHECK(rclc_timer_init_default(&control_timer, &support,
                                    RCL_MS_TO_NS(CONTROL_PERIOD_MS),
                                    control_timer_callback));

    rcl_timer_t imu_timer;
    if (s_imu_present) {
        RCCHECK(rclc_publisher_init_default(
            &s_pub_imu, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
        RCCHECK(rclc_timer_init_default(&imu_timer, &support,
                                        RCL_MS_TO_NS(IMU_PERIOD_MS),
                                        imu_timer_callback));
    }

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &s_sub_cmd_vel, &s_cmd_vel_msg,
                                           &cmd_vel_callback, ON_NEW_DATA));
    if (s_imu_present) {
        RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
    }

    // Horloge synchronisée avec l'agent → stamps /odom et /imu cohérents pour l'EKF
    RCSOFTCHECK(rmw_uros_sync_session(1000));

    odom_msg_init();
    imu_msg_init();
    s_last_cmd_tick = xTaskGetTickCount();

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app_main(void)
{
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    static size_t uart_port = UART_NUM_0;
    ESP_ERROR_CHECK(rmw_uros_set_custom_transport(
        true, (void *)&uart_port,
        esp32_serial_open, esp32_serial_close,
        esp32_serial_write, esp32_serial_read));
#endif
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    // Matériel de la cinematique initialisé, robot a l'arret, avant tout le reste
    kin_init();

    // Calibration gyro au boot, robot immobile (~1 s)
    s_imu_present = imu_init();
    if (s_imu_present) {
        imu_calibrate_gyro();
    }

    // Test au banc, ROUES EN L'AIR, active par BOOT_BENCH_TEST (config.h).
    // Chaque cinematique fournit le sien.
#if BOOT_BENCH_TEST
    kin_bench_test();
#endif

    xTaskCreate(micro_ros_task, "uros_task", 16384, NULL, 5, NULL);
}
