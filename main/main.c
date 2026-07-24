// mowbot — contrôleur diffdrive micro-ROS pour ESP32-P4 + MDD10A rev2.0
//
//   sub /cmd_vel (geometry_msgs/Twist)  → cinématique inverse → PID vitesse/roue → PWM+DIR
//   encodeurs (PCNT matériel)           → odométrie → pub /odom (nav_msgs/Odometry) à 50 Hz
//
// Deadman : moteurs coupés si aucun cmd_vel depuis CMD_VEL_TIMEOUT_MS.

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
#include "encoders.h"
#include "imu.h"
#include "motors.h"
#include "odometry.h"
#include "pid.h"

static const char *TAG = "mowbot";

#define RCCHECK(fn)                                                          \
    do {                                                                     \
        rcl_ret_t rc_ = (fn);                                                \
        if (rc_ != RCL_RET_OK) {                                             \
            ESP_LOGE(TAG, "rcl error %d ligne %d", (int)rc_, __LINE__);      \
            motors_stop();                                                   \
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

// Consignes vitesse roue [m/s] : écrites par le callback cmd_vel,
// lues par le timer de contrôle (même executor → pas de concurrence).
static float s_target_left;
static float s_target_right;
static TickType_t s_last_cmd_tick;

static pid_ctrl_t s_pid_left;
static pid_ctrl_t s_pid_right;
static odom_state_t s_odom;
static int64_t s_prev_ticks_left;
static int64_t s_prev_ticks_right;

static const float METERS_PER_TICK =
    2.0f * (float)M_PI * WHEEL_RADIUS_M / TICKS_PER_WHEEL_REV;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

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
    for (int i = 0; i < 3; i++) {
        s_imu_msg.angular_velocity_covariance[i * 4] = 2.5e-5;    // gyro ICM-42688 LN
        s_imu_msg.linear_acceleration_covariance[i * 4] = 2.5e-3;
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

    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        s_imu_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
        s_imu_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    }
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
    float v = (float)msg->linear.x;
    float w = (float)msg->angular.z;

    // Cinématique inverse diffdrive
    s_target_left = clampf(v - w * TRACK_WIDTH_M * 0.5f,
                           -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    s_target_right = clampf(v + w * TRACK_WIDTH_M * 0.5f,
                            -MAX_WHEEL_SPEED_MPS, MAX_WHEEL_SPEED_MPS);
    s_last_cmd_tick = xTaskGetTickCount();
}

static void odom_publish(void)
{
    if (rmw_uros_epoch_synchronized()) {
        int64_t ns = rmw_uros_epoch_nanos();
        s_odom_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
        s_odom_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    }

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

    // Deadman : agent déconnecté ou teleop arrêté → stop
    if ((xTaskGetTickCount() - s_last_cmd_tick) > pdMS_TO_TICKS(CMD_VEL_TIMEOUT_MS)) {
        s_target_left = 0.0f;
        s_target_right = 0.0f;
    }

    int64_t ticks_left = encoder_get_ticks(ENCODER_LEFT);
    int64_t ticks_right = encoder_get_ticks(ENCODER_RIGHT);
    float d_left = (float)(ticks_left - s_prev_ticks_left) * METERS_PER_TICK;
    float d_right = (float)(ticks_right - s_prev_ticks_right) * METERS_PER_TICK;
    s_prev_ticks_left = ticks_left;
    s_prev_ticks_right = ticks_right;

    odometry_update(&s_odom, d_left, d_right, dt);

    float v_left = d_left / dt;
    float v_right = d_right / dt;

    // PID vitesse par roue (encodeurs calibres et fiables depuis le passage
    // au banc). Consigne nulle : moteur coupe + reset PID (pas de freinage
    // actif -> pas d'oscillation a l'arret, l'integrale ne pousse plus).
    if (s_target_left == 0.0f) {
        pid_reset(&s_pid_left);
        motors_set(MOTOR_LEFT, 0.0f);
    } else {
        motors_set(MOTOR_LEFT, pid_update(&s_pid_left, s_target_left, v_left, dt));
    }
    if (s_target_right == 0.0f) {
        pid_reset(&s_pid_right);
        motors_set(MOTOR_RIGHT, 0.0f);
    } else {
        motors_set(MOTOR_RIGHT, pid_update(&s_pid_right, s_target_right, v_right, dt));
    }

    // Le PID tourne à 50 Hz (fluide) mais on ne publie /odom qu'1 cycle sur
    // ODOM_PUBLISH_DIV (-> 10 Hz) pour rester sous la limite du série 115200.
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
    RCCHECK(rclc_node_init_default(&node, "mowbot_base", "", &support));

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

// --- TEST BANC AU BOOT (active par BOOT_BENCH_TEST dans config.h) ------------
// Fait tourner UNE roue a la fois (2 s avant, 2 s arriere) et logge le delta
// de ticks des DEUX encodeurs a chaque phase.
#if BOOT_BENCH_TEST
static void boot_test_phase(const char *nom, motor_id_t motor, float duty)
{
    const TickType_t DUREE = pdMS_TO_TICKS(2000);
    int64_t g0 = encoder_get_ticks(ENCODER_LEFT);
    int64_t d0 = encoder_get_ticks(ENCODER_RIGHT);
    motors_set(motor, duty);
    vTaskDelay(DUREE);
    motors_stop();
    int64_t dg = encoder_get_ticks(ENCODER_LEFT) - g0;
    int64_t dd = encoder_get_ticks(ENCODER_RIGHT) - d0;
    ESP_LOGW(TAG, "%-16s : enc_G=%+6lld  enc_D=%+6lld",
             nom, (long long)dg, (long long)dd);
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void boot_test_run(void)
{
    const float DUTY = 0.25f;
    ESP_LOGW(TAG, "==== TEST BANC PAR ROUE (2 s/phase, roues en l'air !) ====");
    ESP_LOGW(TAG, "attendu : la roue testee compte FORT (+ en avant, - en arriere),");
    ESP_LOGW(TAG, "          l'autre encodeur reste ~0. Sinon : 0=muet, signe=INVERT,");
    ESP_LOGW(TAG, "          mauvais compteur=cables croises G/D.");
    boot_test_phase("G avant  (+)", MOTOR_LEFT,  +DUTY);
    boot_test_phase("G arriere(-)", MOTOR_LEFT,  -DUTY);
    boot_test_phase("D avant  (+)", MOTOR_RIGHT, +DUTY);
    boot_test_phase("D arriere(-)", MOTOR_RIGHT, -DUTY);

    // Phase "a la main" : 30 s d'affichage des compteurs bruts, 1 fois/s.
    // Tournez les roues a la main et regardez les valeurs bouger.
    ESP_LOGW(TAG, "---- TEST A LA MAIN : tournez les roues (30 s) ----");
    for (int i = 0; i < 30; i++) {
        ESP_LOGW(TAG, "t=%2ds  enc_G=%+8lld  enc_D=%+8lld", i,
                 (long long)encoder_get_ticks(ENCODER_LEFT),
                 (long long)encoder_get_ticks(ENCODER_RIGHT));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGW(TAG, "==== FIN TEST BANC ====");
}
#endif  // BOOT_BENCH_TEST
// -----------------------------------------------------------------------------

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

    // Matériel initialisé (et moteurs à zéro) avant tout le reste
    motors_init();
    motors_stop();
    encoders_init();
    pid_init(&s_pid_left, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);
    pid_init(&s_pid_right, PID_KP, PID_KI, PID_KD, -1.0f, 1.0f);

    // Calibration gyro au boot, robot immobile (~1 s)
    s_imu_present = imu_init();
    if (s_imu_present) {
        imu_calibrate_gyro();
    }

    // --- TEST BOOT DOUX : ~1 cm avant / ~1 cm arriere (TEMPORAIRE) ---------
    // Valide moteurs + encodeurs + sens SANS deplacement dangereux : arret en
    // Chaque roue testee SEPAREMENT : 2 s en avant puis 2 s en arriere, en
    // affichant les compteurs des DEUX encodeurs a chaque phase, puis 30 s de
    // compteurs pour test a la main. Active par BOOT_BENCH_TEST (config.h),
    // a flasher via banc.sh. ROUES EN L'AIR !
#if BOOT_BENCH_TEST
    boot_test_run();
#endif
    // --- FIN TEST BOOT ------------------------------------------------------

    xTaskCreate(micro_ros_task, "uros_task", 16384, NULL, 5, NULL);
}
