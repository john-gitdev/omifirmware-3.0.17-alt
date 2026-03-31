#include "button.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/poweroff.h>

#include "haptic.h"
#include "led.h"
#include "mic.h"
#include "speaker.h"
#include "transport.h"
#include "wdog_facade.h"
#ifdef CONFIG_OMI_ENABLE_WIFI
#include "wifi.h"
#endif

#include "imu.h"
#ifdef CONFIG_OMI_ENABLE_OFFLINE_STORAGE
#include "sd_card.h"
#endif

LOG_MODULE_REGISTER(button, CONFIG_LOG_DEFAULT_LEVEL);

extern bool is_off;
extern void set_led_state(void);

// ---------------------------------------------------------------------------
// BLE service
// ---------------------------------------------------------------------------

static void button_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t button_data_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset);

static struct bt_uuid_128 button_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x23BA7924, 0x0000, 0x1000, 0x7450, 0x346EAC492E92));
static struct bt_uuid_128 button_characteristic_data_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x23BA7925, 0x0000, 0x1000, 0x7450, 0x346EAC492E92));

static struct bt_gatt_attr button_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&button_uuid),
    BT_GATT_CHARACTERISTIC(&button_characteristic_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           button_data_read_characteristic,
                           NULL,
                           NULL),
    BT_GATT_CCC(button_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service button_service = BT_GATT_SERVICE(button_service_attr);

static void button_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Client subscribed for notifications");
    } else if (value == 0) {
        LOG_INF("Client unsubscribed from notifications");
    } else {
        LOG_ERR("Invalid CCC value: %u", value);
    }
}

// ---------------------------------------------------------------------------
// Notification values (BLE payload)
// ---------------------------------------------------------------------------

#define NOTIF_DOUBLE_TAP    2
#define NOTIF_VOICE_START   3  // LONG_TAP — voice command begins
#define NOTIF_VOICE_END     5  // BUTTON_RELEASE — voice command ends

static int final_button_state[2] = {0, 0};

static void notify_bt(int value)
{
    final_button_state[0] = value;
    struct bt_conn *conn = get_current_connection();
    if (conn != NULL) {
        bt_gatt_notify(conn, &button_service.attrs[1], &final_button_state, sizeof(final_button_state));
    }
}

// ---------------------------------------------------------------------------
// GPIO / polling setup
// ---------------------------------------------------------------------------

static const struct device *const buttons = DEVICE_DT_GET(DT_ALIAS(buttons));
static const struct gpio_dt_spec usr_btn = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(usr_btn), gpios, {0});

static volatile bool was_pressed = false;

static struct gpio_callback button_cb_data;

static void button_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    was_pressed = (gpio_pin_get_dt(&usr_btn) == 1);
    LOG_INF("Button %s (GPIO)", was_pressed ? "pressed" : "released");
}

// ---------------------------------------------------------------------------
// Timing constants
// ---------------------------------------------------------------------------

#define BUTTON_CHECK_INTERVAL_MS  40    // polling period: 40 ms = 25 Hz
#define DOUBLE_TAP_WINDOW_MS      600   // max gap between tap 1 release and tap 2 press
#define VOICE_PRESS_MS            1000  // hold from cold to start voice command
#define SHUTDOWN_HOLD_MS          3000  // hold on second press to shut down

// Ticks from ms
#define MS_TO_TICKS(ms)  ((ms) / BUTTON_CHECK_INTERVAL_MS)

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------

// States
typedef enum {
    ST_IDLE,           // nothing happening
    ST_PRESSED,        // finger down (first press)
    ST_WAIT_DOUBLE,    // first tap released, waiting for second press within window
    ST_VOICE_ACTIVE,   // voice command fired, waiting for release
    ST_SECOND_PRESSED, // second finger down, timing for shutdown
} BtnFsmState;

// LED toggle — true = main.c controls LED, false = LED off
bool led_state = true;

static BtnFsmState fsm_state = ST_IDLE;
static uint32_t    ticks = 0;        // free-running monotonic tick counter (never reset)
static uint32_t    state_enter_tick; // tick at which we entered the current state
static bool        shutdown_haptic_fired = false;

static uint32_t ticks_in_state(void)
{
    return ticks - state_enter_tick;
}

static uint32_t ms_in_state(void)
{
    return ticks_in_state() * BUTTON_CHECK_INTERVAL_MS;
}

static void enter_state(BtnFsmState next)
{
    fsm_state = next;
    state_enter_tick = ticks;
    shutdown_haptic_fired = false;
}

// ---------------------------------------------------------------------------
// Work handler
// ---------------------------------------------------------------------------

void check_button_level(struct k_work *work_item);
K_WORK_DELAYABLE_DEFINE(button_work, check_button_level);

void check_button_level(struct k_work *work_item)
{
    ticks++;

    bool pressed = was_pressed;

switch (fsm_state) {

    case ST_IDLE:
        if (pressed) {
            LOG_INF("FSM: IDLE -> PRESSED");
            enter_state(ST_PRESSED);
        }
        break;

    case ST_PRESSED:
        if (!pressed) {
            if (ms_in_state() < VOICE_PRESS_MS) {
                LOG_INF("FSM: PRESSED -> WAIT_DOUBLE");
                enter_state(ST_WAIT_DOUBLE);
            } else {
                enter_state(ST_IDLE);
            }
        } else if (ms_in_state() >= VOICE_PRESS_MS) {
            // --- LONG PRESS ACTION ---
            LOG_INF("FSM: Long press detected - toggling LED");
            led_state = !led_state;
            if (!led_state) {
                led_off();
            } else {
                set_led_state();
            }
            // Wait here for release so it doesn't toggle repeatedly
            enter_state(ST_VOICE_ACTIVE);
        }
        break;

    case ST_VOICE_ACTIVE:
        if (!pressed) {
            LOG_INF("FSM: Long press released, returning to IDLE");
            enter_state(ST_IDLE);
        }
        break;

    case ST_WAIT_DOUBLE:
        if (pressed) {
            LOG_INF("FSM: WAIT_DOUBLE -> SECOND_PRESSED");
            enter_state(ST_SECOND_PRESSED);
        } else if (ms_in_state() >= DOUBLE_TAP_WINDOW_MS) {
            // --- SINGLE TAP ACTION: DO NOTHING ---
            LOG_INF("FSM: Single tap confirmed - doing nothing");
            enter_state(ST_IDLE);
        }
        break;

    case ST_SECOND_PRESSED:
        if (!pressed) {
            LOG_INF("FSM: SECOND_PRESSED -> IDLE, double tap");
            notify_bt(NOTIF_DOUBLE_TAP);
            enter_state(ST_IDLE);
        } else {
            if (!shutdown_haptic_fired && ms_in_state() >= SHUTDOWN_HOLD_MS) {
                LOG_INF("Shutdown hold reached, haptic + power off");
                shutdown_haptic_fired = true;
#ifdef CONFIG_OMI_ENABLE_HAPTIC
                play_haptic_milli(200);
                k_msleep(300);
#endif
                turnoff_all();
            }
        }
        break;
    }

    k_work_reschedule(&button_work, K_MSEC(BUTTON_CHECK_INTERVAL_MS));
}

// ---------------------------------------------------------------------------
// BLE read characteristic
// ---------------------------------------------------------------------------

static ssize_t button_data_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset)
{
    LOG_INF("button_data_read_characteristic: %d", final_button_state[0]);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &final_button_state, sizeof(final_button_state));
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

static int button_register_callback(void)
{
    int ret;

    ret = gpio_pin_configure_dt(&usr_btn, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure button GPIO (%d)", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&usr_btn, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure button interrupt (%d)", ret);
        return ret;
    }

    gpio_init_callback(&button_cb_data, button_gpio_callback, BIT(usr_btn.pin));
    gpio_add_callback(usr_btn.port, &button_cb_data);

    LOG_INF("Button GPIO interrupt registered");
    return 0;
}

int button_init(void)
{
    int ret;

    if (!device_is_ready(buttons)) {
        LOG_ERR("Buttons device not ready");
        return -ENODEV;
    }

    ret = pm_device_runtime_get(buttons);
    if (ret < 0) {
        LOG_ERR("Failed to enable buttons device (%d)", ret);
        return ret;
    }

    ret = button_register_callback();
    if (ret < 0) {
        LOG_ERR("Failed to register button callback (%d)", ret);
        return ret;
    }

    return 0;
}

void activate_button_work(void)
{
    k_work_schedule(&button_work, K_MSEC(BUTTON_CHECK_INTERVAL_MS));
}

void register_button_service(void)
{
    bt_gatt_service_register(&button_service);
}

// ---------------------------------------------------------------------------
// Power off
// ---------------------------------------------------------------------------

void turnoff_all(void)
{
    int rc;

    led_off();
    is_off = true;

#ifdef CONFIG_OMI_ENABLE_HAPTIC
    k_msleep(300);
    haptic_off();
#endif

    k_msleep(1000);

    transport_off();
    k_msleep(300);

    mic_off();
    k_msleep(100);

#ifdef CONFIG_OMI_ENABLE_SPEAKER
    speaker_off();
    k_msleep(100);
#endif

#ifdef CONFIG_OMI_ENABLE_ACCELEROMETER
    accel_off();
    k_msleep(100);
#endif

    if (is_sd_on()) {
        app_sd_off();
    }
    k_msleep(300);

#ifdef CONFIG_OMI_ENABLE_BUTTON
    pm_device_runtime_put(buttons);
    k_msleep(100);
#endif

#ifdef CONFIG_OMI_ENABLE_USB
    NRF_USBD->INTENCLR = 0xFFFFFFFF;
#endif

    LOG_INF("System powering off");

    rc = gpio_pin_configure_dt(&usr_btn, GPIO_INPUT);
    if (rc < 0) {
        LOG_ERR("Could not configure usr_btn GPIO (%d)", rc);
        return;
    }

    rc = gpio_pin_interrupt_configure_dt(&usr_btn, GPIO_INT_LEVEL_LOW);
    if (rc < 0) {
        LOG_ERR("Could not configure usr_btn GPIO interrupt (%d)", rc);
        return;
    }

#ifdef CONFIG_OMI_ENABLE_WIFI
    wifi_turn_off();
#endif

    rc = watchdog_deinit();
    if (rc < 0) {
        LOG_ERR("Failed to deinitialize watchdog (%d)", rc);
        return;
    }

    lsm6dsl_time_prepare_for_system_off();
    k_msleep(1000);
    LOG_INF("Entering system off; press usr_btn to restart");

    sys_poweroff();
}

// ---------------------------------------------------------------------------
// Legacy FSM accessors (kept for API compatibility with main.c)
// ---------------------------------------------------------------------------

FSM_STATE_T get_current_button_state(void)
{
    // Map internal FSM state to legacy FSM_STATE_T for any callers
    switch (fsm_state) {
    case ST_IDLE:           return IDLE;
    case ST_VOICE_ACTIVE:   return GRACE;
    default:                return IDLE;
    }
}

void force_button_state(FSM_STATE_T state)
{
    // Legacy shim — force to IDLE or GRACE only
    if (state == IDLE) {
        enter_state(ST_IDLE);
    }
}
