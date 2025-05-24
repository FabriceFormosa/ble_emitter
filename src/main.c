#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>


#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define TRAME_DATA_MAX_LEN 14

struct trame_ble {
    uint8_t start;
    uint8_t cmd;
    uint8_t size; // Nombre de caractères contenu dans le champ data
    uint8_t data[TRAME_DATA_MAX_LEN];
    uint8_t checksum;
    uint8_t end;
};

static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

void uart_cb(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t buf[16];
        int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
        for (int i = 0; i < recv_len; i++) {
            uint8_t c = buf[i];
            // traite c
            printk("UART reçu: 0x%02X\n", c);
        }
    }
}

void build_trame(struct trame_ble *trame, uint8_t cmd, const uint8_t *data, uint8_t size);
static void trame_notify(struct k_work *work);
void send_ble_update(struct trame_ble *ptrame);
K_WORK_DELAYABLE_DEFINE(trame_work, trame_notify);

static struct bt_conn *current_conn;

/*******************************************************************************/
#define BTN_COUNT 4

static const struct gpio_dt_spec buttons[BTN_COUNT] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios)
};

static struct gpio_callback button_cbs[BTN_COUNT];

//extern struct bt_conn *current_conn; // défini dans ton code BLE
//extern uint8_t trame_data[14];       // supposé déjà défini




/************************************************************************************/
#define BT_UUID_MODEL_NUMBER_STRING BT_UUID_DECLARE_16(0x2A24)
#define BT_UUID_FIRMWARE_REVISION_STRING BT_UUID_DECLARE_16(0x2A26)


#define UUID_BLE_SERV_VAL   BT_UUID_128_ENCODE(0x0003ABCD, 0x0000, 0x1000, 0x8000, 0x00805F9B0131)
/*31201 TX Serveur - RX Client*/
#define UUID_BLE5SENT_VAL   BT_UUID_128_ENCODE(0x00031201, 0x0000, 0x1000, 0x8000, 0x00805F9B0130)
/*31202 RX Serveur - TX Client */
#define UUID_BLE5DATA_VAL   BT_UUID_128_ENCODE(0x00031202, 0x0000, 0x1000, 0x8000, 0x00805F9B0130)


static struct bt_uuid_128 ble_service_uuid = BT_UUID_INIT_128(UUID_BLE_SERV_VAL);
static struct bt_uuid_128 ble_data_uuid    = BT_UUID_INIT_128(UUID_BLE5DATA_VAL);
static struct bt_uuid_128 ble_sent_uuid    = BT_UUID_INIT_128(UUID_BLE5SENT_VAL);

#define FIRMWARE_VERSION_STRING "1.0.0"
#define BLE_VERSION_STRING "5.0"

/* === LED === */
#define LED0_EMIT DT_ALIAS(led0)
#define LED1_RECEIVE DT_ALIAS(led1)
static const struct gpio_dt_spec led_emit = GPIO_DT_SPEC_GET(LED0_EMIT, gpios);
static const struct gpio_dt_spec led_receive = GPIO_DT_SPEC_GET(LED1_RECEIVE, gpios);

/* === BLE notification interval === */
#define TRAME_INTERVAL K_SECONDS(5)

/* === Données à transmettre === */
static uint8_t trame_data_tor[24] = {   0x00, 0x01, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 ,
                                    0x01, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 ,
                                    0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x01 };

static uint8_t trame_data[14] = {   0x01, 0x01, 0x01, 0x01,0x01, 0x01, 0x01,
                                    0x01, 0x01, 0x01, 0x01,0x01, 0x01, 0x01 };



static uint8_t trame_cmd = 0x10;// envoi TOR 0x20; // Envoi Pesée





const struct device *uart1 = DEVICE_DT_GET(DT_NODELABEL(uart0));



static void button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (BIT(buttons[i].pin) & pins) {
            int val = gpio_pin_get_dt(&buttons[i]);
            if (val >= 0) {
                trame_data[i] = (uint8_t)val;
                printk("Bouton %d changé : %d\n", i, val);

                  struct trame_ble trame;
                    
                build_trame(&trame, trame_cmd, trame_data, sizeof(trame_data));

                send_ble_update(&trame);
            }
        }
    }
}

void init_buttons(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (!device_is_ready(buttons[i].port)) {
            printk("Bouton %d non prêt\n", i);
            continue;
        }

        gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&buttons[i], GPIO_INT_EDGE_BOTH);

        gpio_init_callback(&button_cbs[i], button_handler, BIT(buttons[i].pin));
        gpio_add_callback(buttons[i].port, &button_cbs[i]);

        printk("Bouton %d initialisé sur pin %d\n", i, buttons[i].pin);
    }
}



void build_trame(struct trame_ble *trame, uint8_t cmd, const uint8_t *data, uint8_t size)
{
    memcpy(trame->data, data, size);

    trame->start = '<';
    trame->cmd = cmd;
    trame->size = size;

     
   // Le checksum est calculé en effectuant la somme de tous les octets 
   // composants la trame à l'exception de l'octet de début de trame, 
   // de fin de trame et de ceux contenus dans le champ checksum

    // Calcul du checksum : somme de cmd + size + data[]
   // Calcul du checksum = cmd + size + chaque octet de data[]
    uint8_t checksum = 0;
    checksum += trame->cmd;
    checksum += trame->size;
    for (uint8_t i = 0; i < size; i++) {
        checksum += trame->data[i];
    }
    trame->checksum = checksum;

    trame->end = '>';
}



/* === CCCD callback === */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notification %s\n", notif_enabled ? "activée" : "désactivée");

    if (notif_enabled && current_conn) {

        u_int8_t name[] = "Apli";
        u_int8_t version[] = "1.0";
        u_int8_t info[100] ;
       snprintf((char *)info, sizeof(info), "%s;%s", name, version);

        struct trame_ble trame;
                    
        build_trame(&trame, 0x01, info, sizeof(info));

        send_ble_update(&trame);


        k_work_schedule(&trame_work, TRAME_INTERVAL);
    }
}



/* === read_cmd from client === */
static ssize_t read_cmd(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr,
                            const void *buf,
                            uint16_t len,
                            uint16_t offset,
                            uint8_t flags)
{
    if (len < 1) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    const uint8_t *value = buf;

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    printk("Commande reçue de %s \n", addr);

    printk("Données reçues (len=%d) : ", len);
    for (int i = 0; i < len; i++) {
        printk("0x%02X ", ((uint8_t *)buf)[i]);
    }
    printk("\n");

    if (*value == 0x01) {
        gpio_pin_set_dt(&led_receive, 1);
        printk("LED allumée via BLE\n");
    } else if (*value == 0x00) {
        gpio_pin_set_dt(&led_receive, 0);
        printk("LED éteinte via BLE\n");
    } else {
        printk("Valeur inconnue : 0x%02X\n", *value);
    }

    return len;
}

/* === Service BLE === */

static ssize_t ble_version_read(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf,
                                uint16_t len,
                                uint16_t offset)
{
    const char *version = BLE_VERSION_STRING;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, version, strlen(version));
}

static ssize_t firmware_version_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf,
                                     uint16_t len,
                                     uint16_t offset)
{
    const char *fw = FIRMWARE_VERSION_STRING;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, fw, strlen(fw));
}

BT_GATT_SERVICE_DEFINE(device_info_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),

    BT_GATT_CHARACTERISTIC(BT_UUID_FIRMWARE_REVISION_STRING,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           firmware_version_read, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_MODEL_NUMBER_STRING,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           ble_version_read, NULL, NULL)
);

BT_GATT_SERVICE_DEFINE(trame_svc,
    BT_GATT_PRIMARY_SERVICE(&ble_service_uuid.uuid),


      BT_GATT_CHARACTERISTIC(&ble_sent_uuid.uuid,
                                BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                       NULL, NULL, NULL),
                       
     BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&ble_data_uuid.uuid,
          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ,
                       BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
    
                           NULL, read_cmd, NULL),

    

  
 
);



void send_ble_update(struct trame_ble *ptrame)
{
  
    uint8_t *p = ptrame;
    

    if (current_conn) {
        int err = bt_gatt_notify(current_conn, &trame_svc.attrs[1], ptrame, sizeof(struct trame_ble));
        if (err) {
            printk("Erreur notification BLE : %d\n", err);
        } else {
            printk("Trame mise à jour envoyée : ");
        for (int i = 0; i < sizeof(struct trame_ble); i++) {
            printk("%02X ", *p++);
        }
        printk("\n");
        }
    }
}
/* === Notification périodique === */
static void trame_notify(struct k_work *work)
{
    if (!current_conn) return;


    
    struct trame_ble trame;
     uint8_t *ptrame = (uint8_t *)&trame;
    build_trame(&trame, trame_cmd, trame_data, sizeof(trame_data));
    



    int err = bt_gatt_notify(current_conn, &trame_svc.attrs[2], &trame, sizeof(trame));
    
    if (err) {
        printk("Erreur de notification : %d\n", err);
    } else {
        printk("Trame envoyée : ");
        for (int i = 0; i < sizeof(trame); i++) {
            printk("%02X ", ptrame[i]);
        }
        printk("\n");

        gpio_pin_set_dt(&led_emit, 1);
        k_msleep(100);
        gpio_pin_set_dt(&led_emit, 0);
    }

    k_work_schedule(&trame_work, TRAME_INTERVAL);
}

/* === Connexion BLE === */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err == 0) {
        printk("Connecté\n");
        current_conn = conn;
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        printk("Device connected: %s\n", addr);

        
 
     

        
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Déconnecté (raison %d)\n", reason);
    current_conn = NULL;
}


BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    
};

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("MTU négociée : TX = %d, RX = %d\n", tx, rx);
}
static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
};

/* === main === */
void main(void)
{
    int err;

    printk("Démarrage BLE\n");

 if (!device_is_ready(uart_dev)) {
        printk("UART device not ready\n");
        return;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    /************************************************************/
    init_buttons();
    /*************************************************************/

    if (!gpio_is_ready_dt(&led_emit)) {
        printk("LED Emit non disponible\n");
        return;
    }

    err = gpio_pin_configure_dt(&led_emit, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Erreur configuration LED\n");
        return;
    }

    
    if (!gpio_is_ready_dt(&led_receive)) {
        printk("LED Receive non disponible\n");
        return;
    }

    err = gpio_pin_configure_dt(&led_receive, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Erreur configuration LED Receive\n");
        return;
    }

    gpio_pin_set_dt(&led_emit, 1);
    printk("LED allumée au boot\n");
    k_sleep(K_SECONDS(2));
    gpio_pin_set_dt(&led_emit, 0);

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init échouée (%d)\n", err);
        return;
    }

    

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        printk("Échec publicité BLE (%d)\n", err);
        return;
    }

    printk("Publicité démarrée\n");
}
