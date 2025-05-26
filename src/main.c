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

 

#define UART_DEVICE_NODE DT_NODELABEL(uart0)

enum 
{
    CMD1_TX = 0x00,
    CMD2_TX = 0x01,
    CMD3_TX = 0x10, // opto
    CMD4_TX = 0x20,
    CMD5_TX = 0x30,
    CMD6_TX = 0x31,
    CMD7_TX = 0x32,

};

enum 
{
    CMD1_RX = 0x30,
    CMD2_RX = 0x31,
    CMD3_RX = 0x32,
    CMD4_RX = 0x40,
};

static void optos_read_handler(struct k_work *work);
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
void build_trame(uint8_t cmd, const uint8_t *pdata, uint8_t size_data);
static void trame_notify(struct k_work *work);
void send_ble(uint8_t *ptrame , int len_trame);

bool notif_enabled = false;

#define TRAME_SIZE 14

static uint8_t uart_buffer[TRAME_SIZE];
static size_t uart_buf_pos = 0;

void uart_cb(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t c;
        int recv = uart_fifo_read(dev, &c, 1);
        if (recv > 0) {
            printk("UART reçu: 0x%02X\n", c);
            
            uart_buffer[uart_buf_pos++] = c;

            if (uart_buf_pos == TRAME_SIZE) {
                // Trame complète reçue
                build_trame(CMD4_TX, uart_buffer, TRAME_SIZE);
                uart_buf_pos = 0; // Réinitialise le buffer pour la prochaine trame
            }
        }
    }
}



static struct bt_conn *current_conn;

/*******************************************************************************/
#define OPTOS_COUNT 4


static const struct gpio_dt_spec optos[OPTOS_COUNT] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios)
};

static struct gpio_callback optos_cbs[OPTOS_COUNT];

static uint8_t optos_state[OPTOS_COUNT] = {0};

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
static uint8_t trame_data_opto_debug[4] = {   0x00, 0x00, 0x00, 0x00 };
static uint8_t trame_data_opto[24] = {   0x00, 0x01, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 ,
                                    0x01, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 ,
                                    0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x01 };


K_WORK_DELAYABLE_DEFINE(optos_read_work, optos_read_handler);

static void optos_read_handler(struct k_work *work)
{
    read_optos_state();
    k_work_schedule(&optos_read_work, TRAME_INTERVAL); // relance toutes les 10 sec
}
static void optos_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    for (int i = 0; i < OPTOS_COUNT; i++) {
        if (BIT(optos[i].pin) & pins) {
            int val = gpio_pin_get_dt(&optos[i]);
            if (val >= 0) {
                trame_data_opto_debug[i] = (uint8_t)val;
                printk("Opto %d changé : %d\n", i, val);
                build_trame(CMD3_TX, trame_data_opto_debug, sizeof(trame_data_opto_debug));
            }
        }
    }
}

void init_optos(void)
{
    for (int i = 0; i < OPTOS_COUNT; i++) {
        if (!device_is_ready(optos[i].port)) {
            printk("Opto %d non prêt\n", i);
            continue;
        }

        gpio_pin_configure_dt(&optos[i], GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&optos[i], GPIO_INT_EDGE_BOTH);

        gpio_init_callback(&optos_cbs[i], optos_handler, BIT(optos[i].pin));
        gpio_add_callback(optos[i].port, &optos_cbs[i]);

        printk("optos %d initialisé sur pin %d\n", i, optos[i].pin);
    }
}

void read_optos_state(void)
{
    for (int i = 0; i < OPTOS_COUNT; i++) {
        int val = gpio_pin_get_dt(&optos[i]);
        if (val >= 0) {
            optos_state[i] = (uint8_t)val;
            printk("Optos %d lu périodiquement : %d\n", i, val);
        }
    }

    build_trame(CMD3_TX, optos_state, OPTOS_COUNT);
}


void build_trame(uint8_t cmd, const uint8_t *pdata, uint8_t size_data)
{
    uint8_t i;
    uint8_t trame_len = size_data + 5; // < + cmd + size + data + checksum + >
    uint8_t *trame_buffer = k_malloc(trame_len);

    if (!trame_buffer) {
        printk("Erreur d'allocation mémoire pour la trame\n");
        return;
    }

    trame_buffer[0] = '<';
    trame_buffer[1] = cmd;
    trame_buffer[2] = size_data;
    memcpy(&trame_buffer[3], pdata, size_data);

    uint8_t checksum = cmd + size_data;
    for (i = 0; i < size_data; i++) {
        checksum += pdata[i];
    }

    trame_buffer[3 + size_data] = checksum;
    trame_buffer[4 + size_data] = '>';

    send_ble(trame_buffer, trame_len);

    k_free(trame_buffer);
}




/* === CCCD callback === */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notification %s\n", notif_enabled ? "activée" : "désactivée");

    if (notif_enabled && current_conn) {

        u_int8_t name[] = "Apli";
        u_int8_t version[] = "1.0";
        u_int8_t info[100] ;
        int len = snprintf((char *)info, sizeof(info), "%s;%s", name, version);
        build_trame(CMD2_TX, info, len );
    }
}



/* === write_cmd from client === */
static ssize_t write_cmd(struct bt_conn *conn,
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
    // En BLE, un service est une collection de caractéristiques (characteristics) qui fournissent des fonctionnalités liées.
   // Un service primaire est un service principal qu’un client BLE peut découvrir pour comprendre ce que le périphérique propose.
    BT_GATT_PRIMARY_SERVICE(&ble_service_uuid.uuid),

    //Cette caractéristique a la propriété de notifier le client quand sa valeur change (en envoyant des notifications BLE).
      BT_GATT_CHARACTERISTIC(&ble_sent_uuid.uuid,
                              BT_GATT_CHRC_NOTIFY,
                              BT_GATT_PERM_READ,
                                NULL, NULL, NULL),
    // Client Characteristic Configuration 
    // Sert à autoriser ou désactiver les notifications ou indications côté client 
    // Dans la structure GATT, le descripteur CCC est lié à la caractéristique précédente   
    // BT_GATT_CCC est un descripteur de la caractéristique notify.           
     BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // C’est la fonction appelée côté serveur (ici dans le firmware) quand le client envoie une écriture sur cette caractéristique. 
    // Cela permet d’interpréter la donnée reçue, déclencher une action, stocker une valeur
    BT_GATT_CHARACTERISTIC(&ble_data_uuid.uuid,
          BT_GATT_CHRC_WRITE ,
            BT_GATT_PERM_WRITE ,
                           NULL, write_cmd, NULL),
    );

    

  
 


void send_ble(uint8_t *ptrame,int len_trame)
{
    if( !notif_enabled)
     return;

    if (current_conn) {
        int err = bt_gatt_notify(current_conn, &trame_svc.attrs[1], ptrame, len_trame);
        if (err) {
            printk("Erreur notification BLE : %d\n", err);
        } else {
            printk("Trame %02X mise à jour envoyée : ",ptrame[2]);
        for (int i = 0; i < len_trame; i++) {
            printk("%02X ", ptrame[i]);
        }
        printk("\n");
        }
    }
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
/* 
static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("MTU négociée : TX = %d, RX = %d\n", tx, rx);
}
static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
}; */

/* === main === */
int main(void)
{
    int err;

    printk("Démarrage BLE\n");

 if (!device_is_ready(uart_dev)) {
        printk("UART device not ready\n");
        return 0 ;
    }

    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    /************************************************************/
    init_optos();
    /*************************************************************/

    if (!gpio_is_ready_dt(&led_emit)) {
        printk("LED Emit non disponible\n");
        return 0;
    }

    err = gpio_pin_configure_dt(&led_emit, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Erreur configuration LED\n");
        return 0;
    }

    
    if (!gpio_is_ready_dt(&led_receive)) {
        printk("LED Receive non disponible\n");
        return 0;
    }

    err = gpio_pin_configure_dt(&led_receive, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Erreur configuration LED Receive\n");
        return 0;
    }

    gpio_pin_set_dt(&led_emit, 1);
    printk("LED allumée au boot\n");
    k_sleep(K_SECONDS(2));
    gpio_pin_set_dt(&led_emit, 0);

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init échouée (%d)\n", err);
        return 0;
    }

    // Les clients pourront voir ton device, lire son nom, et se connecter
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        printk("Échec publicité BLE (%d)\n", err);
        return 0;
    }

    printk("Publicité démarrée\n");

    // Initialisation 1ier appel
    //Remonte l'état des 24 entrées. Chaque x correspond à l'état (0 ou 1) d'une entrée. La trame est émise toutes les 5s ou lors d'un changement d'état sur l'une des IO
    k_work_schedule(&optos_read_work, TRAME_INTERVAL);

    //k_work_schedule(&trame_work, TRAME_INTERVAL);
}
