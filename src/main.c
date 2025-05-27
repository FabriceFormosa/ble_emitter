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


#define FIRMWARE_VERSION_STRING "1.0.0"
#define BLE_VERSION_STRING "5.0"
#define BT_UUID_MODEL_NUMBER_STRING BT_UUID_DECLARE_16(0x2A24)
#define BT_UUID_FIRMWARE_REVISION_STRING BT_UUID_DECLARE_16(0x2A26)
#define UUID_BLE_SERV_VAL   BT_UUID_128_ENCODE(0x0003ABCD, 0x0000, 0x1000, 0x8000, 0x00805F9B0131)
/*31201 TX Serveur - RX Client*/
#define UUID_BLE5SENT_VAL   BT_UUID_128_ENCODE(0x00031201, 0x0000, 0x1000, 0x8000, 0x00805F9B0130)
/*31202 RX Serveur - TX Client */
#define UUID_BLE5DATA_VAL   BT_UUID_128_ENCODE(0x00031202, 0x0000, 0x1000, 0x8000, 0x00805F9B0130)
// Zephyr limite le nom Bluetooth (Device Name) à 248 caractères
#define BLE_NAME_MAX_LEN 48
#define DEVICE_NAME "NRF52_Emitter"
#define START_CHAR '<'
#define END_CHAR   '>'
#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define UART_BUF_SIZE 256
#define OPTOS_COUNT 4
/* === LED === */
#define LED0_EMIT DT_ALIAS(led0)
#define LED1_RECEIVE DT_ALIAS(led1)
/* === BLE notification interval === */
#define TRAME_INTERVAL_OPTOS K_SECONDS(5)
#define TRAME_INTERVAL_INFOS_APPLI K_SECONDS(10)

enum {
    CMD1 = 0x00,       // CMD1
    CMD2 = 0x02,       // CMD2
    CMD3 = 0x10,       // CMD3
    CMD4 = 0x20,       // CMD4
    CMD5 = 0x30,       // CMD5
    CMD6 = 0x31,       // CMD6
    CMD7 = 0x32,       // CMD7
    CMD8 = 0x40,       // CMD8
    CMD9 = 0x41,       // CMD9
    CMD10 = 0x4F,       // CMD10
    CMD11 = 0xFF       // CMD11
};

uint8_t calc_checksum_ascii_sum(const char *trame);
void build_ascii_trame_dyn(uint8_t cmd, const uint8_t *data, uint8_t data_size);
static void optos_read_handler(struct k_work *work);
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static void trame_notify(struct k_work *work);
void send_ble(uint8_t *ptrame , int len_trame);
void app_info_work_handler(struct k_work *work);

static struct bt_conn *current_conn;
static uint8_t BleNewName[BLE_NAME_MAX_LEN] ={0};
bool notif_enabled = false;
static char uart_buffer[UART_BUF_SIZE];
static int uart_buf_pos = 0;
static bool in_trame = false;
static int cpt_semi = 0;
static const struct gpio_dt_spec optos[OPTOS_COUNT] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios)
};

static struct gpio_callback optos_cbs[OPTOS_COUNT];
static char optos_state[OPTOS_COUNT] = {0};

static struct bt_uuid_128 ble_service_uuid = BT_UUID_INIT_128(UUID_BLE_SERV_VAL);
static struct bt_uuid_128 ble_data_uuid    = BT_UUID_INIT_128(UUID_BLE5DATA_VAL);
static struct bt_uuid_128 ble_sent_uuid    = BT_UUID_INIT_128(UUID_BLE5SENT_VAL);


static const struct gpio_dt_spec led_emit = GPIO_DT_SPEC_GET(LED0_EMIT, gpios);
static const struct gpio_dt_spec led_receive = GPIO_DT_SPEC_GET(LED1_RECEIVE, gpios);



/* === Données à transmettre === */
static char trame_data_opto_debug[4] = {   '0', '0', '0', '0' };

static char trame_data_opto[24] = {   '1', '0', '0', '0','0',
                                      '0', '0', '0', '0','0',
                                      '0', '0', '0', '0','0',
                                      '0', '0', '0', '0','0',
                                      '0', '0', '0', '0',
                                     };







void uart_cb(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t buf[64];  // Lire jusqu’à 64 octets à la fois
        int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
        if (recv_len <= 0) {
            return;
        }

 
        for (int i = 0; i < recv_len; i++) 

        for (int i = 0; i < recv_len; i++) {
            char c = buf[i];

            // Détection début de trame
            if (c == '$') {
                cpt_semi = 0;
                uart_buf_pos = 0;
                in_trame = true;
                uart_buffer[uart_buf_pos++] = c;
            }
            else if (in_trame) {
                if (uart_buf_pos < UART_BUF_SIZE - 1) {
                    uart_buffer[uart_buf_pos++] = c;

                    if( c == ';')
                    {
                        cpt_semi++;
                    }

                    if (cpt_semi == 4) {
                        //uart_buffer[uart_buf_pos] = '\0'; // fin de chaîne
                        in_trame = false;
                        build_ascii_trame_dyn(CMD4,uart_buffer,uart_buf_pos);
                        printk("Trame Buffer UART data size : %d \n",uart_buf_pos);
                        uart_buf_pos = 0;
                    }
                } else {
                    printk("Buffer UART overflow\n");
                    uart_buf_pos = 0;
                    in_trame = false;
                }
            }
        }
    }
}















K_WORK_DELAYABLE_DEFINE(optos_read_work, optos_read_handler);
K_WORK_DELAYABLE_DEFINE(app_info_work, app_info_work_handler);

// === Handler appelé toutes les 10 secondes ===
void app_info_work_handler(struct k_work *work)
{
    const char name[] = "Apli";
    const char version[] = "1.0";
    uint8_t info[100];
    

    uint8_t len = snprintf((char *)info, sizeof(info), "%s;%s", name, version);
    if (len > 0 && len < sizeof(info)) {
      
      // build_ascii_trame_dyn(CMD2, info,len);
    } else {
        printk("Erreur lors de la création de la trame\n");
    }

    // Replanifie le travail dans 10 secondes
    k_work_schedule(&app_info_work, TRAME_INTERVAL_INFOS_APPLI);
}

static void optos_read_handler(struct k_work *work)
{
    read_optos_state();
    k_work_schedule(&optos_read_work, TRAME_INTERVAL_OPTOS); // relance toutes les 10 sec
}
static void optos_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    for (int i = 0; i < OPTOS_COUNT; i++) {
        if (BIT(optos[i].pin) & pins) {
            int val = gpio_pin_get_dt(&optos[i]);
            if (val >= 0) {
                trame_data_opto[i] = (val == 0) ? '0' : '1';
               // printk("Opto %d changé : %d\n", i, val);
                build_ascii_trame_dyn(CMD3, trame_data_opto, sizeof(trame_data_opto));
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
        memset( optos_state,'0',sizeof(optos_state));
        if (val >= 0) {
            optos_state[i] = (val == 0) ? '0' : '1';
            
        }
    }

    //build_ascii_trame_dyn(CMD3, optos_state, OPTOS_COUNT);
    build_ascii_trame_dyn(CMD3, trame_data_opto, sizeof(trame_data_opto));
}


/* === CCCD callback === */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notification %s\n", notif_enabled ? "activée" : "désactivée");
   

}


void build_ascii_trame_dyn(uint8_t cmd, const uint8_t *data, uint8_t data_size)
{
    // Taille max : 1 ( < ) + 2 (cmd) + 1 ( ; ) + 2 (size) + 1 ( ; ) + data_size + 1 ( ; ) + 2 (checksum) + 1 ( > ) + 1 (\0)
    size_t trame_len = 1 + 2 + 1 + 2 + 1 + data_size + 1 + 2 + 1 + 1;

    char *trame = k_malloc(trame_len);
    if (!trame) {
        printk("Erreur allocation mémoire trame\n");
        return;
    }

    // Calcul du checksum
    uint8_t checksum = 0;
    snprintf(trame, trame_len,
        "<%02X;%02X;%.*s;%02X>",
        cmd, data_size, data_size, data, checksum);

   
    checksum = calc_checksum_ascii_sum(trame);

    // Construction de la trame au format "<cmd;size;data;checksum>"
    int len = snprintf(trame, trame_len,
        "<%02X;%02X;%.*s;%02X>",
        cmd, data_size, data_size, data, checksum);

    printk("Construction de la trame au format <cmd;size;data;checksum> : %s \n",trame);

    if (len > 0 && len < trame_len) {
        send_ble((uint8_t *)trame, len);
        
    } else {
        printk("Erreur création trame ASCII dyn\n");
    }

    k_free(trame);
}



uint8_t calc_checksum_ascii_sum(const char *trame) {
    uint8_t sum = 0;

    const char *start = strchr(trame, START_CHAR);
    char *last_sep = strrchr(trame, ';');

    size_t checksum_len = last_sep - start;



   // printk("trame :%s  checksum_len : %d \n",trame,checksum_len);

    // On commence à 1 pour ignorer le '<'
    
    for (size_t i = 1; i <= checksum_len ; i++) {

      // printk("ind : %d - sum : 0x%02X -- car:%c \n",i,sum, (char)trame[i] );
        sum += (uint8_t)trame[i];
      // k_sleep(K_MSEC(100));
        
    }

   // printk("checksum : 0x%02X  - dec : %i \n",sum,sum);
    return sum;  // modulo 256 implicite car uint8_t
}

void parse_ascii_trame_string(const char *input) {

    // Cherche délimiteurs < et >
    const char *start = strchr(input, START_CHAR);
    const char *end = strchr(input, END_CHAR);
    if (!start || !end || end <= start) {
        printf("Délimiteurs invalides\n");
        return;
    }

    size_t len = end - start - 1;
    if (len < 5) { // min <c;s;d;c>
        printf("Trame trop courte\n");
        return;
    }

    // Copie la trame sans < et >
    char *trame_str = malloc(len + 1);
    if (!trame_str) {
        printf("Erreur allocation mémoire\n");
        return;
    }
    memcpy(trame_str, start + 1, len);
    trame_str[len] = '\0';

    // Découpage manuel des champs par ';'
    char *fields[4] = {0};
    int field_idx = 0;
    char *p = trame_str;
    char *next = NULL;

    while (field_idx < 4) {
        next = strchr(p, ';');
        if (field_idx < 3) {
            if (next) {
                *next = '\0';
                fields[field_idx++] = p;
                p = next + 1;
            } else {
                // Moins de 4 champs
                break;
            }
        } else {
            // 4e champ, prend tout jusqu'à la fin
            fields[field_idx++] = p;
            break;
        }
    }

    if (field_idx != 4) {
        printf("Format incorrect (nombre de champs != 4)\n");
        free(trame_str);
        return;
    }

    // Analyse des champs
    int cmd = atoi(fields[0]);
    int size = atoi(fields[1]);
    char *data = fields[2];
    // checksum reçu en hexadécimal, ex "44"
    int checksum_received = (int)strtol(fields[3], NULL, 16);

    // Vérification taille data
    if ((int)strlen(data) != size) {
        printf("Taille data incorrecte (attendu=%d, reçu=%ld)\n", size, strlen(data));
        free(trame_str);
        return;
    }

    // Calcul checksum
    uint8_t checksum_calcul = calc_checksum_ascii_sum(trame_str);

    if (checksum_received != checksum_calcul) {
        printf("Checksum invalide (attendu=0x%02X, reçu=0x%02X)\n", checksum_calcul, checksum_received);
        free(trame_str);
        return;
    }

    printf("Trame valide: cmd=0x%X (%d), size=%d, data='%s', checksum=0x%02X\n",
           cmd, cmd, size, data, checksum_received);



   switch(cmd)
    {

        // permet de demander à l'interface de retourner le nom du module Bluetooth.
        // TRAME CLIENT : 
        case CMD5 : // 0x30
        {
            
            const char *name = bt_get_name();
            printk("CMD1_RX lecture name :%s longueur name:%d \n",name,strlen(name));

            // Retourne le nom du module Bluetooth.
            //build_trame(CMD5_TX,name,strlen(name));
            //build_ascii_trame_dyn(CMD5,name,strlen(name));
        }break;
        
        // Permet de demander à l'interface de retourner le code pin de l'interface Bluetooth.
        case CMD6 : // 0x31
        {
  

        }break;
         
        // Permet de demander à l'interface de retourner la MAC adresse de l'interface Bluetooth.
        // TRAME CLIENT : 3C 32 00 32 3E
        case CMD7 : // 0x32
        {
          
            bt_addr_le_t addr;
            size_t count = 1;
             
            bt_id_get(&addr, &count); // Lecture 1 adresse
       
            printk("CMD3_RX lecture MAC ADRESS BT Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
            addr.a.val[5], addr.a.val[4], addr.a.val[3],
            addr.a.val[2], addr.a.val[1], addr.a.val[0]); 

            static uint8_t mac_buf[6];
            uint8_t reversed_addr[6] = {
                addr.a.val[5], addr.a.val[4], addr.a.val[3],
                addr.a.val[2], addr.a.val[1], addr.a.val[0]
            };
            memcpy(mac_buf, reversed_addr, sizeof(mac_buf));

           // build_trame(CMD7_TX,mac_buf,sizeof(mac_buf));
           //build_ascii_trame_dyn(CMD7,mac_buf,sizeof(mac_buf));
            

        }break;
        
        // Permet de définir le nom du module Bluetooth.
        // TRAME CLIENT : 3C 40 06 4D 6F 6E 4E 6F 6D 46 3E  "MonNom"
 /*        case CMD8 : // 0x40
        {
            int err = 0;

            char name_buf[size+1]; // nouveau nom sans car. de fin de chaine ?
            memcpy(name_buf,data,size);
            name_buf[size]='\0';
            

            if( size <  sizeof(BleNewName)  )
            {
                memset(BleNewName,0,sizeof(BleNewName));
                memcpy(BleNewName,name_buf,sizeof(name_buf));
                printk("CMD4_RX BleNewName :%s longueur BleNewName:%d \n",BleNewName,strlen(BleNewName));

                // Retourne le nouveau nom du module Bluetooth effectif si validation commande 0x4F.
                //build_trame(CMD5_TX,BleNewName,strlen(BleNewName));
                build_ascii_trame_dyn(CMD5,BleNewName,strlen(BleNewName));
            }
            
 
        }break; */
        
        // Permet de définir le code pin du module Bluetooth. Si la commande est correctement exécutée, 
        // la carte retourne un message 0x31
        case CMD9 : // 0x41
        {

        }break;
        
        // Permet de valider et mettre à jour les paramètres du module Bluetooth
        // TRAME CLIENT : 3C 4F 00 4F 3E
        case CMD10 :
        {
            size_t len = strlen(BleNewName);

            if (len > 0 && len <= BLE_NAME_MAX_LEN) {
           // Mise à jour du nom Bluetooth
            int err = bt_set_name(BleNewName);
            if (err != 0) {
                printk("Erreur bt_set_name : %d\n", err);
                
            }
            else
            {
                // il faut arrêter et redémarrer l'advertising
                // pour que le nouveau nom soit pris en compte lors des annonces.
                bt_le_adv_stop();
                bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);

            }
 
        }

        }break;

        // Permet de générer un reset de la carte
        case CMD11 :
        {

        }break;

        default:
        {

        }break;
    }


    free(trame_str);


    parse_error:
    printk("Erreur lors du parsing\n");
    free(trame_str);
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

    printk("Données reçues %s (len=%d) : \n",buf, len);

    parse_ascii_trame_string(buf);

    //printk("%s ",

    for (int i = 0; i < len; i++) {
        printk(" %02X ", ((uint8_t *)buf)[i]);
    }

    //printk("\n");

   //   parse_ascii_trame_dyn(buf,len);

 //   parse_trame(buf,len);

  /*   if (*value == 0x01) {
        gpio_pin_set_dt(&led_receive, 1);
        printk("LED allumée via BLE\n");
    } else if (*value == 0x00) {
        gpio_pin_set_dt(&led_receive, 0);
        printk("LED éteinte via BLE\n");
    } else {
        printk("Valeur inconnue : 0x%02X\n", *value);
    } */

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

    

void send_ble(uint8_t *ptrame, int len_trame)
{
    if (!notif_enabled)
        return;

    if (current_conn) {
        int err = bt_gatt_notify(current_conn, &trame_svc.attrs[1], ptrame, len_trame);
        if (err) {
            printk("Erreur notification BLE : %d\n", err);
        } else {

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

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    printk("MTU négociée : TX = %d, RX = %d\n", tx, rx);
}
static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
};


void print_bt_name(void)
{
    const char *name = bt_get_name();
    printk("Nom Bluetooth : %s\n", name);
   
}


/* === main === */
int main(void)
{

   

    int err;

    print_bt_name();

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
    k_work_schedule(&optos_read_work, TRAME_INTERVAL_OPTOS);

    k_work_schedule(&app_info_work, TRAME_INTERVAL_INFOS_APPLI);


}
