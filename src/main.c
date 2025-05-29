
/**
 * @file main.c
 * @brief Application Zephyr BLE + UART + GPIO
 *
 * Gère :
 *  - 8 entrées opto-isolées en GPIO avec interruptions (changement d'état)
 *  - Réception UART de trames structurées
 *  - Envoi de trames ASCII structurées via BLE
 */

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
#include <stdlib.h>

#define FIRMWARE_VERSION_STRING "1.0.0"
#define BLE_VERSION_STRING "5.0"
#define BT_UUID_MODEL_NUMBER_STRING BT_UUID_DECLARE_16(0x2A24)
#define BT_UUID_FIRMWARE_REVISION_STRING BT_UUID_DECLARE_16(0x2A26)
#define UUID_BLE_SERV_VAL BT_UUID_128_ENCODE(0x0003ABCD, 0x0000, 0x1000, 0x8000, 0x00805F9B0131)
/*31201 TX Serveur - RX Client*/
#define UUID_BLE5SENT_VAL BT_UUID_128_ENCODE(0x00031201, 0x0000, 0x1000, 0x8000, 0x00805F9B0130)
/*31202 RX Serveur - TX Client */
#define UUID_BLE5DATA_VAL BT_UUID_128_ENCODE(0x00031202, 0x0000, 0x1000, 0x8000, 0x00805F9B0130)
// Zephyr limite le nom Bluetooth (Device Name) à 248 caractères
#define BLE_NAME_MAX_LEN 48
#define DEVICE_NAME "NRF52_Emitter"
#define START_CHAR '<'
#define END_CHAR '>'
#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define UART_BUF_SIZE 256

/* === LED === */
#define LED0_EMIT DT_ALIAS(led0)
#define LED1_RECEIVE DT_ALIAS(led1)
/* === BLE notification interval === */
#define TRAME_INTERVAL_OPTOS K_SECONDS(5)
#define TRAME_INTERVAL_INFOS_APPLI K_SECONDS(10)

enum
{
    CMD1 = 0x00,  // CMD1
    CMD2 = 0x02,  // CMD2
    CMD3 = 0x10,  // CMD3
    CMD4 = 0x20,  // CMD4
    CMD5 = 0x30,  // CMD5
    CMD6 = 0x31,  // CMD6
    CMD7 = 0x32,  // CMD7
    CMD8 = 0x40,  // CMD8
    CMD9 = 0x41,  // CMD9
    CMD10 = 0x4F, // CMD10
    CMD11 = 0xFF  // CMD11
};

static uint8_t calc_checksum_ascii_sum(const char *trame);
static void build_ascii_trame_dyn(uint8_t cmd, const uint8_t *data, uint8_t data_size);
static void optos_read_handler(struct k_work *work);
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static void send_ble(uint8_t *ptrame, int len_trame);
static void app_info_work_handler(struct k_work *work);

static struct bt_conn *current_conn;
static uint8_t BleNewName[BLE_NAME_MAX_LEN] = {0};
static bool notif_enabled = false;
static char uart_buffer[UART_BUF_SIZE];
static int uart_buf_pos = 0;
static bool in_trame = false;
static int cpt_semi = 0;


/**
 * @brief UUIDs du service et des caractéristiques BLE personnalisées
 *
 * Ces structures statiques définissent les identifiants uniques (UUID 128 bits)
 * pour le service BLE personnalisé et ses deux caractéristiques :
 * - ble_service_uuid : UUID du service BLE principal
 * - ble_data_uuid    : UUID de la caractéristique pour envoyer des données
 * - ble_sent_uuid    : UUID de la caractéristique pour recevoir des données
 */
static struct bt_uuid_128 ble_service_uuid = BT_UUID_INIT_128(UUID_BLE_SERV_VAL);
static struct bt_uuid_128 ble_data_uuid = BT_UUID_INIT_128(UUID_BLE5DATA_VAL);
static struct bt_uuid_128 ble_sent_uuid = BT_UUID_INIT_128(UUID_BLE5SENT_VAL);


/**
 * @brief Travaux différés périodiques (k_work)
 *
 * Définition de deux travaux différés avec planification :
 * - optos_read_work : déclenché régulièrement pour lire l’état des entrées opto
 * - app_info_work   : envoie périodiquement les infos de l’application (nom/version)
 *
 * Ces objets sont créés avec K_WORK_DELAYABLE_DEFINE, ce qui permet de
 * les planifier avec k_work_schedule().
 *
 */

K_WORK_DELAYABLE_DEFINE(optos_read_work, optos_read_handler);
K_WORK_DELAYABLE_DEFINE(app_info_work, app_info_work_handler);

/* Buffers pour état optocoupleurs */

/**
 * @brief Trame représentant l’état actuel des optos (format ASCII '0'/'1')
 */
static char trame_data_opto[24] = {
    '0','0','0','0','0','0','0','0',
    '0','0','0','0','0','0','0','0',
    '0','0','0','0','0','0','0','0',
};

/**
 * @brief Trame représentant l’état précédent des optos (pour détection changement)
 */
static char prev_trame_data_opto[24] = {
    '0','0','0','0','0','0','0','0',
    '0','0','0','0','0','0','0','0',
    '0','0','0','0','0','0','0','0',
};

#define NB_PINS 8
#define GPIO_NODE DT_NODELABEL(gpio0)

// Liste des pins à configurer
// The following are default pin settings:
// • P0.00 and P0.01 are used for the 32.768 kHz crystal and are not available on the connectors. See
// 32.768 kHz crystal on page 19 for more information.
// • P0.05, P0.06, P0.07, and P0.08 are used by the UART connected to the interface MCU. See
// Virtual serial port on page 8 for more information.
// • P0.09 and P0.10 are by default used by signals NFC1 and NFC2. See NFC antenna interface on page
// 25 for more information.
// • P0.13–P0.20 are by default connected to the buttons and LEDs. See Buttons and LEDs on page
// 16 for more information.

static const uint8_t pins[NB_PINS] = {
    2, 3, 4, 11, 12, 13, 14, 15

};

static struct gpio_callback gpio_cbs[NB_PINS];
static const struct device *gpio_dev;

// --- Fonctions ---

//Callback d’interruption GPIO
void gpio_pin_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins_triggered)
{
    bool changed = false;

    // Parcourt les broches surveillées
    for (int i = 0; i < NB_PINS; i++)
    {
        // Si l'interruption a été déclenchée pour cette broche
        if (pins_triggered & BIT(pins[i]))
        {
            // Lire la valeur de la broche
            int val = gpio_pin_get(dev, pins[i]);
            char new_state = (val == 0) ? '1' : '0';

            // Si l’état a changé, on met à jour la trame et on note un changement
            if (new_state != prev_trame_data_opto[i])
            {
                printk("Changement sur P0.%d -> %c\n", pins[i], new_state);
                prev_trame_data_opto[i] = new_state;
                trame_data_opto[i] = new_state;
                changed = true;
            }
        }
    }

    // Si un changement est détecté, on construit et envoie une nouvelle trame
    if (changed)
    {
        build_ascii_trame_dyn(CMD3, trame_data_opto, sizeof(trame_data_opto));
    }
}

/**
 * @brief Lecture de l'état des optocoupleurs (pins GPIO)
 *
 * Lit l'état des pins optos configurés en entrée avec pull-up,
 * inverse la logique et stocke '1' si le pin est bas (0),
 * sinon '0'. Ensuite construit et envoie une trame BLE avec les données.
 */
void read_optos_state(void)
{
    for (int i = 0; i < NB_PINS; i++)
    {
        int val = gpio_pin_get(gpio_dev, pins[i]);
        if (val >= 0)
        {
            // Logique inverse : 0 logique devient '1'
            trame_data_opto[i] = (val == 0) ? '1' : '0';
        }
        else
        {
            printk("Erreur lecture GPIO P0.%d\n", pins[i]);
        }
    }
    // Envoi des données optos dans une trame BLE
    build_ascii_trame_dyn(CMD3, (uint8_t *)trame_data_opto, NB_PINS);
}

/**
 * @brief Handler périodique du travail qui lit l'état des optos toutes les 10 secondes
 */
static void optos_read_handler(struct k_work *work)
{
    read_optos_state();
    // Planifie une nouvelle exécution dans TRAME_INTERVAL_OPTOS
    k_work_schedule(&optos_read_work, TRAME_INTERVAL_OPTOS);
}

/**
 * @brief Initialisation des GPIO des optocoupleurs
 *
 * Configure les broches définies dans le tableau `pins` comme entrées avec pull-up interne.
 * Active les interruptions sur front montant et descendant pour chaque broche.
 * Associe la fonction de callback `gpio_pin_callback` à chaque broche configurée.
 *
 * Cette fonction doit être appelée une fois au démarrage pour configurer
 * les GPIO liées aux optocoupleurs.
 */
void init_gpio_optos(void)
{
    int ret;
    gpio_dev = DEVICE_DT_GET(GPIO_NODE);

    if (!device_is_ready(gpio_dev))
    {
        printk("GPIO device non prêt\n");
        return;
    }

    for (int i = 0; i < NB_PINS; i++)
    {
        ret = gpio_pin_configure(gpio_dev, pins[i], GPIO_INPUT | GPIO_PULL_UP);
        if (ret != 0)
        {
            printk("Erreur config P0.%d : %d\n", pins[i], ret);
            return;
        }
        gpio_init_callback(&gpio_cbs[i], gpio_pin_callback, BIT(pins[i]));
        gpio_add_callback(gpio_dev, &gpio_cbs[i]);
        ret = gpio_pin_interrupt_configure(gpio_dev, pins[i], GPIO_INT_EDGE_BOTH);
        if (ret != 0)
        {
            printk("Erreur interruption P0.%d : %d\n", pins[i], ret);
            return;
        }
    }
    printk("GPIO optos configurées\n");
}


/**
 * @brief Callback d'interruption UART pour réception asynchrone.
 *
 * Cette fonction est appelée lors d'une interruption UART déclenchée par la réception de données.
 * Elle lit les octets reçus depuis le FIFO UART, détecte et assemble des trames commençant par '$'.
 * Une trame est considérée complète lorsqu'elle contient 4 séparateurs ';'.
 * Dès réception complète, la trame est traitée via build_ascii_trame_dyn.
 * 
 * @param dev        Pointeur sur la structure device UART.
 * @param user_data  Pointeur utilisateur (non utilisé ici).
 */
void uart_cb(const struct device *dev, void *user_data)
{
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev))
    {
        uint8_t buf[64];
        int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
        if (recv_len <= 0)
            return;

        for (int i = 0; i < recv_len; i++)
        {
            char c = buf[i];
            if (c == '$')
            {
                cpt_semi = 0;
                uart_buf_pos = 0;
                in_trame = true;
                uart_buffer[uart_buf_pos++] = c;
            }
            else if (in_trame)
            {
                if (uart_buf_pos < UART_BUF_SIZE - 1)
                {
                    uart_buffer[uart_buf_pos++] = c;
                    if (c == ';')
                        cpt_semi++;
                    if (cpt_semi == 4)
                    {
                        in_trame = false;
                        build_ascii_trame_dyn(CMD4, uart_buffer, uart_buf_pos);
                        printk("Trame UART reçue (taille: %d)\n", uart_buf_pos);
                        uart_buf_pos = 0;
                    }
                }
                else
                {
                    printk("Buffer UART overflow\n");
                    uart_buf_pos = 0;
                    in_trame = false;
                }
            }
        }
    }
}

/**
 * @brief Initialisation UART0 avec callback et interruption Rx activée
 */
void init_uart0(void)
{
    if (!device_is_ready(uart_dev))
    {
        printk("UART device non prêt\n");
        return;
    }
    uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    uart_irq_rx_enable(uart_dev);
}

/**
 * @brief Handler périodique pour envoyer des informations applicatives via BLE.
 * 
 * Cette fonction est appelée périodiquement par le système de scheduling de Zephyr (k_work).
 * Elle prépare une trame contenant des informations applicatives, ici un nom et une version,
 * et peut envoyer cette trame via BLE (fonction désactivée par défaut dans ce code).
 * 
 * @param work Pointeur vers la structure k_work associée à ce handler.
 */
void app_info_work_handler(struct k_work *work)
{
    const char name[] = "Apli";     // Nom de l'application à envoyer
    const char version[] = "1.0";   // Version de l'application à envoyer

    uint8_t info[100];              // Buffer pour construire la chaîne d'information
    uint8_t len = snprintf((char *)info, sizeof(info), "%s;%s", name, version);

    if (len > 0 && len < sizeof(info))
    {
        // Envoi de la trame en BLE (désactivé par défaut)
        // build_ascii_trame_dyn(CMD2, info, len);
    }
    else
    {
        printk("Erreur format infos appli\n");
    }

    // Replanifie ce handler pour s'exécuter à nouveau après un intervalle défini
    k_work_schedule(&app_info_work, TRAME_INTERVAL_INFOS_APPLI);
}


/**
 * @brief Callback appelé lors du changement de configuration du descripteur CCCD.
 * 
 * Cette fonction est déclenchée lorsque le client BLE active ou désactive
 * les notifications sur une caractéristique via le descripteur Client Characteristic Configuration Descriptor (CCCD).
 * 
 * @param attr Pointeur vers l'attribut GATT (le CCCD) concerné.
 * @param value Nouvelle valeur du CCCD (par exemple BT_GATT_CCC_NOTIFY si notifications activées).
 */
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    // Met à jour l'état local de l'activation des notifications
    notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    
    // Affiche dans le log l'état actuel des notifications
    printk("Notification %s\n", notif_enabled ? "activée" : "désactivée");
}


/**
 * @brief Construit et envoie une trame ASCII formatée dynamiquement via BLE.
 * 
 * La trame a le format suivant :
 *   <cmd;data_size;data;checksum>
 * où cmd et data_size sont en hexadécimal sur 2 caractères,
 * data est une chaîne ASCII, checksum est un octet de contrôle en hexadécimal.
 * 
 * @param cmd Code de commande (1 octet).
 * @param data Pointeur vers les données ASCII à insérer dans la trame.
 * @param data_size Taille des données à insérer.
 */
void build_ascii_trame_dyn(uint8_t cmd, const uint8_t *data, uint8_t data_size)
{
    // Calcul approximatif de la taille nécessaire pour la trame entière,
    // incluant délimiteurs, séparateurs et checksum en ASCII hexadécimal
    size_t trame_len = 1 + 2 + 1 + 2 + 1 + data_size + 1 + 2 + 1 + 1;

    // Allocation dynamique de la trame
    char *trame = k_malloc(trame_len);
    if (!trame)
    {
        printk("Erreur malloc trame\n");
        return; // allocation échouée, on quitte
    }

    // Construction initiale de la trame avec checksum temporaire à 0,
    // nécessaire pour calculer le checksum réel ensuite
    snprintf(trame, trame_len,
             "<%02X;%02X;%.*s;%02X>",
             cmd, data_size, data_size, data, 0);

    // Calcul du checksum réel sur la trame construite (checksum temporaire ignoré)
    uint8_t checksum = calc_checksum_ascii_sum(trame);

    // Reconstruction complète de la trame avec checksum correct
    int len = snprintf(trame, trame_len,
                       "<%02X;%02X;%.*s;%02X>",
                       cmd, data_size, data_size, data, checksum);

    // Vérification que la trame est bien construite sans dépassement
    if (len > 0 && len < trame_len)
    {
        // Envoi de la trame via notification BLE
        send_ble((uint8_t *)trame, len);
    }
    else
    {
        printk("Erreur création trame finale\n");
    }

    // Libération de la mémoire allouée dynamiquement
    k_free(trame);
}

/**
 * @brief Calcul du checksum somme ASCII de la trame,
 *        entre le premier '<' et avant le dernier ';'
 */
uint8_t calc_checksum_ascii_sum(const char *trame)
{
    uint8_t sum = 0;
    const char *start = strchr(trame, START_CHAR);
    const char *last_sep = strrchr(trame, ';');

    if (!start || !last_sep || last_sep <= start)
        return 0;

    for (const char *p = start + 1; p < last_sep; p++)
    {
        sum += (uint8_t)(*p);
    }

    return sum;
}


// <  3  0  ;  0  ;  ;  4  4  > trame name bluetooth

/**
 * @brief Analyse une trame ASCII reçue, vérifie son format, son checksum et exécute la commande.
 * 
 * La trame doit être de la forme : <cmd;size;data;checksum>
 * où cmd est un entier, size la taille des données, data la chaîne de données,
 * et checksum un code hexadécimal de contrôle.
 * 
 * Cette fonction extrait les champs, valide leur contenu et exécute la commande appropriée.
 * 
 * @param input Pointeur vers la chaîne de caractères contenant la trame ASCII.
 */
void parse_ascii_trame_string(const char *input)
{
    // Vérification que l'entrée n'est pas nulle
    if (!input)
    {
        printk("Trame nulle reçue\n");
        return;
    }

    printk(" parse_ascii_trame_string  input :%s \n", input);

    // Recherche des délimiteurs de début et fin de trame
    const char *start = strchr(input, START_CHAR);
    const char *end = strchr(input, END_CHAR);
    if (!start || !end || end <= start)
    {
        printk("Délimiteurs invalides\n");
        return;
    }

    // Calcul de la longueur de la trame
    size_t len = end - start + 1;

    // Validation de la taille minimale et maximale
    if (len < 5 || len > 100)
    {
        printk("Trame longueur invalide : %d\n", len);
        return;
    }

    // Extraction des champs séparés par ';'
    char *fields[4] = {0};
    int field_idx = 0;
    const char *p = input;
    char *next = NULL;

    while (field_idx < 4)
    {
        next = strchr(p, ';');
        if (field_idx < 3)
        {
            if (!next)
                break;
            *next = '\0';             // Terminer la chaîne ici pour isoler le champ
            fields[field_idx++] = (char *)p; // Stocker le champ
            p = next + 1;            // Avancer vers le champ suivant
        }
        else
        {
            fields[field_idx++] = (char *)p; // Le dernier champ (checksum) est jusqu'à la fin
            break;
        }
    }

    // Vérification que 4 champs ont bien été trouvés
    if (field_idx != 4)
    {
        printk("Format incorrect (champs != 4)\n");
        goto parse_error;
    }

    // Conversion des champs en valeurs utiles
    int cmd = atoi(fields[0]);
    int size = atoi(fields[1]);
    char *data = fields[2];
    int checksum_received = (int)strtol(fields[3], NULL, 16);

    // Validation de la taille de la donnée
    if (size < 0 || size > 50)
    {
        printk("Taille invalide\n");
        goto parse_error;
    }

    // Vérification que la longueur réelle de data correspond à size
    if ((int)strlen(data) != size)
    {
        printk("Taille data incorrecte (attendu=%d, reçu=%ld)\n", size, strlen(data));
        goto parse_error;
    }

    printk(" 2 parse_ascii_trame_string  input :%s \n", input);

    // Calcul du checksum de la trame reçue
    uint8_t checksum_calcul = calc_checksum_ascii_sum(input);

    // Comparaison checksum reçu vs calculé
    if (checksum_received != checksum_calcul)
    {
        printk("Checksum invalide (attendu=0x%02X, reçu=0x%02X)\n", checksum_calcul, checksum_received);
        goto parse_error;
    }

    // Trame validée, affichage des informations
    printk("Trame OK: cmd=0x%X (%d), size=%d, data='%s', checksum=0x%02X\n",
           cmd, cmd, size, data, checksum_received);

    // Traitement des commandes selon le code cmd
    switch (cmd)
    {
    case CMD5:
    { // Lecture nom module
        const char *name = bt_get_name();
        if (name)
        {
            printk("CMD5_RX nom BT: %s (len=%d)\n", name, strlen(name));
            build_ascii_trame_dyn(CMD5, name, strlen(name));
        }
        else
        {
            printk("bt_get_name a échoué\n");
        }
    }
    break;

    case CMD7:
    { // Adresse MAC
        bt_addr_le_t addr;
        size_t count = 1;
        bt_id_get(&addr, &count);
        printk("MAC BT: %02X:%02X:%02X:%02X:%02X:%02X\n",
               addr.a.val[5], addr.a.val[4], addr.a.val[3],
               addr.a.val[2], addr.a.val[1], addr.a.val[0]);

        uint8_t mac_buf[6] = {
            addr.a.val[5], addr.a.val[4], addr.a.val[3],
            addr.a.val[2], addr.a.val[1], addr.a.val[0]};

        build_ascii_trame_dyn(CMD7, mac_buf, sizeof(mac_buf));
    }
    break;

    case CMD10:
    { // Validation du nom Bluetooth
        size_t len = strlen(BleNewName);
        if (len > 0 && len <= BLE_NAME_MAX_LEN)
        {
            int err = bt_set_name(BleNewName);
            if (err != 0)
            {
                printk("Erreur bt_set_name : %d\n", err);
            }
            else
            {
                // Redémarrer la publicité BLE avec le nouveau nom
                bt_le_adv_stop();
                bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
            }
        }
    }
    break;

    case CMD11:
    { // Reset (pas implémenté ici)
    }
    break;

    default:
        printk("Commande inconnue: 0x%02X\n", cmd);
        break;
    }

    return;

parse_error:
    // Gestion d'erreur en cas d'échec de parsing (libération mémoire si allouée)
    return;
}

/**
 * @brief Callback appelé lors d'une écriture sur la caractéristique BLE commande.
 * 
 * Affiche les données reçues du client BLE et appelle l'analyseur de trame ASCII.
 * 
 * @param conn Pointeur vers la connexion Bluetooth active.
 * @param attr Pointeur vers l'attribut GATT écrit.
 * @param buf Pointeur vers les données écrites.
 * @param len Longueur des données reçues.
 * @param offset Offset dans l'attribut (non utilisé ici).
 * @param flags Flags d'écriture.
 * 
 * @return Nombre d'octets écrits ou code d'erreur.
 */
static ssize_t write_cmd(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf,
                         uint16_t len,
                         uint16_t offset,
                         uint8_t flags)
{
    // Vérifie la longueur minimale
    if (len < 1)
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

    // Conversion de l'adresse Bluetooth de la connexion en chaîne
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Commande reçue de %s \n", addr);

    // Affiche chaque caractère reçu
    for (int i = 0; i < len; i++)
    {
        printk(" %c ", ((uint8_t *)buf)[i]);
    }

    // Termine la chaîne de caractères pour assurer une fin correcte
    ((uint8_t *)buf)[len] = '\0';
    printk("\n");

    // Analyse la trame ASCII reçue
    parse_ascii_trame_string((const char *)buf);

    return len;
}

/**
 * @brief Lecture de la version BLE.
 * 
 * Fonction appelée lors d'une lecture de la caractéristique version BLE.
 * 
 * @param conn Pointeur vers la connexion Bluetooth.
 * @param attr Pointeur vers l'attribut GATT.
 * @param buf Buffer pour stocker la valeur lue.
 * @param len Longueur maximale à lire.
 * @param offset Offset dans la valeur.
 * @return Taille de données copiées ou erreur.
 */
static ssize_t ble_version_read(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf,
                                uint16_t len,
                                uint16_t offset)
{
    const char *version = BLE_VERSION_STRING; // Chaîne constante version BLE
    return bt_gatt_attr_read(conn, attr, buf, len, offset, version, strlen(version));
}

/**
 * @brief Lecture de la version du firmware.
 * 
 * Fonction appelée lors d'une lecture de la caractéristique version firmware.
 * 
 * @param conn Pointeur vers la connexion Bluetooth.
 * @param attr Pointeur vers l'attribut GATT.
 * @param buf Buffer pour stocker la valeur lue.
 * @param len Longueur maximale à lire.
 * @param offset Offset dans la valeur.
 * @return Taille de données copiées ou erreur.
 */
static ssize_t firmware_version_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf,
                                     uint16_t len,
                                     uint16_t offset)
{
    const char *fw = FIRMWARE_VERSION_STRING; // Chaîne constante version firmware
    return bt_gatt_attr_read(conn, attr, buf, len, offset, fw, strlen(fw));
}


/**
 * @brief Définition du service Device Information BLE.
 * 
 * Ce service contient les caractéristiques Firmware Revision et Model Number,
 * toutes deux en lecture uniquement.
 */
BT_GATT_SERVICE_DEFINE(device_info_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),

                       BT_GATT_CHARACTERISTIC(BT_UUID_FIRMWARE_REVISION_STRING,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              firmware_version_read, NULL, NULL),
                       BT_GATT_CHARACTERISTIC(BT_UUID_MODEL_NUMBER_STRING,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              ble_version_read, NULL, NULL));


/**
 * @brief Définition du service personnalisé trame BLE.
 * 
 * Ce service expose :
 * - Une caractéristique notify pour envoyer des trames au client.
 * - Un descripteur CCC pour gérer l'activation/désactivation des notifications.
 * - Une caractéristique write pour recevoir les commandes du client.
 */
BT_GATT_SERVICE_DEFINE(trame_svc,
                       BT_GATT_PRIMARY_SERVICE(&ble_service_uuid.uuid),

                       BT_GATT_CHARACTERISTIC(&ble_sent_uuid.uuid,
                                              BT_GATT_CHRC_NOTIFY,
                                              BT_GATT_PERM_READ,
                                              NULL, NULL, NULL),

                       BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

                       BT_GATT_CHARACTERISTIC(&ble_data_uuid.uuid,
                                              BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE,
                                              NULL, write_cmd, NULL), );

/**
 * @brief Envoie une notification BLE contenant une trame.
 * 
 * Vérifie que les notifications sont activées et qu'une connexion est active.
 * 
 * @param ptrame Pointeur vers la trame à envoyer.
 * @param len_trame Longueur de la trame.
 */
void send_ble(uint8_t *ptrame, int len_trame)
{
    if (!notif_enabled) // Notification non activée, on ne fait rien
        return;

    if (current_conn) // Si une connexion est active
    {
        int err = bt_gatt_notify(current_conn, &trame_svc.attrs[1], ptrame, len_trame);
        if (err)
        {
            printk("Erreur notification BLE : %d\n", err);
        }
        else
        {
            // Notification envoyée avec succès (aucune action supplémentaire ici)
        }
    }
}


/**
 * @brief Callback appelé lors d'une connexion BLE établie.
 * 
 * Stocke la connexion courante et affiche l'adresse du client.
 * 
 * @param conn Pointeur vers la connexion.
 * @param err Code d'erreur (0 si succès).
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err == 0)
    {
        printk("Connecté\n");
        current_conn = conn; // Sauvegarde la connexion active
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        printk("Device connected: %s\n", addr);
    }
}

/**
 * @brief Callback appelé lors d'une déconnexion BLE.
 * 
 * Affiche la raison de la déconnexion et remet la connexion courante à NULL.
 * 
 * @param conn Pointeur vers la connexion.
 * @param reason Code de la raison de déconnexion.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Déconnecté (raison %d)\n", reason);
    current_conn = NULL;
}

/**
 * @brief Enregistre les callbacks de connexion/déconnexion BLE.
 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* === main === */

/**
 * @brief Fonction principale d'initialisation et boucle d'exécution.
 * 
 * Initialise le Bluetooth, les GPIO, UART, démarre la publicité BLE,
 * planifie les tâches périodiques et attend indéfiniment.
 * 
 * @return 0 en cas de succès, sinon code d'erreur.
 */
int main(void)
{
    int err;

    printk("Démarrage BLE\n");

    init_gpio_optos();  // Initialisation GPIO optocoupleurs

    init_uart0();       // Initialisation UART0

    err = bt_enable(NULL); // Activation Bluetooth

    if (err)
    {
        printk("Bluetooth init échouée (%d)\n", err);
        return 0;
    }

    // Démarrage publicité BLE avec nom du device visible
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err)
    {
        printk("Échec publicité BLE (%d)\n", err);
        return 0;
    }

    printk("Publicité démarrée\n");

    // Planifie la lecture périodique des optocoupleurs (toutes les 5s)
    k_work_schedule(&optos_read_work, TRAME_INTERVAL_OPTOS);

    // Planifie la remontée d'infos applicatives périodique
    k_work_schedule(&app_info_work, TRAME_INTERVAL_INFOS_APPLI);

    // Boucle infinie d'attente, le kernel gère le multitâche
    while (1)
    {
        k_sleep(K_FOREVER);
    }
}
