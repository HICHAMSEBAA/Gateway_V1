/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// lib
#include <stdio.h>
#include <inttypes.h>
#include "nvs_flash.h"

// sdkconfig
#include "sdkconfig.h"

// esp lib
#include "esp_log.h"
#include <esp_system.h>
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

// freertos
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// mqtt
#include "mqtt_client.h"
#include "esp_wifi.h"

// nrf
#include "NRF.h"

// Variables to store sensor data from the Slave sensor
uint8_t RH1, RH2, TC1, TC2;

// Variables to store calculated temperature and humidity values
float tCelsius, tFahrenheit, RH;

// Variable to store voltage
float V;

// Variables to store RGB data
int R, G, B;

// Mode definitions for transmission (TX) and reception (RX)
#define TX_MODE 1
#define RX_MODE 0

// Number of slave devices in the network
#define NUMBER_OF_SLAVE 2

// Timing parameters
#define DELAY 1000  // Delay in milliseconds
#define DELAY_US 15 // Delay in microseconds

// Data rate for RF communication
#define DATA_RATE RF24_250KBPS // Options: RF24_250KBPS, RF24_1MBPS, RF24_2MBPS

// Transmission power level
#define POWER_DB RF24_PA_0dB // Options: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX

// Define the transmission pipe addresses for the slave devices
uint64_t SlaveAddrs[NUMBER_OF_SLAVE] = {0x11223344AA, 0x112233447A};

// Define an array to store transmission data with an initial value "SEND"
char myTxData[32] = "SEND To My Data";

// selves command
char selvesCommans[32];
uint8_t Node_num_commmand = NUMBER_OF_SLAVE + 1;

// command
#define DATA_REQUES "SEND To My Data"
#define LED_ON "LED_ON"
#define LED_OFF "LED_OFF"
#define POWER_1 "P_1"
#define POWER_2 "P_2"
#define POWER_3 "P_3"

// Define an array to store received data
uint8_t myRxData[32] = "None";

// Define an acknowledgment payload with an initial value "Ack by ESP32S3 !!"
char myAckPayload[32] = "Ack by ESP32S3 !!";

// Counter for the total number of requests made
int TotalRequest = 0;

// Counter for the number of data packets received
int DataCount = 0;

// Counter for the number of times the NRF module is reset
int NrfReset = 0;

// Counter for the waiting time
int waitCount = 0;

// Flag used to indicate errors in communication
int C_ERROR_COUNT = 0;
bool errorFlag = true;

// -------------------------------------------------------------------------------------------- //

static const char *TAG = "MQTT";

// WIFI
#define EXAMPLE_ESP_WIFI_SSID "Redmi 8"
#define EXAMPLE_ESP_WIFI_PASS "20002023"
#define MAX_RETRY 10
static int retry_cnt = 0;

// MQTT
uint32_t MQTT_CONNECTED = 0;
uint32_t MQTT_COMMAND = 0;
char commands_topic[50];
char commands_data[50];
esp_mqtt_client_handle_t client = NULL;
#define BROKER_URI "mqtt://192.168.121.178:1883"
#define MQTT_PUB_GATWAY "esp32/gatway"
#define MQTT_SUB_GATWAY "esp32/gatwaycommand"
#define MQTT_PUB_NODE1 "esp32/node1"
#define MQTT_PUB_NODE2 "esp32/node2"
#define MQTT_SUB_NODE1_LED "esp32/node1/led"
#define MQTT_SUB_NODE1_P "esp32/node1/power"
#define MQTT_SUB_NODE2_LED "esp32/node2/led"
#define MQTT_SUB_NODE2_P "esp32/node2/power"
const char *topics[NUMBER_OF_SLAVE] = {MQTT_PUB_NODE1, MQTT_PUB_NODE2};

// Characteristic
char ESP32_Characteristic[100];
char NODE1_Characteristic[100];
char NODE2_Characteristic[100];

// mqttpub
char PUB_DATA_NODES[50];
char PUB_DATA_GATWAY[10];
uint8_t TOPIC_;

// FreeRTOS
TaskHandle_t myTask1Handle = NULL;
TaskHandle_t myTask2Handle = NULL;
TaskHandle_t myTask3Handle = NULL;
TaskHandle_t myTask4Handle = NULL;
QueueHandle_t DATA_NODES;
QueueHandle_t DATA_GATWAY;
SemaphoreHandle_t MQTT_PUB_TREGER_NODES = NULL;

// -------------this part of the code handles the MQTT protocol------------- //
/**
 * @brief Logs an error message if the provided error code is nonzero.
 *
 * This function checks the provided error code and logs an error message
 * using the ESP_LOGE macro if the error code is nonzero.
 *
 * @param message A string message describing the context of the error.
 * @param error_code The error code to check and log if nonzero.
 */
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        // Log the error message with the error code in hexadecimal format
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/**
 * @brief Handles MQTT client events.
 *
 * This function is the event handler for MQTT client events. It logs the event details
 * and performs specific actions based on the event type.
 *
 * @param handler_args Pointer to the handler arguments (not used).
 * @param base The event base.
 * @param event_id The event ID.
 * @param event_data Pointer to the event data structure.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    // Log the dispatched event details
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);

    // Cast the event data to the appropriate type
    esp_mqtt_event_handle_t event = event_data;

    // Process the event based on the event ID
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        // MQTT client is connected
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, MQTT_SUB_NODE1_LED, 0);
        esp_mqtt_client_subscribe(client, MQTT_SUB_NODE1_P, 0);
        esp_mqtt_client_subscribe(client, MQTT_SUB_NODE2_LED, 0);
        esp_mqtt_client_subscribe(client, MQTT_SUB_NODE2_P, 0);
        esp_mqtt_client_subscribe(client, MQTT_SUB_GATWAY, 0);
        sprintf(ESP32_Characteristic, "3|ESP S3|ARM Cortex M4 32-bit|240 MHz|512 KB|384 KB|%d|250 Kbps|0 db|52|%d|2 Byte", NUMBER_OF_SLAVE, DELAY);
        sprintf(NODE1_Characteristic, "3|STM32F446RE|ARM Cortex M4 32-bit|180 MHz|512 KB|128 KB|3|100|Low");
        sprintf(NODE2_Characteristic, "3|STM32F446RE|ARM Cortex M4 32-bit|180 MHz|512 KB|128 KB|3|100|Low");
        esp_mqtt_client_publish(client, MQTT_PUB_GATWAY, &(ESP32_Characteristic), 0, 0, 0);
        esp_mqtt_client_publish(client, MQTT_PUB_NODE1, &(NODE1_Characteristic), 0, 0, 0);
        esp_mqtt_client_publish(client, MQTT_PUB_NODE2, &(NODE2_Characteristic), 0, 0, 0);
        MQTT_CONNECTED = 1; // Set MQTT connected flag
        break;

    case MQTT_EVENT_DISCONNECTED:
        // MQTT client is disconnected
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        MQTT_CONNECTED = 0; // Clear MQTT connected flag
        break;

    case MQTT_EVENT_ERROR:
        // MQTT error event
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        // Check if the error type is related to TCP transport
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            // Log error details related to TCP transport
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;

    case MQTT_EVENT_DATA:
        // MQTT data event
        sprintf(commands_topic, "%.*s", event->topic_len, event->topic);
        sprintf(commands_data, "%.*s", event->data_len, event->data);
        MQTT_COMMAND = 1;
        // Add handling for MQTT data event if needed
        break;

    default:
        // Other MQTT events
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        // Add handling for other MQTT events if needed
        break;
    }
}

/**
 * @brief Starts the MQTT client application.
 *
 * This function initializes the MQTT client with the provided configuration, registers
 * the event handler for MQTT client events, and starts the MQTT client.
 */
static void mqtt_app_start(void)
{
    // Log the start of the MQTT application
    ESP_LOGI(TAG, "STARTING MQTT");

    // Initialize MQTT client configuration with the broker URI
    esp_mqtt_client_config_t mqttConfig = {
        .broker.address.uri = BROKER_URI // Set the URI of the MQTT broker (e.g., IP address or domain name)
    };

    // Initialize the MQTT client with the provided configuration
    client = esp_mqtt_client_init(&mqttConfig);

    // Register the event handler for MQTT client events
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);

    // Start the MQTT client
    esp_mqtt_client_start(client);
}

// -------------this patr of code handles the WIFI connection------------- //
/**
 * @brief WiFi event handler function.
 *
 * This function handles WiFi events such as station start, connection, disconnection,
 * and obtaining an IP address. Based on the event type, it performs appropriate actions
 * such as connecting to WiFi, starting the MQTT client, retrying WiFi connection, etc.
 *
 * @param arg Pointer to the handler arguments (not used).
 * @param event_base The event base.
 * @param event_id The event ID.
 * @param event_data Pointer to the event data structure.
 * @return esp_err_t indicating success or failure of the event handling.
 */
static esp_err_t wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        // Start station mode and connect to WiFi
        esp_wifi_connect();
        ESP_LOGI(TAG, "Trying to connect with Wi-Fi\n");
        break;

    case WIFI_EVENT_STA_CONNECTED:
        // WiFi connected
        ESP_LOGI(TAG, "Wi-Fi connected\n");
        break;

    case IP_EVENT_STA_GOT_IP:
        // Obtained IP address, start MQTT client
        ESP_LOGI(TAG, "got ip: starting MQTT Client\n");
        mqtt_app_start();
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        // WiFi disconnected, retry connection if retries left
        ESP_LOGI(TAG, "disconnected: Retrying Wi-Fi\n");
        if (retry_cnt++ < MAX_RETRY)
        {
            esp_wifi_connect();
        }
        else
        {
            ESP_LOGI(TAG, "Max Retry Failed: Wi-Fi Connection\n");
        }
        break;

    default:
        break;
    }

    return ESP_OK;
}

/**
 * @brief Initializes WiFi configuration and event handling.
 *
 * This function initializes WiFi configuration and sets up event handling for WiFi events
 * such as station start, connection, disconnection, and obtaining an IP address.
 * It registers event handlers for WiFi and IP events, sets up WiFi station mode with the
 * provided SSID and password, and starts WiFi.
 */
void wifi_init(void)
{
    // Create default event loop
    esp_event_loop_create_default();

    // Register event handler for WiFi events
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);

    // Register event handler for IP events
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    // Initialize WiFi configuration
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,           // Set WiFi SSID
            .password = EXAMPLE_ESP_WIFI_PASS,       // Set WiFi password
            .threshold.authmode = WIFI_AUTH_WPA2_PSK // Set authentication mode
        },
    };

    // Initialize TCP/IP stack
    esp_netif_init();
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Set WiFi mode to station mode
    esp_wifi_set_mode(WIFI_MODE_STA);

    // Set WiFi configuration
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);

    // Start WiFi
    esp_wifi_start();
}

// -------------this part of code handles the NRF connection------------- //
/**
 * @brief Configures the NRF24 module for either transmit or receive mode.
 *
 * This function sets up the NRF24 module to operate in either transmit or receive mode
 * based on the provided transmit_mode flag. It also applies common settings such as
 * enabling automatic acknowledgment, dynamic payloads, and acknowledgment payloads.
 *
 * @param transmit_mode A boolean flag indicating the mode to configure.
 *                      - true: Transmit mode
 *                      - false: Receive mode
 * @param num The index of the slave address to use for communication.
 *            This index refers to the SlaveAddrs array which holds the addresses of the slave devices.
 */
void nrf24_config_mode(bool transmit_mode, uint8_t num)
{
    if (transmit_mode)
    {
        // Configure the NRF24 module for transmit mode
        NRF24_stopListening();                  // Stop listening for incoming data
        NRF24_openWritingPipe(SlaveAddrs[num]); // Open the writing pipe to the specified slave address
    }
    else
    {
        // Configure the NRF24 module for receive mode
        NRF24_startListening();                    // Start listening for incoming data
        NRF24_openReadingPipe(1, SlaveAddrs[num]); // Open the reading pipe to the specified slave address
    }

    // Common settings for both transmit and receive modes
    NRF24_set_AutoAck(true);       // Enable automatic acknowledgment
    NRF24_enableDynamicPayloads(); // Enable dynamic payloads for variable length messages
    NRF24_enable_AckPayload();     // Enable acknowledgment payloads
}

/**
 * @brief Attempts to send a message to a specified slave device.
 *
 * This function switches the NRF24 module to transmit mode, then attempts to send a message
 * to the specified slave device multiple times (up to a defined limit). It waits for an
 * acknowledgment payload from the receiver to confirm successful message delivery.
 *
 * @param num The index of the slave address to use for communication.
 *            This index refers to the SlaveAddrs array which holds the addresses of the slave devices.
 * @return true if the message was successfully delivered and acknowledged; false otherwise.
 */
bool ShootTheMessage(int num, char *myTxData)
{
    // Switch NRF24 back to transmit mode
    nrf24_config_mode(TX_MODE, num);

    // Flag to track message delivery success
    bool messageDelivered = false;

    // Number of attempts to send the message
    int stop = 5;

    // Buffer to store the acknowledgment payload
    char AckPayload[32];

    for (int i = 0; i < stop; i++)
    {
        // Load the message into the NRF24's transmit buffer
        if (NRF24_write(myTxData, 32))
        {
            // Message sent successfully, now wait for acknowledgment
            NRF24_read(AckPayload, 32);
            if (strlen(AckPayload) != 0)
            {
                // Acknowledgment received successfully
                messageDelivered = true;
                break;
            }

            // Short delay before retrying
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    return messageDelivered;
}

/**
 * Analyzes the received payload, extracting data, performing calculations,
 * and potentially displaying results.
 *
 * @param rxPayload Pointer to the received payload data.
 * @param length Length of the received payload (should be 32).
 */
bool ReceiveAndAcknowledgeData(void)
{
    // Flag to track data reception success
    bool dataReceived = NRF24_available();

    // Check if data is available on the NRF24 module
    if (dataReceived)
    {
        // Read the received data into the buffer
        NRF24_read(myRxData, 32);

        // Immediately send an acknowledgement payload on pipe 1
        NRF24_write_AckPayload(1, myAckPayload, 32);

        // Increment received data counter
        DataCount++;
    }

    return dataReceived;
}

/**
 * @brief Analyzes the received payload data and extracts sensor values.
 *
 * This function checks the received payload for errors, extracts sensor values such as
 * temperature, humidity, voltage, and RGB values, performs necessary calculations, and
 * prints the extracted and calculated data.
 *
 * @param rxPayload Pointer to the received payload data.
 * @param length The length of the received payload data.
 */
void AnlyseThePayload(uint8_t *rxPayload, uint8_t length)
{
    // Check for error flag in the payload and ensure length is within bounds
    if (rxPayload[0] != 6 || length > 32)
    {
        // Error in the payload data
        // printf("Error in the Payload Data.\n\r");
        return; // Exit early if error detected
    }

    // Extract data from the received payload
    RH1 = rxPayload[1];              // First 8 bits of relative humidity
    RH2 = rxPayload[2];              // Second 8 bits of relative humidity
    TC1 = rxPayload[3];              // First 8 bits of temperature in Celsius
    TC2 = rxPayload[4];              // Second 8 bits of temperature in Celsius
    V = rxPayload[5] * (3.3 / 4096); // Voltage
    R = rxPayload[6];                // Red value of RGB
    G = rxPayload[7];                // Green value of RGB
    B = rxPayload[8];                // Blue value of RGB

    // Calculate temperature in Celsius
    if (TC1 > 127) // Check if the temperature is negative
    {
        tCelsius = (float)TC2 / 10 * (-1);
    }
    else
    {
        tCelsius = (float)((TC1 << 8) | TC2) / 10;
    }

    // Convert temperature to Fahrenheit
    tFahrenheit = tCelsius * 9 / 5 + 32;

    // Calculate relative humidity
    RH = (float)((RH1 << 8) | RH2) / 10;

    // Format
    sprintf(PUB_DATA_NODES, "1|%.2f|%.2f|%.2f|%.2f|%d|%d|%d|", tCelsius, tFahrenheit, RH, V, R, G, B);

    // Print the extracted and calculated data
    // printf("1|%d|%.2f|%.2f|%.2f|%.2f|%d|%d|%d|\n\r", num, tCelsius, tFahrenheit, RH, V, R, G, B);
}

/**
 * @brief Checks the error flag and resets the NRF24 module if necessary.
 *
 * This function increments the cumulative error count if the error flag is set.
 * If the cumulative error count reaches a predefined limit, it resets the NRF24 module
 * and waits for a short delay before continuing. If no error is detected, the error
 * counter is reset.
 *
 * @param errorFlag A boolean flag indicating if an error has occurred.
 */
void AnnounceAndRestartIfNecessary(bool errorFlag)
{
    // Check if there is an error
    if (errorFlag)
    {
        // Increment cumulative error count
        C_ERROR_COUNT++; // Assuming C_ERROR_COUNT is defined elsewhere

        // Reset the error flag for the next wait cycle
        errorFlag = false; // Assuming typo here, reset to false

        // Check if the cumulative error count has reached the threshold
        if (C_ERROR_COUNT >= 3)
        {
            // Announce the NRF24 reset and reconnect process
            NRF24_REST(DATA_RATE, POWER_DB, DELAY_US); // Reset the NRF24 module with defined parameters

            // Increment the reset counter
            NrfReset++; // Assuming NrfReset is defined elsewhere

            // Wait for a short delay before continuing
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    else
    {
        // Reset the cumulative error count if no error is detected
        C_ERROR_COUNT = 0;
    }
}

/**
 * @brief Sets up the NRF24 radio module for communication.
 *
 * This function initializes the NRF24 radio module with specified parameters,
 * configures it for transmit mode with acknowledgment, and prints the current
 * radio settings along with the delay value.
 *
 * @param num The index of the slave address to use for communication.
 *            This index refers to the SlaveAddrs array which holds the addresses of the slave devices.
 */
void setup(int num)
{
    // Initialize the NRF24 radio module with specified data rate, power level, and delay
    NRF24_Init(DATA_RATE, POWER_DB, DELAY_US);

    // // Announce the start of communication setup (optional)
    // printf("________________________Engaging communication channels...________________________ \n\r");

    // Configure NRF24 for transmit mode with acknowledgment
    nrf24_config_mode(TX_MODE, num);

    // // Print the current radio settings (optional)
    // printRadioSettings();

    // // Print the current delay value
    // printf("Delay : %d \n\r", DELAY);
}

/**
 * @brief Main communication loop for transmitting messages and receiving acknowledgments.
 *
 * This function attempts to transmit a message to a specified slave device, switches to receive mode
 * to wait for an acknowledgment, and processes the received data. If the acknowledgment is not received
 * within a set number of attempts, it sets an error flag. The function also prints the current data count
 * and total request count, and handles error conditions by potentially restarting the NRF24 module.
 *
 * @param num The index of the slave address to use for communication.
 *            This index refers to the SlaveAddrs array which holds the addresses of the slave devices.
 */
void loop(uint8_t num, char *myTxData)
{
    TOPIC_ = num;
    // Attempt to transmit a message
    if (ShootTheMessage(num, myTxData))
    {
        // Switch NRF24 to receive mode and listen for acknowledgment
        nrf24_config_mode(RX_MODE, num);

        // Loop for a limited time waiting for acknowledgment
        for (int i = 0; i < 5; i++)
        {
            // Check if data is received successfully
            if (ReceiveAndAcknowledgeData())
            {
                // Analyze and display data
                AnlyseThePayload(myRxData, 32);

                // Reset connection error flag
                errorFlag = false;

                // Exit the inner loop (acknowledgment received)
                break;
            }
            else
            {
                // Short delay before retrying
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            // Reached the wait limit without acknowledgment
            if (i == 4)
            {
                errorFlag = true;
            }
        }
    }
    else
    {
        // Set connection error flag
        errorFlag = true;
    }

    // the current data count and total request count
    sprintf(PUB_DATA_GATWAY, "2|%d|%d", DataCount, TotalRequest);

    // Print the current data count and total request count
    printf("2|%d|%d\n\r", DataCount, TotalRequest);

    // Announce and restart the NRF24 module if necessary
    AnnounceAndRestartIfNecessary(errorFlag);

    // // Delay before the next iteration of the loop
    // vTaskDelay(DELAY / portTICK_PERIOD_MS);
}

/**
 * @brief NodeComm - Function to handle communication with nodes.
 * 
 * This function iterates over a predefined number of slave nodes and sends commands
 * to each one. If the current node matches the node specified for a command, it sends
 * a specific command. Otherwise, it sends a general command. After sending the command,
 * it queues the data for further processing and delays for a specified period before
 * moving to the next node.
 * 
 * Globals:
 * - NUMBER_OF_SLAVE: Total number of slave nodes to communicate with.
 * - Node_num_commmand: The index of the node that should receive a specific command.
 * - selvesCommans: Command data for the specified node.
 * - myTxData: General command data for other nodes.
 * - TotalRequest: Counter for the total number of requests made.
 * - DATA_GATWAY: Queue handle for gateway data.
 * - PUB_DATA_GATWAY: Data to be published to the gateway queue.
 * - DATA_NODES: Queue handle for node data.
 * - PUB_DATA_NODES: Data to be published to the node queue.
 */
void NodeComm(void)
{
    for (size_t i = 0; i < NUMBER_OF_SLAVE; i++) {
        if (i == Node_num_commmand) {
            // Send specific command to the targeted node
            loop(i, selvesCommans);
            printf("slave %d : %s\n", i, selvesCommans);
            // Reset the command node number to avoid repeated commands
            Node_num_commmand = NUMBER_OF_SLAVE + 1;
        } else {
            // Send general command to all other nodes
            loop(i, myTxData);
            printf("slave %d : %s\n", i, myTxData);
        }
        // Increment the total number of requests
        TotalRequest++;
        // Queue data for the gateway and nodes
        xQueueSend(DATA_GATWAY, (void *)PUB_DATA_GATWAY, (TickType_t)0);
        xQueueSend(DATA_NODES, (void *)PUB_DATA_NODES, (TickType_t)0);
        // Delay to allow for task scheduling
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// -------------this patr of code for Tasks------------- //
/**
 * @brief vTaskHandleNodeComm - FreeRTOS task to handle node communication.
 * 
 * This task runs indefinitely, checking for MQTT connectivity and, if connected, 
 * performs node communication by calling the NodeComm function. After communication,
 * it triggers a semaphore to indicate that nodes' data is ready for MQTT publishing.
 * 
 * @param pvParameters - Pointer to the parameters passed to the task (not used in this implementation).
 * 
 * Globals:
 * - MQTT_CONNECTED: Flag indicating the MQTT connection status.
 * - MQTT_PUB_TREGER_NODES: Semaphore handle to trigger MQTT publishing for nodes.
 */
void vTaskHandleNodeComm(void *pvParameters)
{
    // Loop indefinitely for communication
    while (1) {
        if (MQTT_CONNECTED) {
            // Perform node communication
            NodeComm();
            // Give semaphore to trigger MQTT publishing for nodes
            xSemaphoreGive(MQTT_PUB_TREGER_NODES);
        }
    }
}


/**
 * @brief vTaskPublisherData - FreeRTOS task to publish data via MQTT.
 * 
 * This task runs indefinitely, checking for MQTT connectivity and publishing data
 * received from queues to MQTT topics. It first handles data from the gateway queue,
 * and then waits for a semaphore to trigger the publishing of nodes' data.
 * 
 * @param pvParameters - Pointer to the parameters passed to the task (not used in this implementation).
 * 
 * Globals:
 * - MQTT_CONNECTED: Flag indicating the MQTT connection status.
 * - DATA_GATWAY: Queue handle for gateway data.
 * - MQTT_PUB_GATWAY: MQTT topic for publishing gateway data.
 * - MQTT_PUB_TREGER_NODES: Semaphore handle to trigger MQTT publishing for nodes.
 * - NUMBER_OF_SLAVE: Total number of slave nodes.
 * - DATA_NODES: Queue handle for node data.
 * - topics: Array of MQTT topics for each node.
 * - client: MQTT client handle.
 */
void vTaskPublisherData(void *pvParameters)
{
    // Buffer to hold received data
    char rxbuff[50];

    // Loop indefinitely for publishing data
    while (1) {
        if (MQTT_CONNECTED) {
            // Check and publish data from the gateway queue
            if (xQueueReceive(DATA_GATWAY, &(rxbuff), (TickType_t)5)) {
                esp_mqtt_client_publish(client, MQTT_PUB_GATWAY, &(rxbuff), 0, 0, 0);
                // Uncomment for debugging
                // printf("GATWAY rxbuff : %s Time : %lld\n", rxbuff, esp_timer_get_time() / 1000);
            }

            // Wait for the semaphore to trigger publishing nodes' data
            if (xSemaphoreTake(MQTT_PUB_TREGER_NODES, portMAX_DELAY) == pdTRUE) {
                for (size_t i = 0; i < NUMBER_OF_SLAVE; i++) {
                    // Check and publish data from the node queue
                    if (xQueueReceive(DATA_NODES, &(rxbuff), (TickType_t)5)) {
                        esp_mqtt_client_publish(client, topics[i], &(rxbuff), 0, 0, 0);

                    }
                }
            }
        }
    }
}

/**
 * @brief vTaskSubscriberCommand - FreeRTOS task to handle incoming MQTT commands.
 * 
 * This task runs indefinitely, checking for MQTT connectivity and command availability.
 * When a command is received, it processes the command based on the topic and updates
 * the corresponding node's command. It handles various commands for different nodes
 * including LED and power commands.
 * 
 * @param pvParameters - Pointer to the parameters passed to the task (not used in this implementation).
 * 
 * Globals:
 * - MQTT_CONNECTED: Flag indicating the MQTT connection status.
 * - MQTT_COMMAND: Flag indicating if a new MQTT command is available.
 * - commands_topic: Topic of the received MQTT command.
 * - commands_data: Data of the received MQTT command.
 * - MQTT_SUB_GATWAY: MQTT topic for gateway subscription.
 * - MQTT_SUB_NODE1_LED: MQTT topic for Node 1 LED subscription.
 * - MQTT_SUB_NODE1_P: MQTT topic for Node 1 power subscription.
 * - MQTT_SUB_NODE2_LED: MQTT topic for Node 2 LED subscription.
 * - MQTT_SUB_NODE2_P: MQTT topic for Node 2 power subscription.
 * - Node_num_commmand: Index of the node to receive the command.
 * - selvesCommans: Command data for the specified node.
 * - LED_ON: Command to turn the LED on.
 * - LED_OFF: Command to turn the LED off.
 * - POWER_1: Command to set power level 1.
 * - POWER_2: Command to set power level 2.
 * - POWER_3: Command to set power level 3.
 */
void vTaskSubscriberCommand(void *pvParameters)
{
    // Loop indefinitely to process incoming commands
    while (1) {
        if (MQTT_CONNECTED && MQTT_COMMAND) {
            // Uncomment for debugging
            // printf("Topic : %s\n", commands_topic);
            // printf("Data  : %s\n", commands_data);

            if (strcmp(commands_topic, MQTT_SUB_GATWAY) == 0) {
                printf("MQTT_SUB_GATWAY \n");
            } else if (strcmp(commands_topic, MQTT_SUB_NODE1_LED) == 0) {
                Node_num_commmand = 0;
                if (strcmp(commands_data, "1") == 0) {
                    sprintf(selvesCommans, LED_ON);
                    // Uncomment for debugging
                    // printf("%s Node 1\n", selvesCommans);
                } else {
                    sprintf(selvesCommans, LED_OFF);
                    // Uncomment for debugging
                    // printf("%s Node 1\n", selvesCommans);
                }
            } else if (strcmp(commands_topic, MQTT_SUB_NODE1_P) == 0) {
                Node_num_commmand = 0;
                if (strcmp(commands_data, "0") == 0) {
                    sprintf(selvesCommans, POWER_1);
                    // Uncomment for debugging
                    // printf("%s Node 1\n", selvesCommans);
                } else if (strcmp(commands_data, "1") == 0) {
                    sprintf(selvesCommans, POWER_2);
                    // Uncomment for debugging
                    // printf("%s Node 1\n", selvesCommans);
                } else {
                    sprintf(selvesCommans, POWER_3);
                    // Uncomment for debugging
                    // printf("%s Node 1\n", selvesCommans);
                }
            } else if (strcmp(commands_topic, MQTT_SUB_NODE2_LED) == 0) {
                Node_num_commmand = 1;
                if (strcmp(commands_data, "1") == 0) {
                    sprintf(selvesCommans, LED_ON);
                    // Uncomment for debugging
                    // printf("%s Node 2\n", selvesCommans);
                } else {
                    sprintf(selvesCommans, LED_OFF);
                    // Uncomment for debugging
                    // printf("%s Node 2\n", selvesCommans);
                }
            } else if (strcmp(commands_topic, MQTT_SUB_NODE2_P) == 0) {
                Node_num_commmand = 1;
                if (strcmp(commands_data, "0") == 0) {
                    sprintf(selvesCommans, POWER_1);
                    // Uncomment for debugging
                    // printf("%s Node 2\n", selvesCommans);
                } else if (strcmp(commands_data, "1") == 0) {
                    sprintf(selvesCommans, POWER_2);
                    // Uncomment for debugging
                    // printf("%s Node 2\n", selvesCommans);
                } else {
                    sprintf(selvesCommans, POWER_3);
                    // Uncomment for debugging
                    // printf("%s Node 2\n", selvesCommans);
                }
            } else {
                return;
            }
        }
        // Reset the command flag
        MQTT_COMMAND = 0;
        // Delay to allow for task scheduling
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


/**
 * @brief main - Main application entry point.
 * 
 * This function initializes the NVS and WiFi, creates the necessary queues and semaphore,
 * and starts the FreeRTOS tasks for handling node communication, publishing data, and 
 * subscribing to commands.
 * 
 * Globals:
 * - DATA_NODES: Queue handle for node data.
 * - DATA_GATWAY: Queue handle for gateway data.
 * - MQTT_PUB_TREGER_NODES: Semaphore handle to trigger MQTT publishing for nodes.
 * 
 * Functions:
 * - nvs_flash_init: Initializes the NVS flash.
 * - wifi_init: Initializes WiFi.
 * - setup: Additional setup for the application.
 * - xQueueCreate: Creates a queue.
 * - xSemaphoreCreateBinary: Creates a binary semaphore.
 * - xTaskCreate: Creates a FreeRTOS task.
 */

void app_main(void)
{
    // Initialize NVS flash storage
    nvs_flash_init();

    // Initialize WiFi
    wifi_init();

    // Create queue for node data with space for NUMBER_OF_SLAVE items, each holding a 50-char array
    DATA_NODES = xQueueCreate(NUMBER_OF_SLAVE, sizeof(char) * 50);

    // Create queue for gateway data with space for 1 item, holding a 10-char array
    DATA_GATWAY = xQueueCreate(1, sizeof(char) * 10);

    // Check if queue creation was successful
    if (DATA_NODES == NULL || DATA_GATWAY == NULL) {
        printf("Failed to create queue\n");
        return;
    }

    // Create binary semaphore for triggering MQTT publishing for nodes
    MQTT_PUB_TREGER_NODES = xSemaphoreCreateBinary();

    // Check if semaphore creation was successful
    if (MQTT_PUB_TREGER_NODES == NULL) {
        printf("Failed to create semaphore\n");
        return;
    }

    // Additional setup for the application (setup function not defined in the provided code)
    setup(0);

    // Create task for handling node communication
    xTaskCreate(vTaskHandleNodeComm, "Task 1", 2096, NULL, 1, myTask1Handle);

    // Create task for publishing data
    xTaskCreate(vTaskPublisherData, "Task 2", 2096, NULL, 1, myTask2Handle);

    // Create task for subscribing to commands
    xTaskCreate(vTaskSubscriberCommand, "Task 3", 2096, NULL, 1, myTask3Handle);
}


