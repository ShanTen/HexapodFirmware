#include "mbed.h"
#include "WiFiInterface.h"
#include "TCPSocket.h"
#include "mbed_events.h"

/* Includes */
#include "mbed.h"
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include <cstdio>
// #include "SX1278/sx1278.h"

/////////////////////////////////// Movement Command Macros /////////////////////////////////////////////

#define HOME_POSITION "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500D500\r\n" 

#define MOVE_FORWAD "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500D500\r\n#2P1300#3P1700#8P1700#9P1300#27P1700#29P1300T500D500\r\n#1P1300#7P1700#28P1700T500D500\r\n#2P1833#8P1167#27P1167T500D500\r\n#1P1500#2P1833#7P1500#28P1500T500D500\r\n#5P1300#6P1700#25P1700#26P1312#30P1300#32P1700T500D500\r\n#4P1300#24P1700#31P1300T500D500\r\n#2P1500#4P1500#5P1833#8P1500#24P1500#25P1167#27P1500#30P1833#31P1500T500D500\r\n#3P1500#5P1500#6P1500#9P1500#25P1500#26P1500#29P1500#30P1500#32P1500T500D500\r\n"
#define MOVE_BACK "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500D500\r\n#2P1300#3P1700#8P1700#9P1300#27P1700#29P1300T500D500\r\n#1P1700#7P1300#28P1300T500D500\r\n#2P1833#8P1167#27P1167T500D500\r\n#1P1500#7P1500#28P1500T500D500\r\n#5P1300#6P1700#25P1700#26P1300#30P1300#32P1700T500D500\r\n#4P1700#24P1300#31P1700T500D500\r\n#5P1833#25P1167#30P1833T500D500\r\n#2P1500#3P1500#4P1500#5P1833#8P1500#9P1500#24P1500#27P1500#29P1500#31P1500T500D500\r\n#5P1500#6P1500#25P1500#26P1500#30P1500#32P1500T500D500\r\n"
#define MOVE_LEFT "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500D500\r\n#1P1500#2P1300#3P1700#8P1700#9P1300#25P1500#27P1700#29P1300T500D500\r\n#1P1700#7P1700#28P1700T500D500\r\n#2P1833#8P1167#27P1167T500D500\r\n#1P1500#5P1300#6P1700#7P1500#25P1700#26P1300#28P1500#30P1300#32P1700T500D500\r\n#4P1700#5P1300#24P1700#25P1700#31P1700T500D500\r\n#5P1833#25P1167#30P1833T500D500\r\n#2P1500#3P1500#8P1500#9P1500#27P1500#29P1500T500D500\r\n#4P1500#24P1500#28P1500#31P1500T500D500\r\n#5P1500#6P1500#25P1500#26P1500#30P1500#32P1500T500D500\r\n"
#define MOVE_RIGHT "#1P1500#2P1500#3P1500#4P1500#5P1500#6P1500#7P1500#8P1500#9P1500#24P1500#25P1500#26P1500#27P1500#28P1500#29P1500#30P1500#31P1500#32P1500T500D500\r\n#1P1500#2P1700#3P1300#8P1300#9P1700#25P1500#27P1300#29P1700T500D500\r\n#1P1300#7P1300#28P1300T500D500\r\n#2P1833#8P1167#27P1167T500D500\r\n#1P1500#5P1700#6P1300#7P1500#25P1300#26P1700#28P1500#30P1700#32P1300T500D500\r\n#4P1300#5P1700#24P1300#25P1300#31P1300T500D500\r\n#5P1833#25P1167#30P1833T500D500\r\n#2P1500#3P1500#8P1500#9P1500#27P1500#29P1500T500D500\r\n#4P1500#24P1500#28P1500#31P1500T500D500\r\n#5P1500#6P1500#25P1500#26P1500#30P1500#32P1500T500D500\r\n"

///////////////////////////////////////////////////////////////////////////////////////////////////////


// WiFi credentials
// const char *ssid = "Chips2021";       // Replace with your WiFi SSID
// const char *password = "Roop$70965"; // Replace with your WiFi Password

const char *ssid = "SP";       // Replace with your WiFi SSID
const char *password = "22323554"; // Replace with your WiFi Password

// IP and port of the server
// const char *host_ip = "192.168.1.18";
const char *host_ip = "192.168.1.27";
const int port = 8080;

WiFiInterface *wifi;
TCPSocket socket;
SocketAddress server_addr;
EventQueue queue;
Thread t;
TCPSocket server;
bool enableLORA = 0;

static DevI2C devI2c(PB_11,PB_10);
static LPS22HBSensor press_temp(&devI2c);
static HTS221Sensor hum_temp(&devI2c);

#define TX_PIN PC_4  // Transmit pin (A0)
#define RX_PIN PC_5  // Receive pin (A1)

static BufferedSerial uart_Movement(TX_PIN, RX_PIN, 9600);  // UART3 -- Sending Movement Instructions => wired 
static DigitalIn mq_sensor(PA_15); 

// //Bing

static SPI spi_LoRA(PA_7, PA_6, PA_5); // SPI -- LoRA 
static DigitalOut cs(PB_6);
SX1278_LoRa lora(&spi, &nss);
// Configuration constants for India (865-867 MHz band)
#define FREQUENCY 865500000  // 865.5 MHz
#define BANDWIDTH 125000     // 125 kHz bandwidth
#define SPREADING_FACTOR 7   // SF7
#define CODING_RATE 5        // 4/5 coding rate
#define TX_POWER 14          // 14 dBm transmit power
#define PREAMBLE_LENGTH 8    // Preamble length
#define SX1278_REG_VERSION  0x42

void initializeLoRa() {
    spi.format(8, 0);
    spi.frequency(1000000);

    if (lora.init()) {
        lora.setFrequency(FREQUENCY);
        lora.setSpreadingFactor(SPREADING_FACTOR);
        lora.setBandwidth(BANDWIDTH);
        lora.setCodingRate(CODING_RATE);
        lora.setTxPower(TX_POWER);
        lora.setPreambleLength(PREAMBLE_LENGTH);
    }
}

uint8_t sx1278_read_register(uint8_t reg) {
    nss = 0;  // Select the SX1278 by pulling NSS low
    spi.write(reg & 0x7F);  // Send register address (0x7F is for read)
    uint8_t result = spi.write(0x00);  // Read the register value
    nss = 1;  // Deselect the SX1278
    return result;
}

void sx1278_write_register(uint8_t reg, uint8_t value) {
    nss = 0;  // Select the SX1278 by pulling NSS low
    spi.write(reg | 0x80);  // Send register address with write flag (0x80)
    spi.write(value);       // Write the value to the register
    nss = 1;  // Deselect the SX1278
}

//Chilling

bool StartsWith(const char *a, const char *b)
{
   if(strncmp(a, b, strlen(b)) == 0) return 1;
   return 0;
}



void poll_LoRA()
{

    float temp, pres, humd;
    int gasv;
    char json_buffer[128];

    press_temp.get_temperature(&temp);
    press_temp.get_pressure(&pres);
    hum_temp.get_humidity(&humd);
    int sensor_value = mq_sensor.read();

    sprintf(json_buffer, "SENSOR-DATA-{\"pressure\":%.2f, \"temperature\":%.2f, \"humidity\":%.2f, \"mq12\":%d }", pres, temp, humd, gasv);

    lora.send(json_buffer, sizeof(json_buffer));

    char instruction_buffer[128];

    if (lora.receive()) 
    {

        strcpy(instruction_buffer, lora.getMessage());

        // Process received data
        if(StartsWith(instruction_buffer, "MOVEMENT-UPED"))
        {
            printf("MOVE UP\n");
            uart_Movement.write(MOVE_FORWAD, strlen(MOVE_FORWAD));
        }
        else if (StartsWith(instruction_buffer, "MOVEMENT-DOWN"))
        {
            printf("MOVE DOWN\n");
            uart_Movement.write(MOVE_BACK, strlen(MOVE_BACK));
        }
        else if (StartsWith(instruction_buffer, "MOVEMENT-RITE"))
        {
            printf("MOVE RIGHT\n");
            uart_Movement.write(MOVE_RIGHT, strlen(MOVE_RIGHT));
        }
        else if (StartsWith(instruction_buffer, "MOVEMENT-LEFT"))
        {
            printf("MOVE LEFT\n");
            uart_Movement.write(instruction_buffer, strlen(MOVE_LEFT));
        }
        else if (StartsWith(instruction_buffer, "TOGGLE-LORA"))
        {
            enableLORA = !enableLORA;
            printf("Toggling LoRA state to [%d]\n", enableLORA);
        }
        else
        {
            printf("Received on LoRA: %s\n",  instruction_buffer);
        }
    }

}

//always on
void poll_TCP()
{
    

    float temp, pres, humd;
    char json_buffer[128];

    press_temp.get_temperature(&temp);
    press_temp.get_pressure(&pres);
    hum_temp.get_humidity(&humd);

    sprintf(json_buffer, "SENSOR-DATA-{\"pressure\":%.2f, \"temperature\":%.2f, \"humidity\":%.2f}", pres, temp, humd);

    printf("Called send\n");

    int ret = socket.send(json_buffer, strlen(json_buffer)); 
    if (ret < 0) {
        printf("ERROR: Could not send data (%d).\n", ret);
        return ;
    }

    printf("Called recv\n");
    char buffer[1024];
    ret = socket.recv(buffer, sizeof(buffer) - 1); // Leave space for null terminator                       
    // const char *COMMANDS[] = {"G1F1", "G2F1", "G3F1", "G4F1"};
    // const int COMMAND_SIZE = 6;

    if (ret < 0) {
        printf("ERROR: Could not receive data (%d).\n", ret);
    } else {
        buffer[ret] = '\0'; // Null-terminate the received data

        if(StartsWith(buffer, "MOVEMENT-UPED"))
        {
            printf("MOVE UP\n");
            uart_Movement.write(MOVE_FORWAD, strlen(MOVE_FORWAD));
        }
        else if (StartsWith(buffer, "MOVEMENT-DOWN"))
        {
            printf("MOVE DOWN\n");
            uart_Movement.write(MOVE_BACK, strlen(MOVE_BACK));
        }
        else if (StartsWith(buffer, "MOVEMENT-RITE"))
        {
            printf("MOVE RIGHT\n");
            uart_Movement.write(MOVE_RIGHT, strlen(MOVE_RIGHT));
        }
        else if (StartsWith(buffer, "MOVEMENT-LEFT"))
        {
            printf("MOVE LEFT\n");
            uart_Movement.write(MOVE_LEFT, strlen(MOVE_LEFT));
        }
        else if (StartsWith(buffer, "TOGGLE-LORA"))
        {
            enableLORA = !enableLORA;
            printf("Toggling LoRA state to [%d]\n", enableLORA);
        }
        else
        {
            printf("DataRecieved - [%s]\n", buffer);
        }
        
    }   
}

int main() {
    initializeLoRa();

    uart_Movement.set_format(8, BufferedSerial::None, 1); 

    //sensor buffers
    uint8_t id;
    float value1, value2;

    //init sensors 
    press_temp.init(NULL);  
    hum_temp.init(NULL);

    //enable sensors
    press_temp.enable();
    hum_temp.enable();

    press_temp.read_id(&id);
    printf("LPS22HB pressure & temperature    = 0x%X\r\n", id);

    hum_temp.read_id(&id);
    printf("HTS221  humidity & temperature    = 0x%X\r\n", id);

    printf("Connecting to WiFi...\n");

    // Get a handle to the WiFi interface
      SocketAddress a;
    wifi = WiFiInterface::get_default_instance();
    wifi->get_ip_address(&a);
    if (!wifi) {
        printf("ERROR: No WiFiInterface found.\n");
        return -1;
    }
    

    // Connect to the WiFi network
    int ret = wifi->connect(ssid, password, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        printf("ERROR: Connection to WiFi failed (%d).\n", ret);
        return -1;
    }
    printf("Connected to WiFi IP is: %s\n", a.get_ip_address());


    // Set the server address
    server_addr.set_ip_address(host_ip);
    server_addr.set_port(port);


    // Open the socket and connect to the server
    ret = socket.open(wifi);
    if (ret != 0) {
        printf("ERROR: Could not open socket (%d).\n", ret);
        return -1;
    }

    //connect to my "server"
    ret = socket.connect(server_addr);
    if (ret != 0) {
        printf("ERROR: Could not connect to server (%d).\n", ret);
        return -1;
    }

    printf("Connected to server at %s:%d\n", host_ip, port);

    // Send a simple message to the server
    const char *message = "ACK - Hello from DISCO_L475VG_IOT01A!\n";
    ret = socket.send(message, strlen(message));
    if (ret < 0) {
        printf("ERROR: Could not send data (%d).\n", ret);
        return -1;
    }
    
    printf("Message sent to server: %s\n", message);

    while(1)
    {
        poll_TCP();
        if(enableLORA){
            poll_LoRA()
            printf("enable LoRa\n");
        }
        ThisThread::sleep_for(500ms);
    }
}