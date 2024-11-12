#include "mbed.h"
#include "WiFiInterface.h"
#include "TCPSocket.h"
#include "mbed_events.h"

/* Includes */
#include "mbed.h"
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include <cstdio>
#include <string>
// #include "SX1278/sx1278.h"

//////////////////////////////////// Gas Sensor Macros and Definitions //////////////////////////////////

#define          GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define          GAS_SMOKE                    (2)


#define         MQ_PIN                       A2     //define which   analog input channel you are going to use
#define         RL_VALUE                     (5)      //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR           (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                      //which is derived from the   chart in datasheet
 
/**********************Software Related Macros***********************************/
#define          CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are   going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL   (500)   //define the time interal(in milisecond) between each samples in the
                                                      //cablibration phase
#define          READ_SAMPLE_INTERVAL         (50)    //define how many samples you are   going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)      //define the time interal(in milisecond) between each samples in 


/****************************Globals**********************************************/
float            LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent"
                                                    //to   the original curve. 
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float            COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent" 
                                                    //to   the original curve.
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float            SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve.   
                                                    //with these two points,   a line is formed which is "approximately equivalent" 
                                                    //to   the original curve.
                                                    //data   format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float            Ro           =  10;                 //Ro is initialized to 10 kilo ohms


/*************************** MQCalibration **************************************
Input:    mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function   assumes that the sensor is in clean air. It use  
         MQResistanceCalculation   to calculates the sensor resistance in clean air 
         and then divides it   with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs   slightly between different sensors.
**********************************************************************************/   

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}


float MQCalibration()
{
  int i;
  float val=0;
  AnalogIn   raw_adc(MQ_PIN);  
   
   for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples

     val += MQResistanceCalculation(raw_adc);

    thread_sleep_for(CALIBRATION_SAMPLE_INTERVAL);
   }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average   value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided   by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according   to the chart in the datasheet 
 
  return val; 
}

float MQRead()
{
  int i;
  float rs=0;
  AnalogIn   raw_adc(MQ_PIN);  
 
  for (i=0;i<READ_SAMPLE_TIMES;i++)   {
    rs += MQResistanceCalculation(raw_adc);
    thread_sleep_for(CALIBRATION_SAMPLE_INTERVAL);
   }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,(   ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id   == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else   if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
   } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
   }    
 
  return 0;
}


/////////////////////////////////// Movement Command Macros /////////////////////////////////////////////

#define HOME_POSITION "#G4C1\n\r"
#define MOVE_FORWAD "#G1C1\n\r"
#define MOVE_BACK "#G2C1\n\r"
#define MOVE_RIGHT "#G3C1\n\r"
#define MOVE_LEFT "#G5C1\n\r" 

///////////////////////////////////////////////////////////////////////////////////////////////////////

// WiFi credentials
const char *ssid = "Hexapod";      
const char *password = "12345678"; 

// IP and port of the server
const char *host_ip = "192.168.1.102"; //change this if you are screwed
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

#define TX_PIN PC_4  // Receive pin (A1) 
#define RX_PIN PC_5  // Transmit pin (A0)

static BufferedSerial uart_Movement(TX_PIN, RX_PIN, 9600);  // UART3 -- Sending Movement Instructions => wired 
static DigitalIn mq_sensor(PA_15); 

// //Bing

static SPI spi_LoRA(PA_7, PA_6, PA_5); // SPI -- LoRA 
static DigitalOut cs(PB_6);
// SX1278_LoRa lora(&spi, &nss);
// Configuration constants for India (865-867 MHz band)
#define FREQUENCY 865500000  // 865.5 MHz
#define BANDWIDTH 125000     // 125 kHz bandwidth
#define SPREADING_FACTOR 7   // SF7
#define CODING_RATE 5        // 4/5 coding rate
#define TX_POWER 14          // 14 dBm transmit power
#define PREAMBLE_LENGTH 8    // Preamble length
#define SX1278_REG_VERSION  0x42

/*void initializeLoRa() {
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
}*/

/*uint8_t sx1278_read_register(uint8_t reg) {
    nss = 0;  // Select the SX1278 by pulling NSS low
    spi.write(reg & 0x7F);  // Send register address (0x7F is for read)
    uint8_t result = spi.write(0x00);  // Read the register value
    nss = 1;  // Deselect the SX1278
    return result;
}*/

/*
void sx1278_write_register(uint8_t reg, uint8_t value) {
    nss = 0;  // Select the SX1278 by pulling NSS low
    spi.write(reg | 0x80);  // Send register address with write flag (0x80)
    spi.write(value);       // Write the value to the register
    nss = 1;  // Deselect the SX1278
}*/

/*
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
*/

bool StartsWith(const char *a, const char *b)
{
   if(strncmp(a, b, strlen(b)) == 0) return 1;
   return 0;
}

//always on
void poll_TCP()
{
    float temp, pres, humd;
    char json_buffer[256];

    int PPMlpg, PPMcarbonmonoxide, PPMsmoke;
    bool isSafeGas = true;
    
    press_temp.get_temperature(&temp);
    press_temp.get_pressure(&pres);
    hum_temp.get_humidity(&humd);
    
    PPMlpg = MQGetGasPercentage(MQRead()/Ro, GAS_LPG);
    PPMcarbonmonoxide = MQGetGasPercentage(MQRead()/Ro, GAS_CO);
    PPMsmoke = MQGetGasPercentage(MQRead()/Ro, GAS_SMOKE);
    
    if(PPMsmoke > 3 || PPMcarbonmonoxide > 2 || PPMlpg > 3) isSafeGas = false;

    sprintf(json_buffer, "SENSOR-DATA-{\"pressure\":%.2f, \"temperature\":%.2f, \"humidity\":%.2f, \"isSafe\":%d}", pres, temp, humd, isSafeGas);

    printf("Called send\n");

    int ret = socket.send(json_buffer, strlen(json_buffer)); 
    if (ret < 0) {
        printf("ERROR: Could not send data (%d).\n", ret);
        return ;
    }

    printf("Called recv\n");
    char buffer[1024];
    ret = socket.recv(buffer, sizeof(buffer) - 1); // Leave space for null terminator                       

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
    // initializeLoRa();

    uart_Movement.set_format(8, BufferedSerial::None, 1); 
    uart_Movement.write(HOME_POSITION, strlen(HOME_POSITION));

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

    printf("Calibrating gas sensor...\r\n");
    float Ro = MQCalibration();
    printf("Calibration done.\r\n");


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

    //connect to hexapod central command 
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

    if(enableLORA) printf("LoRa Enabled.");

    while(1)
    {
        poll_TCP();

        if(enableLORA)
        {
            // poll_LoRA();
            printf("LoRa Enabled.");
        }

        ThisThread::sleep_for(100ms);
    }
}