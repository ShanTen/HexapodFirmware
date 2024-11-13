#include "mbed.h"
#include "WiFiInterface.h"
#include "TCPSocket.h"
#include "mbed_events.h"

/* Includes */
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include <cstdio>
#include <cstring>
#include "LORA/platform/mbed/lora-mbed.h"
/////////////////////////////////// Movement Command Macros /////////////////////////////////////////////

#define HOME_POSITION "#G4C1\n\r"
#define MOVE_FORWAD "#G1C1\n\r"
#define MOVE_BACK "#G2C1\n\r"
#define MOVE_RIGHT "#G3C1\n\r"
#define MOVE_LEFT "#G5C1\n\r" 

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

/********************** Utils ***********************************/

bool StartsWith(const char *a, const char *b)
{
   if(strncmp(a, b, strlen(b)) == 0) return 1;
   return 0;
}

/****************************Globals Gas Sensor**********************************************/
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


///////////////////////////////////////////////////////////////////////////////////////////////////////

// WiFi credentials
const char *ssid = "Hexapod";      
const char *password = "12345678\0"; 

// IP and port of the server
const char *host_ip = "192.168.0.101\0"; //change this if you are screwed
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

//////////////////////////////////////// LoRa Set up ////////////////////////////////////////

lora dev;
lora_mbed mbed_dev;

#define LORA_RADIO_RECEIVER_ID  0xAB
#define LORA_RADIO_SENDER_ID    0xCD
 
/* Pinout dependednt on your hw */
#define RADIO_MOSI_PIN  PA_5
#define RADIO_MISO_PIN  PA_6
#define RADIO_SCK_PIN   PA_7
#define RADIO_NSS_PIN   PA_8
#define RADIO_RST_PIN   PA_9
#define RADIO_DI0_PIN   PA_10

typedef struct
{
	uint8_t receiver_id;           // Receiver address
	uint8_t sender_id;             // Sender address
	uint8_t msg_id;                // Message ID
	uint8_t payload_len;           // Message payload length
} radio_layer_msg_header;

typedef struct
{
	uint8_t hdr[3];
	uint8_t status;
	char sensor_data[256];
} radio_msg_sensor_frame;

//sender code 
void radio_send(radio_msg_sensor_frame *msgf)
{
    static uint8_t msg_cnt = 0;
    if (!msgf)
    {
        return;
    }

    // Send result data
    radio_layer_msg_header hdr;
    hdr.receiver_id = LORA_RADIO_RECEIVER_ID;
    hdr.sender_id = LORA_RADIO_SENDER_ID;
    hdr.msg_id = msg_cnt++;
    hdr.payload_len = sizeof(radio_msg_sensor_frame);
    lora_begin_packet(&dev, false);                     // start packet
    lora_write_data(&dev, (const uint8_t*)&hdr, sizeof(radio_layer_msg_header));
    lora_write_data(&dev, (const uint8_t*)msgf, sizeof(radio_msg_sensor_frame));
    lora_end_packet(&dev, false);                       // finish packet and send it
    // Switch to RX mode
    lora_receive(&dev, 0);
}

static void on_rx_done(void *ctx, int packet_size)
{
    lora *const dev = (lora*const) ctx;

    radio_layer_msg_header hdr;
    radio_msg_sensor_frame frame;
    uint8_t *p = (uint8_t*)&frame;
    size_t i = 0;

    if (packet_size == 0)
    {
        goto err;
    }

    // Read packet header bytes:
    hdr.receiver_id = lora_read(dev);   // recipient address
    hdr.sender_id = lora_read(dev);     // sender address
    hdr.msg_id = lora_read(dev);        // incoming msg ID
    hdr.payload_len = lora_read(dev);   // incoming msg length

    // If the recipient is valid,
    if (hdr.receiver_id != 0xFE)
    {
        goto err;
    }

    // Check payload length
    if (hdr.payload_len != sizeof(radio_msg_sensor_frame))
    {
        goto err;
    }

    // Read payload frame
    while (lora_available(dev) && i < hdr.payload_len)
    {
        *(p+i) = (uint8_t)lora_read(dev);
        i++;
    }

    // 256 byte string buffer, possible movement options = {"MOVEMENT-UPED", "MOVEMENT-RITE", "MOVEMENT-LEFT", "MOVEMENT-BACK"}
    if(StartsWith(frame.sensor_data, "MOVEMENT-UPED"))
    {
        printf("MOVE UP\n");
        uart_Movement.write(MOVE_FORWAD, strlen(MOVE_FORWAD));
    }
    else if (StartsWith(frame.sensor_data, "MOVEMENT-DOWN"))
    {
        printf("MOVE DOWN\n");
        uart_Movement.write(MOVE_BACK, strlen(MOVE_BACK));
    }
    else if (StartsWith(frame.sensor_data, "MOVEMENT-RITE"))
    {
        printf("MOVE RIGHT\n");
        uart_Movement.write(MOVE_RIGHT, strlen(MOVE_RIGHT));
    }
    else if (StartsWith(frame.sensor_data, "MOVEMENT-LEFT"))
    {
        printf("MOVE LEFT\n");
        uart_Movement.write(MOVE_LEFT, strlen(MOVE_LEFT));
    }
    else if (StartsWith(frame.sensor_data, "TOGGLE-LORA"))
    {
        enableLORA = !enableLORA;
        printf("Toggling LoRA state to [%d]\n", enableLORA);
    }
    else
    {
        printf("DataRecieved - [%s]\n", frame.sensor_data);
    }
    err:
        // Switch back into rx mode
        lora_receive(dev, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////

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

    sprintf(json_buffer, "SENSOR-DATA-{\"pressure\":%.2f, \"temperature\":%.2f, \"humidity\":%.2f, \"gascomposition\":%d}", pres, temp, humd, isSafeGas);

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

void sendDataViaLoRa()
{
    radio_msg_sensor_frame msgf =
    {
        .hdr = {'T','R','0'}, //header thing
    };

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

    sprintf(json_buffer, "SENSOR-DATA-{\"pressure\":%.2f, \"temperature\":%.2f, \"humidity\":%.2f, \"gascomposition\":%d}", pres, temp, humd, isSafeGas);

    msgf.status = 0;
    strncpy(msgf.sensor_data, json_buffer, 256);
    radio_send(&msgf);
}


int main() {
    printf("Hit 0\r\n");
    
    //initialize lora 
    lora dev;
    lora_mbed mbed_dev;

    // Set mbed_dev
    SPI spi(RADIO_MOSI_PIN, RADIO_MISO_PIN, RADIO_SCK_PIN);
    DigitalOut nss(RADIO_NSS_PIN);
    DigitalInOut rst(RADIO_RST_PIN);
    InterruptIn dio0(RADIO_DI0_PIN);

    mbed_dev.spi =   &spi;
    mbed_dev.nss =   &nss;
    mbed_dev.reset = &rst;
    mbed_dev.dio0 =  &dio0;

    dev.frequency = 8655E5;
    dev.on_receive = NULL;
    dev.on_tx_done = NULL;

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

    // Send ACK message to the server
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
            printf("LoRa Enabled.");
            sendDataViaLoRa();
            lora_mbed_init(&dev, &mbed_dev);
            lora_on_receive(&dev, on_rx_done);
        }

        ThisThread::sleep_for(100ms);
    }
}