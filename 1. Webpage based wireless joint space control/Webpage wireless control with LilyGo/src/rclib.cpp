#include "rclib.h"

int txid;
int num1;
int num2;
int Speed;
int Position;
int panduan;
int panduan1;
float q, p;
int yanshi;
int ii, j;
int c[99];
float a1, a2, a3, a4, a5, a6, a7, a8;
float b1[99], b2[99], b3[99], b4[99], b5[99], b6[99], b7[99], b8[99];

bool can_driver_installed = false;
unsigned long wait_time = 1000;

bool rc_init()
{
#ifdef LILYGO
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
    pinMode(23, OUTPUT);
    digitalWrite(23, LOW);
#endif

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // Look in the api-reference for other speed sets.
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        return false;
    }

    // Start TWAI driver
    if (twai_start() != ESP_OK)
    {
        return false;
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK)
    {
        return false;
    }

    // TWAI driver is now successfully installed and started
    can_driver_installed = true;
    return true;
}

// Send a command to cause the motor to stop running in an emergency and keep it enabled.
void rc_estop(int id)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x01;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 0;
    twai_transmit(&txbuffer, 0);
}

/*Send instructions to set the motor status - data[0] represents the index value. 
Send 0x00 to disable the motor, send 0x01 to enable the motor, 0x02: Restart the motor main control; 
0x03: Reset the motor parameters; 0x04: Reset the motor Error status reset*/

void rc_set_status(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x03;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);
}

/*Send instructions to read motor status*/
int rc_read_status(int id)
{
    if (!can_driver_installed)
        return -1;

    int res = -1;

    twai_message_t txbuffer;
    twai_message_t rxbuffer;

    txbuffer.identifier = id << 6 | 0x05;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 0;
    twai_transmit(&txbuffer, 0);

    unsigned long startMillis = millis();
    while (millis() - startMillis < wait_time)
    {

        if (twai_receive(&rxbuffer, 1) == ESP_OK)
        {
            if (rxbuffer.identifier == txbuffer.identifier)
            {
                res = rxbuffer.data[0];
                break;
            }
        }
    }
    return res;
}

/*Send a command to set the motor operating mode - Data[0] represents the index value. 
Send 0x00 to set the motor operating mode to torque mode, send 0x01 to set the motor operating mode to speed mode, 
and send 0x02 to set the motor operating mode to position mode.*/
void rc_set_mode(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x07;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);
}

/*Send a command to read the motor operating mode*/
int rc_read_mode(int id)
{
    if (!can_driver_installed)
        return -1;

    int res = -1;

    twai_message_t txbuffer;
    twai_message_t rxbuffer;

    txbuffer.identifier = id << 6 | 0x09;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 0;
    twai_transmit(&txbuffer, 0);

    unsigned long startMillis = millis();
    while (millis() - startMillis < wait_time)
    {

        if (twai_receive(&rxbuffer, 1) == ESP_OK)
        {
            if (rxbuffer.identifier == txbuffer.identifier)
            {
                res = rxbuffer.data[0];
                break;
            }
        }
    }
    return res;
}

/*Send a command to set the current position of the motor to zero*/
void rc_set_zp(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x0b;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);
}

/*Send instructions to set the motor PID parameters, Data[0] represents the index value, and sending 0x00 represents setting the parameters of the position loop P，
0x01: Position loop I; 0x02: Position loop D; 0x03: Position loop slope; 0x04: Position loop filter period; 
0x05: Speed ​​loop P; 0x06: Speed ​​loop I; 0x07: Speed ​​loop D; 0x08: Speed ​​loop slope; 0x09 : Speed ​​loop filter period; 
0x0A: Q-axis current loop P; 0x0B: Q-axis current loop I; 0x0C: Q-axis current loop D; 0x0D: Q-axis current loop slope; 0x0E: Q-axis current loop filter period; 
0x0F: D Axis current loop P; 0x10: D-axis current loop I; 0x11: D-axis current loop D; 0x12: D-axis current loop slope; 0x13: D-axis current loop filter period.*/
void rc_set_PID(int id, uint8_t suoyin, float shuju)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x0d;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 5;
    txbuffer.data[0] = suoyin;
    float *p2 = (float *)&txbuffer.data[1];
    *p2 = shuju;
    twai_transmit(&txbuffer, 0);
}

/*Send an instruction to read the PID parameters, Data[0] represents the index value, send 0x00 to read the position loop P parameter, 
0x01: position loop I; 0x02: position loop D; 0x03: position loop slope; 0x04: position loop filter period; 
0x05: Speed ​​loop P; 0x06: Speed ​​loop I; 0x07: Speed ​​loop D; 0x08: Speed ​​loop slope; 0x09: Speed ​​loop filter period; 
0x0A: Q-axis current loop P; 0x0B: Q-axis current loop I; 0x0C: Q Axis current loop D; 0x0D: Q-axis current loop slope; 0x0E: Q-axis current loop filter period; 
0x0F: D-axis current loop P; 0x10: D-axis current loop I; 0x11: D-axis current loop D; 0x12: D-axis Current loop slope; 0x13: D-axis current loop filter period*/
float rc_read_PID(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return -99999.0f;

    float res = -99999.0f;

    twai_message_t txbuffer;
    twai_message_t rxbuffer;

    txbuffer.identifier = id << 6 | 0x0F;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);

    unsigned long startMillis = millis();
    while (millis() - startMillis < wait_time)
    {

        if (twai_receive(&rxbuffer, 1) == ESP_OK)
        {
            if (rxbuffer.identifier == txbuffer.identifier)
            {
                res = *(float *)&rxbuffer.data[0];
                break;
            }
        }
    }
    return res;
}

/*Send instructions to set motor limit parameters, Data[0] represents the index value, 
0x01: motor temperature limit; 0x02: voltage limit; 0x03: current limit; 0x04: speed limit; 
0x05: position limit - minimum value; 0x06: position limit - maximum Value; 0x07: Brake start; 0x08: Brake maintenance; 0x09: Overvoltage value; 
Data[1]-Data[4] data from low to high, input the converted hexadecimal number*/
void rc_set_lim(int id, uint8_t suoyin, float shuju)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x11;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 5;
    txbuffer.data[0] = suoyin;
    float *p2 = (float *)&txbuffer.data[1];
    *p2 = shuju;
    twai_transmit(&txbuffer, 0);
}

/*Send an instruction to read the limit parameters, Data[0] represents the index value, 
0x01: Motor temperature limit; 0x02: Voltage limit; 0x03: Current limit; 0x04: Speed ​​limit; 
0x05: Position limit - minimum value; 0x06: Position limit - maximum Value; 0x07: Brake start; 0x08: Brake maintenance; 0x09: Overvoltage value;*/
float rc_read_lim(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return -99999.0f;

    float res = -99999.0f;

    twai_message_t txbuffer;
    twai_message_t rxbuffer;

    txbuffer.identifier = id << 6 | 0x13;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);

    unsigned long startMillis = millis();
    while (millis() - startMillis < wait_time)
    {

        if (twai_receive(&rxbuffer, 1) == ESP_OK)
        {
            if (rxbuffer.identifier == txbuffer.identifier)
            {
                res = *(float *)&rxbuffer.data[0];
                break;
            }
        }
    }
    return res;
}

/*Send instructions to make the motor run according to the target value according to the current mode. Data[0]-Data[3] data from low to high, input the converted hexadecimal number*/
void rc_spr(int id, float shuju)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x15;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 4;
    float *p1 = (float *)&txbuffer.data[0];
    *p1 = shuju;
    twai_transmit(&txbuffer, 0);
}

/*In position mode, send a command to make the motor run to the specified position according to the specified speed and current limit parameters. 
The value of Data[0]-Data[3] represents the position to which the motor runs, and the value of Data[4]-Data[7] represents the speed at which the motor runs.*/
void rc_spt(int id, float position, float speed)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x17;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 8;
    float *p1 = (float *)&txbuffer.data[0];
    *p1 = position;
    float *p2 = (float *)&txbuffer.data[4];
    *p2 = speed;
    twai_transmit(&txbuffer, 0);
}

/*Send command to set the position data parameters of continuous trajectory operation
Data[0]-Data[1] represents the index value, 00 00 is to set point 0, Data[2]-Data[5] data from low to high, enter the converted hexadecimal number.*/
void rc_set_ctp(int id, uint16_t suoyin, float shuju)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x19;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 6;
    uint16_t *p1 = (uint16_t *)&txbuffer.data[0];
    *p1 = suoyin;
    float *p2 = (float *)&txbuffer.data[4];
    *p2 = shuju;
    twai_transmit(&txbuffer, 0);
}

/*Send command to set the speed data parameters of continuous trajectory operation
Data[0]—Data[1] represents the index value, 00 00 is to set point 0, Data[2]—Data[5] data from low to high, enter the converted hexadecimal number.*/
void rc_set_ctv(int id, uint16_t suoyin, float shuju)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x1b;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 6;
    uint16_t *p1 = (uint16_t *)&txbuffer.data[0];
    *p1 = suoyin;
    float *p2 = (float *)&txbuffer.data[4];
    *p2 = shuju;
    twai_transmit(&txbuffer, 0);
}

/*Send command to set the moment flow data parameters for continuous trajectory operation
Data[0]-Data[1] represents the index value, 00 is to set point 0, Data[2]-Data[5] data from low to high, input the converted hexadecimal number.*/
void rc_set_ctmf(int id, u_int16_t suoyin, float shuju)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x1b;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 6;
    uint16_t *p1 = (uint16_t *)&txbuffer.data[0];
    *p1 = suoyin;
    float *p2 = (float *)&txbuffer.data[4];
    *p2 = shuju;
    twai_transmit(&txbuffer, 0);
}

/*Send a command to make the motor run according to the specified trajectory data
Data[0]-Data[1] represents the index value, 00 means running according to the data of point 0*/
void rc_tdr(int id, uint16_t suoyin)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x1f;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 2;
    uint16_t *p1 = (uint16_t *)&txbuffer.data[0];
    *p1 = suoyin;
    twai_transmit(&txbuffer, 0);
}
/*Send a command to record the current position, speed and moment flow data to the specified data index position
Data[0]-Data[1] represents the index value, 00 means recording to point 0 */
void rc_record(int id, uint16_t suoyin)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x21;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 2;
    uint16_t *p1 = (uint16_t *)&txbuffer.data[0];
    *p1 = suoyin;
    twai_transmit(&txbuffer, 0);
}

/*Send command to read the data when the motor is running
Data[0] represents the index value, 0x00 means reading the current position data of the motor
0x01: current speed; 0x02: Q axis current; 0x03: Q axis voltage; 0x04: D axis current;
0x05: D axis voltage; 0x06: current motor temperature; 0x07: program version;
0x0A: position + speed; 0x0B: Q axis voltage + Q axis current; 0x0C: D axis voltage + D axis current;*/
float rc_read_rd(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return -99999.0f;

    float res = -99999.0f;

    twai_message_t txbuffer;
    twai_message_t rxbuffer;

    txbuffer.identifier = id << 6 | 0x23;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);

    unsigned long startMillis = millis();
    while (millis() - startMillis < wait_time)
    {

        if (twai_receive(&rxbuffer, 1) == ESP_OK)
        {
            if (rxbuffer.identifier == txbuffer.identifier)
            {
                res = *(float *)&rxbuffer.data[0];
                break;
            }
        }
    }
    return res;
}

/*Send a command to set the CAN ID, Data[0] represents the motor serial number to be set
0x01 means setting the motor ID to 57, 0x02 means setting the motor ID to 97,
0x03 means setting the motor ID to d7, and so on. For every increase of 1 in the serial number, the corresponding ID increases by 4 (ID is in hexadecimal)*/
void rc_set_CAN_ID(int id, uint8_t suoyin)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x25;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 1;
    txbuffer.data[0] = suoyin;
    twai_transmit(&txbuffer, 0);
}

/*Send instructions to restore motor parameters, restart the motor and recalibrate*/
void rc_reset(int id)
{
    if (!can_driver_installed)
        return;

    twai_message_t txbuffer;
    txbuffer.identifier = id << 6 | 0x27;
    txbuffer.flags = 0;
    txbuffer.data_length_code = 0;
    twai_transmit(&txbuffer, 0);
}

/*Receive data, determine which function is being used by judging the received ID, and further judge and display the current status based on the returned data. 
Note: A validity judgment is required for the received data.*/
