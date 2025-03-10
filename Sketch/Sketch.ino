/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#include <util/atomic.h>
#include <EEPROM.h>
#include <string.h>

#include "iface_nrf24l01.h"
#include "nrf24l01.h"
#include "xn297_emu.h"

// supported protocols
enum {
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
};

static uint8_t current_protocol = PROTO_CX10_BLUE; // XXX

//SPI Comm.pins with nRF24L01
static const uint8_t MOSI_pin = 3; // D3
static const uint8_t SCK_pin  = 4; // D4
static const uint8_t CE_pin   = 5; // D5
static const uint8_t MISO_pin = A0; 
static const uint8_t CS_pin   = A1;

static const TX_Power RF_POWER = TX_POWER_80mW;

static const uint32_t CX10_GREEN_PACKET_LENGTH = 15;
static const uint32_t CX10_BLUE_PACKET_LENGTH = 19;
static const uint32_t CX10_BLUE_PACKET_PERIOD = 6000;
static const uint32_t CX10_GREEN_PACKET_PERIOD = 3000;
static const uint32_t CX10_GREEN_BIND_COUNT = 1000;
static const uint8_t  CX10_RF_BIND_CHANNEL = 0x02;
static const uint32_t CX10_NUM_RF_CHANNELS = 4;

// PPM stream settings
static const uint8_t CHANNELS = 12; // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) 
    AUX7,  // (CH11) 
    AUX8,  // (CH12) Reset / Rebind
};

static const uint16_t PPM_MIN = 1000;
static const uint16_t PPM_SAFE_THROTTLE = 1050;
static const uint16_t PPM_MID = 1500;
static const uint16_t PPM_MAX = 2000;
static const uint16_t PPM_MIN_COMMAND = 1300;
static const uint16_t PPM_MAX_COMMAND = 1700;

// EEPROM locations
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

static uint16_t overrun_cnt;
static uint8_t transmitterID[4];
static volatile bool ppm_ok = false;
static uint8_t packet[32];
static bool reset = true;
static volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

static String inputString = "";         // a string to hold incoming data
static boolean stringComplete = false;  // whether the string is complete
static char *p, *i;
static char* c = new char[200 + 1]; // match 200 characters reserved for inputString later
static char* errpt;
static uint8_t ppm_cnt;

static uint8_t CX10_txid[4]; // transmitter ID
static uint8_t CX10_freq[4]; // frequency hopping table
static uint8_t CX10_current_chan = 0;
static uint8_t CX10_packet_length;
static uint32_t CX10_packet_period;
static const uint8_t CX10_tx_rx_id[] = {0xCC,0xCC,0xCC,0xCC,0xCC};

static void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if (renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

static void selectProtocol()
{
    // Modified and commented out lines so that Cheerson CX-10 Blue is always selected
  
    // wait for multiple complete ppm frames
    ppm_ok = false;

    // startup stick commands
    //if (ppm[RUDDER] < PPM_MIN_COMMAND)        // Rudder left
    set_txid(true);                      // Renew Transmitter ID
    
    // update eeprom 
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
}


static void CX10_Write_Packet(uint8_t init)
{
    uint8_t offset = 0;
    if (current_protocol == PROTO_CX10_BLUE)
        offset = 4;
    packet[0] = init;
    packet[1] = CX10_txid[0];
    packet[2] = CX10_txid[1];
    packet[3] = CX10_txid[2];
    packet[4] = CX10_txid[3];
    // packet[5] to [8] (aircraft id) is filled during bind for blue board CX10
    packet[5+offset] = lowByte(3000-ppm[AILERON]);
    packet[6+offset]= highByte(3000-ppm[AILERON]);
    packet[7+offset]= lowByte(3000-ppm[ELEVATOR]);
    packet[8+offset]= highByte(3000-ppm[ELEVATOR]);
    packet[9+offset]= lowByte(ppm[THROTTLE]);
    packet[10+offset]= highByte(ppm[THROTTLE]);
    packet[11+offset]= lowByte(ppm[RUDDER]);
    packet[12+offset]= highByte(ppm[RUDDER]);
    if (ppm[AUX2] > PPM_MAX_COMMAND)
        packet[12+offset] |= 0x10; // flip flag
    // rate / mode
    if (ppm[AUX1] > PPM_MAX_COMMAND) // mode 3 / headless on CX-10A
        packet[13+offset] = 0x02;
    else if (ppm[AUX1] < PPM_MIN_COMMAND) // mode 1
        packet[13+offset] = 0x00;
    else // mode 2
        packet[13+offset] = 0x01;
    packet[14+offset] = 0x00;
    if (current_protocol == PROTO_CX10_BLUE) {
        // snapshot (CX10-C)
        if (ppm[AUX3] < PPM_MAX_COMMAND)
            packet[13+offset] |= 0x10;
        // video recording (CX10-C)
        if (ppm[AUX4] > PPM_MAX_COMMAND)
            packet[13+offset] |= 0x08;
    }    

    XN297_WritePayload(packet, CX10_packet_length);
}

static uint32_t process_CX10()
{
    uint32_t nextPacket = micros() + CX10_packet_period;
    XN297_Configure(
            _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));

    // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_freq[CX10_current_chan++]);
    CX10_current_chan %= CX10_NUM_RF_CHANNELS;
    CX10_Write_Packet(0x55);
    return nextPacket;
}

static void CX10_init()
{
    uint8_t i;
    switch (current_protocol) {
        case PROTO_CX10_BLUE:
            for(i=0; i<4; i++) {
                packet[5+i] = 0xFF; // clear aircraft ID
            }
            CX10_packet_length = CX10_BLUE_PACKET_LENGTH;
            CX10_packet_period = CX10_BLUE_PACKET_PERIOD;
            break;
        case PROTO_CX10_GREEN:
            CX10_packet_length = CX10_GREEN_PACKET_LENGTH;
            CX10_packet_period = CX10_GREEN_PACKET_PERIOD;
            break;
    }

    for(i=0; i<4; i++) {
        CX10_txid[i] = transmitterID[i];
    }
    CX10_txid[1] &= 0x2F;
    CX10_freq[0] = (CX10_txid[0] & 0x0F) + 0x03;
    CX10_freq[1] = (CX10_txid[0] >> 4) + 0x16;
    CX10_freq[2] = (CX10_txid[1] & 0x0F) + 0x2D;
    CX10_freq[3] = (CX10_txid[1] >> 4) + 0x40;

    CX10_current_chan = 0;
    NRF24L01_Reset();
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    delay(10);
    XN297_SetTXAddr(CX10_tx_rx_id,5);
    XN297_SetRXAddr(CX10_tx_rx_id,5);
    NRF24L01_FlushTx();

    // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     

    // No Auto Acknowledgment on all data pipes
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      

    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, CX10_packet_length); // rx pipe 0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);

    // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       

    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits

    delay(150);
}

static void CX10_bind()
{
    uint16_t counter=CX10_GREEN_BIND_COUNT;
    bool bound=false;
    uint32_t timeout;
    while (!bound) {
        NRF24L01_SetTxRxMode(TX_EN);
        XN297_Configure(
                _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_RF_BIND_CHANNEL);
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        NRF24L01_FlushTx();
        CX10_Write_Packet(0xAA); // send bind packet
        switch (current_protocol) {
            case PROTO_CX10_GREEN:
                delayMicroseconds(CX10_packet_period);
                if (counter==0)
                    bound = true;
                break;
            case PROTO_CX10_BLUE:
                delay(1);
                NRF24L01_SetTxRxMode(RX_EN);
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
                XN297_Configure(
                        _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | 
                        _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
                timeout = millis()+5;
                while (millis()<timeout) {

                    // data received from aircraft
                    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { 
                        XN297_ReadPayload(packet, CX10_packet_length);
                        if ( packet[9] == 0x01)
                        bound = true;
                        break;
                    }
                }
                break;
        }
        digitalWrite(LED_BUILTIN, counter-- & 0x10);
        if (ppm[AUX8] > PPM_MAX_COMMAND) {
            reset = true;
            return;
        }
    }
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); //start LED off

    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    set_txid(false);

    // Serial port input/output setup
    Serial.begin(115200);

    // reserve 200 bytes for the inputString:
    inputString.reserve(200);
}

void loop()
{
    // reset / rebind
    //Serial.println("begin loop");
    if (reset || ppm[AUX8] > PPM_MAX_COMMAND) {

        reset = false;

        Serial.println("selecting protocol");
        selectProtocol();        

        Serial.println("selected protocol.");
        NRF24L01_Reset();

        Serial.println("nrf24l01 reset.");
        NRF24L01_Initialize();

        Serial.println("nrf24l01 init.");
        CX10_init();
        CX10_bind();
        Serial.println("init protocol complete.");
    }

    uint32_t timeout = process_CX10(); // returns micros()+6000 for time to next packet. 

    // updates ppm values out of ISR
    //update_ppm();
    overrun_cnt=0;

    // Process string into tokens and assign values to ppm
    // The Arduino will also echo the command values that it assigned
    // to ppm
    if (stringComplete) {
        //Serial.println(inputString);
        // process string

        strcpy(c, inputString.c_str());
        p = strtok_r(c,",",&i); // returns substring up to first "," delimiter
        ppm_cnt=0;
        while (p !=0){
            //Serial.print(p);
            int val=strtol(p, &errpt, 10);
            if (!*errpt) {
                Serial.print(val);
                ppm[ppm_cnt]=val;
            }
            else

            // prints "x" if it could not decipher the command. Other
            // values in string may still be assigned.
            Serial.print("x"); 
            Serial.print(";"); // a separator between ppm values
            p = strtok_r(NULL,",",&i);
            ppm_cnt+=1;
        }
        Serial.println("."); // prints "." at end of command

        // clear the string:
        inputString = "";
        stringComplete = false;
    }

    // Read the string from the serial buffer
    while (Serial.available()) {
        // get the new byte:
        char inChar = (char)Serial.read();
        // if the incoming character is a newline, set a flag
        // so the main loop can do something about it:
        if (inChar == '\n') {
            stringComplete = true;
        }
        else {      
            // add it to the inputString:
            inputString += inChar;
        }

    }
    // wait before sending next packet
    while (micros() < timeout) // timeout for CX-10 blue = 6000microseconds. 
    {
        //overrun_cnt+=1;
    };
}
