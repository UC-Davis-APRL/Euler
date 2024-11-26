/*
    nav.h

    Fuses raw sensor data into stable navigation values

    @authors Orfeas Magoulas
*/
#ifndef NAV_H
#define NAV_H
#include "ISComm.h"
#include <stddef.h>
#include <sensors.h>

class Nav
{
private:
    Sensors *sensors;
    uint8_t *s_buffer = nullptr;
    is_comm_instance_t comm;
    void handleINSMessage(ins_1_t *ins);
public:
    void init();
    void run();
    float pitch;
    float yaw;
    float roll;
    float height;
    Nav(Sensors *sensors) : sensors(sensors){}
};

static int portWrite(unsigned int port, const unsigned char* buf, int len)
{
    return Serial2.write(buf, len);
}

inline void Nav::handleINSMessage(ins_1_t *ins)
{
    if (!ins) return;
    roll = ins->theta[0] * C_RAD2DEG_F;
    pitch = ins->theta[1] * C_RAD2DEG_F;
    yaw = ins->theta[2] * C_RAD2DEG_F;
}

inline void Nav::init()
{
    Serial2.begin(921600);

    s_buffer = (uint8_t *)malloc(1024);
    if (!s_buffer)
    {
        Serial.println("[NAV] Failed to allocate INS buffer!");
        while (true) {};
    }

    Serial.println("[NAV] Initializing INS");

    is_comm_init(&comm, s_buffer, 1024);

    int messageSize = is_comm_stop_broadcasts_all_ports(portWrite, 2, &comm);
    Serial2.write(comm.rxBuf.start, messageSize);

    messageSize = is_comm_get_data_to_buf(s_buffer, 1024, &comm, DID_INS_1, sizeof(ins_1_t), 0, 1);
    Serial2.write(comm.rxBuf.start, messageSize);

    Serial.println("[NAV] INS Initialized!");
}

inline void Nav::run()
{
    sensors->updateUltrasonic();
    height=sensors->heightRaw;
    if (Serial2.available())
    {
        uint8_t inByte = Serial2.read();
        uint32_t message_type = is_comm_parse_byte(&comm, inByte);
        switch (message_type)
        {
        case _PTYPE_INERTIAL_SENSE_DATA:
            switch (comm.rxPkt.dataHdr.id)
            {
            case DID_INS_1:
                if (comm.rxPkt.data.ptr)
                {
                    handleINSMessage((ins_1_t *)(comm.rxPkt.data.ptr));
                }
                else
                {
                    Serial.println("Invalid data pointer");
                }
                break;
            default:
                Serial.print("Unexpected message DID: ");
                Serial.println(message_type, DEC);
            }
            break;
        default:
            break;
        }
    }
}

#endif // NAV_H