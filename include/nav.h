/*
    nav.h

    INS integration.

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

    float prev_roll = 0.0f;
    float prev_pitch = 0.0f;
    float prev_yaw = 0.0f;
    unsigned long prev_time = 0;

public:
    void init();
    void run();
    float roll;
    float pitch;
    float yaw;
    float p;
    float q;
    float r;
    float height;
    Nav(Sensors *sensors) : sensors(sensors){}
};

static int portWrite(unsigned int port, const unsigned char* buf, int len)
{
    return Serial2.write(buf, len);
}

/*
    Initialize INS
*/
inline void Nav::init()
{
    Serial.println("[NAV] Initializing INS...");
    Serial2.begin(921600);

    if (sizeof(double) != 8)
    {
        printf("[NAV] Error: INS requires 64 bit double support!\n");
        while (true){};
    }
    s_buffer = (uint8_t *)malloc(1024);
    if (!s_buffer)
    {
        printf("[NAV] Error: Failed to allocate INS buffer!\n");
        while (true) {};
    }

    // Initialize comm interface
    is_comm_init(&comm, s_buffer, 1024);
    
    // Configure INS
    int messageSize = is_comm_stop_broadcasts_all_ports(portWrite, 2, &comm);
    Serial2.write(comm.rxBuf.start, messageSize);
    messageSize = is_comm_get_data_to_buf(s_buffer, 1024, &comm, DID_INS_1, sizeof(ins_1_t), 0, 1);
    Serial2.write(comm.rxBuf.start, messageSize);

    // Initialize previous state
    prev_roll = 0.0f;
    prev_pitch = 0.0f;
    prev_yaw = 0.0f;
    prev_time = millis();

    Serial.println("[NAV] INS initialization complete!");
}

inline void Nav::run()
{
    sensors->updateUltrasonic();
    height = sensors->heightRaw;
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

inline void Nav::handleINSMessage(ins_1_t *ins)
{
    if (!ins) return;

    pitch = ins->theta[0] * C_RAD2DEG_F;
    roll = -ins->theta[1] * C_RAD2DEG_F;
    yaw = ins->theta[2] * C_RAD2DEG_F;

    // Serial.print("[DEBUG] Roll (deg): ");
    // Serial.println(roll);
    // Serial.print("[DEBUG] Pitch (deg): ");
    // Serial.println(pitch);
    // Serial.print("[DEBUG] Yaw (deg): ");
    // Serial.println(yaw);

    unsigned long current_time = millis();

    float delta_time = (current_time - prev_time) / 1000.0f;

    if (delta_time > 0.0f)
    {
        p = (roll - prev_roll) / delta_time;
        q = (pitch - prev_pitch) / delta_time;
        r = (yaw - prev_yaw) / delta_time;
    }
    else
    {
        p = q = r = 0.0f;
    }

    prev_roll = roll;
    prev_pitch = pitch;
    prev_yaw = yaw;
    prev_time = current_time;
}

#endif // NAV_H
