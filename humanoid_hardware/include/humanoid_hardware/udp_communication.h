#ifndef UDP_COMMUNICATION_H
#define UDP_COMMUNICATION_H

#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cstdint>
#include <cstddef>
#include <thread>
#include <chrono>

#define START_BYTE 0xAA
#define STOP_BYTE  0xBB

#pragma pack(push, 1)
struct MotorState
{
    uint16_t drive_ids;                 // Motor ID
    float    position;                  // Position
    float    velocity;                  // Velocity
    float    torque;                    // Torque
    uint8_t  temperature;               // Temperature
    uint16_t quickStatus;               // Status
};
#pragma pack(pop)

#pragma pack(push, 1)
struct MotorCommand {
    uint16_t drive_ids;                 // Motor ID
    float    kp;                        // P Gain
    float    kd;                        // D Gain
    float    target_position;           // Target Position
    float    target_velocity;           // Target Velocity
    float    target_torque;             // Target Torque
};
#pragma pack(pop)

#define MAX_MOTOR_NUM 6

#pragma pack(push, 1)
struct HandshakeMessage 
{
    uint16_t num_motors;
    uint16_t motor_ids[MAX_MOTOR_NUM];
};
#pragma pack(pop)

enum class HandshakeResponse : uint8_t {
    OK = 1,
    ERROR = 0
};


class UDPCommunication
{
    public:
        UDPCommunication(const std::string& jetsonIp, int statePort, int cmdPort);
        ~UDPCommunication();
        
        bool sendCommand(const MotorCommand* commands, size_t num_motor);
        bool readState(MotorState* states, size_t num_motor);
        bool checkStart(uint16_t* motor_id, size_t num_motor);
        bool waitReadySignal();

        int getCmdSocket() const;
        int getStateSocket() const;

    private:
        uint16_t crc16(const uint8_t *data, size_t len);
        
        int                 stateSock_    = -1;
        int                 cmdSock_      = -1;
        struct sockaddr_in  jetsonAddr_;
        struct sockaddr_in  rpiAddr_;
};

#endif // UDP_COMMUNICATION_H