#include "humanoid_hardware/udp_communication.h"

UDPCommunication::UDPCommunication(const std::string& ip, int statePort, int cmdPort) 
{
    cmdSock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&rpiAddr_, 0, sizeof(rpiAddr_));
    rpiAddr_.sin_family = AF_INET;
    rpiAddr_.sin_port   = htons(cmdPort);
    inet_pton(AF_INET, ip.c_str(), &rpiAddr_.sin_addr);

    stateSock_ = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&jetsonAddr_, 0, sizeof(jetsonAddr_));
    jetsonAddr_.sin_family      = AF_INET;
    jetsonAddr_.sin_addr.s_addr = INADDR_ANY;
    jetsonAddr_.sin_port        = htons(statePort);
    bind(stateSock_, (struct sockaddr*)&jetsonAddr_, sizeof(jetsonAddr_));

    struct timeval tv{0,10000};
    setsockopt(stateSock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

UDPCommunication::~UDPCommunication() 
{
    if (stateSock_ >= 0) close(stateSock_);
    if (cmdSock_ >= 0) close(cmdSock_);
}

uint16_t UDPCommunication::crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

bool UDPCommunication::waitReadySignal()
{
    char buffer[128] = {0};
    sockaddr_in sender_addr{};
    socklen_t sender_len = sizeof(sender_addr);

    std::cout << "[WAIT] Waiting for READY signal from RPi...\n";

    while (true)
    {
        ssize_t len = recvfrom(stateSock_, buffer, sizeof(buffer) - 1, 0,
                               (struct sockaddr*)&sender_addr, &sender_len);

        if (len > 0)
        {
            buffer[len] = '\0';

            if (std::string(buffer) == "READY")
            {
                std::cout << "[WAIT] Received READY from "
                          << inet_ntoa(sender_addr.sin_addr) << "\n";
                return true;
            }
            else
            {
                std::cout << "[WAIT] Received unexpected message: " << buffer << "\n";
            }
        }
        else
        {
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            else
            {
                perror("[WAIT] recvfrom error");
                return false;
            }
        }
    }

    return false;
}


bool UDPCommunication::sendCommand(const MotorCommand* commands, size_t num_motor)
{

    size_t payload_size = sizeof(MotorCommand) * num_motor;
    size_t packet_size = 1 + 2 + payload_size + 2 + 1;

    uint8_t* packet = new uint8_t[packet_size];
    size_t offset = 0;

    packet[offset++] = START_BYTE;
    packet[offset++] = payload_size & 0xFF;
    packet[offset++] = (payload_size >> 8) & 0xFF;

    std::memcpy(&packet[offset], commands, payload_size);
    offset += payload_size;

    uint16_t crc = crc16(&packet[1], 2 + payload_size);
    packet[offset++] = crc & 0xFF;
    packet[offset++] = (crc >> 8) & 0xFF;

    packet[offset++] = STOP_BYTE;

    sendto(cmdSock_, packet, packet_size, 0, (struct sockaddr*)&rpiAddr_, sizeof(rpiAddr_));

    delete[] packet;
    return true;
}

bool UDPCommunication::readState(MotorState* states, size_t num_motor)
{
    size_t expected_payload_size = sizeof(MotorState) * num_motor;
    size_t expected_packet_size = 1 + 2 + expected_payload_size + 2 + 1;

    uint8_t* packet = new uint8_t[expected_packet_size];
    ssize_t len = recvfrom(stateSock_, packet, expected_packet_size, 0, nullptr, nullptr);

    if (len <= 0) {
        delete[] packet;
        return false;
    }

    size_t offset = 0;

    if (packet[offset++] != START_BYTE) {
        std::cerr << "[ERROR] Invalid start byte\n";
        delete[] packet;
        return false;
    }

    uint16_t payload_len = packet[offset++];
    payload_len |= (packet[offset++] << 8);

    if (payload_len != expected_payload_size) {
        std::cerr << "[ERROR] Payload length mismatch\n";
        delete[] packet;
        return false;
    }

    std::memcpy(states, &packet[offset], payload_len);
    offset += payload_len;

    uint16_t received_crc = packet[offset++];
    received_crc |= (packet[offset++] << 8);

    uint16_t computed_crc = crc16(&packet[1], 2 + payload_len);

    if (received_crc != computed_crc) {
        std::cerr << "[ERROR] CRC mismatch\n";
        delete[] packet;
        return false;
    }

    if (packet[offset] != STOP_BYTE) {
        std::cerr << "[ERROR] Invalid stop byte\n";
        delete[] packet;
        return false;
    }

    delete[] packet;
    return true;
}

bool UDPCommunication::checkStart(uint16_t* motor_id, size_t num_motor) 
{
    constexpr int MAX_ATTEMPTS = 10;
    constexpr int TIMEOUT_MS = 500;

    HandshakeMessage msg;
    msg.num_motors = num_motor;
    std::memcpy(msg.motor_ids, motor_id, num_motor * sizeof(uint16_t));

    std::cout << "[HANDSHAKE] Sending handshake..." << std::endl;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = TIMEOUT_MS * 1000;
    setsockopt(cmdSock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
        sendto(cmdSock_, &msg, sizeof(msg), 0, (sockaddr*)&rpiAddr_, sizeof(rpiAddr_));

        HandshakeResponse response;
        socklen_t addrlen = sizeof(rpiAddr_);
        ssize_t len = recvfrom(cmdSock_, &response, sizeof(response), 0, (sockaddr*)&rpiAddr_, &addrlen);

        if (len == sizeof(response)) {
            if (response == HandshakeResponse::OK) {
                std::cout << "[HANDSHAKE] Success.\n";
                return true;
            } else {
                std::cerr << "[HANDSHAKE] RPI rejected motor configuration.\n";
                return false;
            }
        } else if (len < 0) {
            perror("[HANDSHAKE] recvfrom error");
        } else {
            std::cerr << "[HANDSHAKE] Unexpected response length: " << len << "\n";
        }

        std::cerr << "[HANDSHAKE] No valid response. Retrying (" << attempt + 1 << "/" << MAX_ATTEMPTS << ")...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_MS));
    }

    std::cerr << "[HANDSHAKE] Failed after " << MAX_ATTEMPTS << " attempts.\n";
    return false;

}

int UDPCommunication::getCmdSocket() const 
{
    return cmdSock_;
}

int UDPCommunication::getStateSocket() const 
{
    return stateSock_;
}
