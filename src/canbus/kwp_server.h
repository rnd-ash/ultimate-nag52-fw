#include <cstdint>
#include "iso_tp.h"

#define MAX_DATA_SIZE 4096

struct KwpPayload {
    uint16_t size;
    uint8_t buffer[MAX_DATA_SIZE];
};

class KwpDataHandler {
public:
    virtual bool recv_payload(KwpPayload* dest);
    virtual void send_payload(KwpPayload* tx);
};

class SerialDataHandler: public KwpDataHandler {

};

class BtDataHandler: public KwpDataHandler {

};

class IsoTpDataHandler: public KwpDataHandler {
public:
    IsoTpDataHandler(uint16_t rx_can_id, uint16_t tx_can_id);
    bool recv_payload(KwpPayload* dest);
    void send_payload(KwpPayload* tx);
private:

};

class KwpServer {
    KwpServer(KwpDataHandler* handler, uint16_t timeout_interval_ms);
    ~KwpServer();
private:
    KwpDataHandler* handler;
    // server_handler;
};