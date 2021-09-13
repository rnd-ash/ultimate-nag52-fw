//
// Created by ashcon on 9/9/21.
//
#ifndef ULTIMATE_NAG52_FW_ISO_TP_H
#define ULTIMATE_NAG52_FW_ISO_TP_H

#include <cstdint>

class IsoTpServer {
public:
    IsoTpServer(uint16_t rx_id, uint16_t tx_id, uint8_t rx_mailbox_id, uint8_t bs, uint8_t st_min);
private:
    void create_server_task();
    uint8_t mailbox_id;
    uint8_t bs;
    uint8_t st_min;
    uint16_t tx_id;
};


#endif //ULTIMATE_NAG52_FW_ISO_TP_H
