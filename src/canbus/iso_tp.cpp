//
// Created by ashcon on 9/9/21.
//

#include "iso_tp.h"

IsoTpServer::IsoTpServer(uint16_t rx_id, uint16_t tx_id, uint8_t rx_mailbox_id, uint8_t bs, uint8_t st_min) {
    CAN0.setRXFilter(rx_mailbox_id, rx_id, 0xFFFF, false);
    this->mailbox_id = rx_mailbox_id;
    this->bs = bs;
    this->st_min = st_min;
    this->tx_id = tx_id;
}
