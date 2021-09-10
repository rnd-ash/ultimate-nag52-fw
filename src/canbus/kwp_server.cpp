#include "kwp_server.h"

KwpServer::~KwpServer() {
    vTaskDelete(this->server_handler);
}

KwpServer::KwpServer(KwpDataHandler *handler, uint16_t timeout_interval_ms) {
    this->handler = handler;

}
