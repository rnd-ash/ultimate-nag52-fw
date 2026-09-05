#include "endpoint.h"

const static char HEX_DEF[17] = "0123456789ABCDEF";
const static size_t UART_MSG_SIZE = 6 + (2 * DIAG_CAN_MAX_SIZE);
const uart_port_t UART_PORT = uart_port_t::UART_NUM_0;

UsbEndpoint::UsbEndpoint() : AbstractEndpoint()
{
    data_size = 0;
    line_idx = 0;
    max_bytes_left = 0;
    to_read = 0;
    length = 0;
    this->status = uart_driver_install(UART_PORT, 512, 512, 0, nullptr, 0);
    if (this->status == ESP_OK)
    {
        this->read_buffer = static_cast<char *>(malloc(UART_MSG_SIZE));
        this->write_buffer = static_cast<char *>(malloc(UART_MSG_SIZE));
        if (nullptr != this->read_buffer && nullptr != this->write_buffer)
        {
            uart_flush(UART_PORT);
            this->read_pos = 0;
        } else {
            this->status = ESP_ERR_NO_MEM;
        }
    }
}

esp_err_t UsbEndpoint::init_state() {
    return this->status;
}

void UsbEndpoint::send_data(uint32_t id, uint8_t *buf, uint16_t len)
{
    if (this->status != ESP_OK || this->write_buffer == nullptr || buf == nullptr) {
        return;
    }
    if (len > DIAG_CAN_MAX_SIZE) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "USBEndpoint", "send_data len %u exceeds max %u, truncating", len, DIAG_CAN_MAX_SIZE);
        len = DIAG_CAN_MAX_SIZE;
    }
    this->write_buffer[0] = '#';
    this->write_buffer[1] = HEX_DEF[(id >> 12) & 0x0F];
    this->write_buffer[2] = HEX_DEF[(id >> 8) & 0x0F];
    this->write_buffer[3] = HEX_DEF[(id >> 4) & 0x0F];
    this->write_buffer[4] = HEX_DEF[id & 0x0F];
    for (uint16_t i = 0; i < len; i++)
    {
        this->write_buffer[5 + (i * 2)] = HEX_DEF[(buf[i] >> 4) & 0x0F];
        this->write_buffer[6 + (i * 2)] = HEX_DEF[buf[i] & 0x0F];
    }
    const size_t tx_len = (static_cast<size_t>(len) * 2u) + 6u;
    if (tx_len > UART_MSG_SIZE) {
        ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Encoded UART payload too large (%u)", (unsigned int)tx_len);
        return;
    }
    this->write_buffer[(len * 2) + 5] = '\n';
    uart_write_bytes(UART_PORT, &this->write_buffer[0], tx_len);
}

bool UsbEndpoint::read_data(DiagMessage *dest)
{
    if (this->status != ESP_OK || dest == nullptr || this->read_buffer == nullptr) {
        return false;
    }
    this->length = 0;
    uart_get_buffered_data_len(UART_PORT, &length);
    if (length != 0)
    {
        max_bytes_left = UART_MSG_SIZE - static_cast<size_t>(this->read_pos);
        to_read = MIN(length, max_bytes_left);
        uart_read_bytes(UART_PORT, &this->read_buffer[this->read_pos], static_cast<uint32_t>(to_read), 0);
        this->read_pos += static_cast<uint16_t>(to_read);
        return false;
    }
    else if (this->read_pos != 0)
    {
        if (this->read_pos < 5)
        {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg. Less than 5 bytes");
            this->read_pos = 0;
            return false;
        }
        else
        {
            uint16_t read_size = (static_cast<uint8_t>(this->read_buffer[0]) << 8) | static_cast<uint8_t>(this->read_buffer[1]);
            if (read_size != this->read_pos - 2)
            {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg. Msg size is %d bytes, buffer has %d bytes", read_size, this->read_pos - 2);
                this->read_pos = 0;
                return false;
            }
            if (read_size < 2 || read_size > DIAG_CAN_MAX_SIZE + 2) {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg size %d", read_size);
                this->read_pos = 0;
                return false;
            }
            else
            {
                // Valid msg!
                dest->id = (static_cast<uint8_t>(this->read_buffer[2]) << 8) | static_cast<uint8_t>(this->read_buffer[3]);
                dest->data_size = read_size - 2;
                memcpy(dest->data, &this->read_buffer[4], dest->data_size);
                this->read_pos = 0;
                return true;
            }
        }
    }
    return false;
}