#include "endpoint.h"
#include "tcu_alloc.h"

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
        this->read_buffer = static_cast<char *>(TCU_HEAP_ALLOC(UART_MSG_SIZE));
        this->write_buffer = static_cast<char *>(TCU_HEAP_ALLOC(UART_MSG_SIZE));
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
    this->write_buffer[(len * 2) + 5] = '\n';
    uart_write_bytes(UART_PORT, &this->write_buffer[0], (len * 2) + 6);
}

bool UsbEndpoint::read_data(DiagMessage *dest)
{
    this->length = 0;
    uart_get_buffered_data_len(UART_PORT, &length);
    if (length != 0)
    {
        max_bytes_left = UART_MSG_SIZE - this->read_pos;
        to_read = MIN(length, max_bytes_left);
        uart_read_bytes(UART_PORT, &this->read_buffer[this->read_pos], to_read, 0);
        this->read_pos += length;
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
            uint16_t read_size = (this->read_buffer[0] << 8) | this->read_buffer[1];
            if (read_size != this->read_pos - 2)
            {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg. Msg size is %d bytes, buffer has %d bytes", read_size, this->read_pos - 2);
                this->read_pos = 0;
                return false;
            }
            else
            {
                // Valid msg!
                dest->id = (this->read_buffer[2] << 8) | this->read_buffer[3];
                dest->data_size = read_size - 2;
                memcpy(dest->data, &this->read_buffer[4], dest->data_size);
                this->read_pos = 0;
                return true;
            }
        }
    }
    return false;
}