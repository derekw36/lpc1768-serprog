#include <stdint.h>

#include "mbed.h"
#include "USBSerial.h"

#include "serprog.h"

#define SERPROG_NAME "lpc1768-serprog"
#define SERPROG_SUPP_CMDS \
( \
    (1 << S_CMD_NOP) | \
    (1 << S_CMD_Q_IFACE) | \
    (1 << S_CMD_Q_CMDMAP) | \
    (1 << S_CMD_Q_PGMNAME) | \
    (1 << S_CMD_Q_SERBUF) | \
    (1 << S_CMD_Q_BUSTYPE) | \
    (1 << S_CMD_SYNCNOP) | \
    (1 << S_CMD_S_BUSTYPE) | \
    (1 << S_CMD_O_SPIOP) | \
    (1 << S_CMD_S_SPI_FREQ) \
)

#define MIN_MAX(a, b, OP) \
({ \
    typeof(a) _a = (a); \
    typeof(b) _b = (b); \
    _a OP _b ? _a : _b; \
})
#define MIN(a, b) MIN_MAX(a, b, <)
#define MAX(a, b) MIN_MAX(a, b, >)

// Peripheral setup
BusOut leds(LED4, LED3, LED2, LED1);

// Extend the USBSerial class to handle serprog commands
class SerProg : private SPI, USBSerial
{
    uint8_t buf[4096];

    ssize_t ser_to_spi(size_t len)
    {
        ssize_t total = 0;
        ssize_t rbytes = 0;
        ssize_t wbytes = 0;

        // transfer from ser to spi in chunks
        leds = 0x01;
        while(total < (ssize_t)len)
        {
            rbytes = USBSerial::read(buf, MIN(len, sizeof(buf)));
            if(rbytes < 0)
            {
                break;
            }

            wbytes = SPI::write(buf, rbytes, NULL, 0);
            if(wbytes < 0)
            {
                break;
            }

            total += wbytes;
            leds = (leds == 0x08) ? 0x01 : leds << 1;
        }

        return total;
    }

    ssize_t spi_to_ser(size_t len)
    {
        ssize_t total = 0;
        ssize_t rbytes = 0;
        ssize_t wbytes = 0;

        // transfer from spi to ser in chunks
        leds = 0x08;
        while(total < (ssize_t)len)
        {
            rbytes = SPI::write(NULL, 0, buf, MIN(len, sizeof(buf)));
            if(rbytes < 0)
            {
                break;
            }

            wbytes = USBSerial::write(buf, rbytes);
            if(wbytes < 0)
            {
                break;
            }

            total += wbytes;
            leds = (leds == 0x01) ? 0x08 : leds >> 1;
        }

        return total;
    }

public:
    SerProg() : SPI(MBED_SPI0, use_gpio_ssel), USBSerial(false)
    {
        // Configure SPI bus before connecting USB
        SPI::format(8, 0);
        SPI::set_default_write_value(0x00);

        // Connect USB
        USBSerial::connect();
    }

    int transact(void)
    {
        // Wait until serprog connects to us
        USBSerial::wait_ready();

        switch(USBSerial::_getc())
        {
            case S_CMD_NOP:
            {
                USBSerial::_putc(S_ACK);
                break;
            }

            case S_CMD_Q_IFACE:
            {
                static const uint16_t ver = S_IFACE_VER;

                USBSerial::_putc(S_ACK);
                USBSerial::write(&ver, 2);
                break;
            }

            case S_CMD_Q_CMDMAP:
            {
                static const uint32_t cmds = SERPROG_SUPP_CMDS;
                uint8_t map[32] = {0};
                memcpy(map, &cmds, sizeof(cmds));

                USBSerial::_putc(S_ACK);
                USBSerial::write(map, 32);
                break;
            }

            case S_CMD_Q_PGMNAME:
            {
                char name[16] = {0};
                strncpy(name, SERPROG_NAME, sizeof(name));

                USBSerial::_putc(S_ACK);
                USBSerial::write(name, 16);
                break;
            }

            case S_CMD_Q_SERBUF:
            {
                static const uint16_t sz = 0xFFFF;

                USBSerial::_putc(S_ACK);
                USBSerial::write(&sz, 2);
                break;
            }

            case S_CMD_Q_BUSTYPE:
            {
                USBSerial::_putc(S_ACK);
                USBSerial::_putc(BUS_SPI);
                break;
            }

            case S_CMD_SYNCNOP:
            {
                USBSerial::_putc(S_NAK);
                USBSerial::_putc(S_ACK);
                break;
            }

            case S_CMD_S_BUSTYPE:
            {
                int bus = _getc();

                if(bus == BUS_SPI)
                {
                    // We are always SPI! Just ACK it
                    USBSerial::_putc(S_ACK);
                }
                else
                {
                    USBSerial::_putc(S_NAK);
                }
                break;
            }

            case S_CMD_O_SPIOP:
            {
                uint32_t slen = 0;
                uint32_t rlen = 0;

                USBSerial::read(&slen, 3);
                USBSerial::read(&rlen, 3);

                SPI::select();

                ser_to_spi(slen);
                USBSerial::_putc(S_ACK);
                spi_to_ser(rlen);

                SPI::deselect();
                break;
            }

            case S_CMD_S_SPI_FREQ:
            {
                uint32_t freq = 0;
                USBSerial::read(&freq, 4);

                if(freq != 0)
                {
                    SPI::frequency((int)freq);

                    USBSerial::_putc(S_ACK);
                    USBSerial::write(&freq, 4);
                }
                else
                {
                    USBSerial::_putc(S_NAK);
                }
                break;
            }

            default:
            {
                USBSerial::_putc(S_NAK);
                break;
            }
        }

        return 0;
    }
};

int main(void)
{
    SerProg sp;

    printf("%s up and running!\n", SERPROG_NAME);
    while(1)
    {
        sp.transact();
    }

    return 0;
}
