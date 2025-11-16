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
SPI spi(MBED_SPI0, use_gpio_ssel);

// Extend the USBSerial class to handle serprog commands
class SerProg : private USBSerial
{
    SPI *spi;
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
            rbytes = read(buf, MIN(len, sizeof(buf)));
            if(rbytes < 0)
            {
                break;
            }

            wbytes = spi->write(buf, rbytes, NULL, 0);
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
            rbytes = spi->write(NULL, 0, buf, MIN(len, sizeof(buf)));
            if(rbytes < 0)
            {
                break;
            }

            wbytes = write(buf, rbytes);
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
    SerProg(SPI *spi) : USBSerial(false), spi(spi)
    {
        // Configure SPI bus before connecting USB
        this->spi->format(8, 0);
        this->spi->set_default_write_value(0x00);

        // Connect USB
        connect();
    }

    int transact(void)
    {
        // Wait until serprog connects to us
        wait_ready();

        switch(_getc())
        {
            case S_CMD_NOP:
            {
                _putc(S_ACK);
                break;
            }

            case S_CMD_Q_IFACE:
            {
                static const uint16_t ver = S_IFACE_VER;

                _putc(S_ACK);
                write(&ver, 2);
                break;
            }

            case S_CMD_Q_CMDMAP:
            {
                static const uint32_t cmds = SERPROG_SUPP_CMDS;
                uint8_t map[32] = {0};
                memcpy(map, &cmds, sizeof(cmds));

                _putc(S_ACK);
                write(map, 32);
                break;
            }

            case S_CMD_Q_PGMNAME:
            {
                char name[16] = {0};
                strncpy(name, SERPROG_NAME, sizeof(name));

                _putc(S_ACK);
                write(name, 16);
                break;
            }

            case S_CMD_Q_SERBUF:
            {
                static const uint16_t sz = 0xFFFF;

                _putc(S_ACK);
                write(&sz, 2);
                break;
            }

            case S_CMD_Q_BUSTYPE:
            {
                _putc(S_ACK);
                _putc(BUS_SPI);
                break;
            }

            case S_CMD_SYNCNOP:
            {
                _putc(S_NAK);
                _putc(S_ACK);
                break;
            }

            case S_CMD_S_BUSTYPE:
            {
                int bus = _getc();

                if(bus == BUS_SPI)
                {
                    // We are always SPI! Just ACK it
                    _putc(S_ACK);
                }
                else
                {
                    _putc(S_NAK);
                }
                break;
            }

            case S_CMD_O_SPIOP:
            {
                uint32_t slen = 0;
                uint32_t rlen = 0;

                read(&slen, 3);
                read(&rlen, 3);

                spi->select();

                ser_to_spi(slen);
                _putc(S_ACK);
                spi_to_ser(rlen);

                spi->deselect();
                break;
            }

            case S_CMD_S_SPI_FREQ:
            {
                uint32_t freq = 0;
                read(&freq, 4);

                if(freq != 0)
                {
                    spi->frequency((int)freq);

                    _putc(S_ACK);
                    write(&freq, 4);
                }
                else
                {
                    _putc(S_NAK);
                }
                break;
            }

            default:
            {
                _putc(S_NAK);
                break;
            }
        }

        return 0;
    }
};

int main(void)
{
    SerProg sp(&spi);

    printf("%s up and running!\n", SERPROG_NAME);
    while(1)
    {
        sp.transact();
    }

    return 0;
}
