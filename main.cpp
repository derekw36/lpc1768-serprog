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

BusOut leds(LED4, LED3, LED2, LED1);
SPI spi(MBED_SPI0, use_gpio_ssel);
USBSerial ser(false);

int main(void)
{
    printf("%s up and running!\n", SERPROG_NAME);
    ser.connect();

    while(1)
    {
        leds = 0x00;
        ser.wait_ready();
        int cmd = ser.getc();

        switch(cmd)
        {
            case S_CMD_NOP:
            {
                ser.putc(S_ACK);
                break;
            }

            case S_CMD_Q_IFACE:
            {
                static const uint16_t ver = S_IFACE_VER;

                ser.putc(S_ACK);
                ser.write(&ver, 2);
                break;
            }

            case S_CMD_Q_CMDMAP:
            {
                static const uint32_t cmds = SERPROG_SUPP_CMDS;
                uint8_t map[32] = {0};
                memcpy(map, &cmds, sizeof(cmds));

                ser.putc(S_ACK);
                ser.write(map, 32);
                break;
            }

            case S_CMD_Q_PGMNAME:
            {
                char name[16] = {0};
                strncpy(name, SERPROG_NAME, sizeof(name));

                ser.putc(S_ACK);
                ser.write(name, 16);
                break;
            }

            case S_CMD_Q_SERBUF:
            {
                static const uint16_t sz = 0xFFFF;

                ser.putc(S_ACK);
                ser.write(&sz, 2);
                break;
            }

            case S_CMD_Q_BUSTYPE:
            {
                ser.putc(S_ACK);
                ser.putc(BUS_SPI);
                break;
            }

            case S_CMD_SYNCNOP:
            {
                ser.putc(S_NAK);
                ser.putc(S_ACK);
                break;
            }

            case S_CMD_S_BUSTYPE:
            {
                int bus = ser.getc();

                if(bus == BUS_SPI)
                {
                    // We are always SPI! Just ACK it
                    ser.putc(S_ACK);
                }
                else
                {
                    ser.putc(S_NAK);
                }
                break;
            }

            case S_CMD_O_SPIOP:
            {
                static uint8_t buf[4096];
                ssize_t bytes = 0;
                size_t total = 0;
                uint32_t slen = 0;
                uint32_t rlen = 0;

                ser.read(&slen, 3);
                ser.read(&rlen, 3);

                spi.select();

                // transfer from ser to spi in chunks
                leds = 0x01;
                while(total < slen)
                {
                    leds = (leds == 0x08) ? 0x01 : leds << 1;
                    bytes = ser.read(buf, MIN(slen, sizeof(buf)));
                    if(bytes > 0)
                    {
                        spi.write(buf, bytes, NULL, 0);
                        total += bytes;
                    }
                    else if(bytes < 0)
                    {
                        break;
                    }
                }

                // transfer from spi to ser in chunks
                bytes = 0, total = 0;
                leds = 0x08;
                ser.putc(S_ACK);
                while(total < rlen)
                {
                    leds = (leds == 0x01) ? 0x08 : leds >> 1;
                    bytes = spi.write(NULL, 0, buf, MIN(rlen, sizeof(buf)));
                    if(bytes > 0)
                    {
                        if(ser.write(buf, bytes) < 0)
                        {
                            break;
                        }
                        total += bytes;
                    }
                }

                spi.deselect();
                break;
            }

            case S_CMD_S_SPI_FREQ:
            {
                uint32_t freq = 0;
                ser.read(&freq, 4);

                if(freq != 0)
                {
                    spi.frequency((int)freq);

                    ser.putc(S_ACK);
                    ser.write(&freq, 4);
                }
                else
                {
                    ser.putc(S_NAK);
                }
                break;
            }

            default:
            {
                ser.putc(S_NAK);
                break;
            }
        }
    }
}
