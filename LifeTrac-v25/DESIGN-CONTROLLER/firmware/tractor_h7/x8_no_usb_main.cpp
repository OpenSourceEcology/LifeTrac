#include <Arduino.h>

#if defined(LIFETRAC_X8_NO_USB_SERIAL)

class USBPhy;

USBPhy *get_usb_phy()
{
    return nullptr;
}

void initVariant() __attribute__((weak));
void initVariant()
{
}

void setup();
void loop();

int main(void)
{
    init();
    initVariant();

    setup();

    for (;;) {
        loop();
    }

    return 0;
}

#endif