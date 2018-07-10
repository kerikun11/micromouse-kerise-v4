#include "Arduino.h"
#include "Machine.h"

extern "C" void app_main()
{
    initArduino();
    setup();
    for(;;) {
        loop();
    }
}
