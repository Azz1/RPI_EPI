#include <Arduino.h>

int main(void)
{
#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
	}
        
	return 0;
}

