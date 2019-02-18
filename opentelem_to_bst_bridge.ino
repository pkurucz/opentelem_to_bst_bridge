/*
 * This is a Arduino project that handles converting
 * Eagletree Vector Open Telemetry serial data (57600 Baud)
 * to Team Blacksheep (TBS) BST (Blacksheep Telemetry)
 * on an I2C bus.
 */

/*
 * TODO:
 * - Make sure the artificial horizon display of the 
 *   Roll and pitch values when the attitude goes past 90 degrees is
 *   handled properly.
 * - Heading / vot_telemetry.GPSTelemetry.CourseDegrees Make sure this is scaled properly
 *   either in BST code or on the taranis
 * - Map the flight controller modes from vector->bst
 * - Add in support for extra Vector data throught the RC channels?
 */

#include "vector_open_telemetry.h"
#include "bst_telemetry.h"
#include "vector_open_telemetry.h"
#include "config.h"
#include "util.h"

/* ----------------------------------------------------- */


/* ----------------------------------------------------- */

// the setup routine runs once when you press reset:
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	LED_OFF();

	Serial.begin(57600, SERIAL_8N1);
	while (!Serial);             // Leonardo: wait for serial monitor
	Serial.println("\r\n\r\nOpentelemetry -> BST Bridge init ... ");
	
	/* Initialize the Blacksheep Telemetry (BST) layer */
	bst_init();
	
	/* Initialize the Eagletree Vector Open Telemetry layer */
	vot_init();
	
	/* Toggle the led to indicate setup complete */
	for(int i = 0; i < 3; i++) {
		LED_ON();
		delay(200);
		LED_OFF();
		delay(100);
	}
	Serial.println("Complete\r\n");

}

/* ----------------------------------------------------- */

// the loop routine runs over and over again forever:
void loop() {
	/* Run the Blacksheep Telemetry (BST) task */
	bst_handler_task();

  /* Run the Eagletree Vector Open Telemetry task */
	vot_handler_task();
}

/* ----------------------------------------------------- */

