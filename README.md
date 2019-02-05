# opentelem_to_bst_bridge
Arduino project to bridge Eagletree Vector Open Telemetry (Serial 57600 baud) -> TBS BST (Blacksheep telemetry) (I2C)

In an idea world this project wouldn't be necessary and the Eagletree Vector would natively support CRSF
protocol (Serial 420000 baud 8N1).  Alas, this project attempts to get all of the telemetry data in
the vector exported back to the crossfire radio link to make it available to the Taranis radio
for lua scripts and the like.

---

Future Features:
	- Standalone mode (non-vector mode) for simple models without AP
		- NEMA GPS support
		- Analog voltage and Current sensor support
	- WS2812 LED strobes

---

Eagletree, my offer still stands to implement CRSF in the vector codebase pro-bono.  Would be willing to sign a NDA if necessary.

~pk

