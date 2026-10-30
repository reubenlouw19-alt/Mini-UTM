# Mini-UTM

Benchtop ball-screw tension/compression test rig with closed-loop control: Stellenbosch skripsie 2026

A miniature universal testing machine for small tensile and compression specimens, built for the Materials Engineering Research Group (MATENG) at Stellenbosch University. A NEMA 23 stepper drives an SFU1204 ball screw. An S-type load cell measures force, and a linear potentiometer on the crosshead closes the displacement control loop. The rig is run from a browser dashboard over USB and is designed to be used together with Digital Image Correlation (DIC).

Mechanical Project 478 · Reuben Louw · Supervisor: Dr M Neaves

---

## Download the Required Software

- **Google Chrome or Microsoft Edge.** The dashboard talks to the rig through Web Serial, which only Chromium browsers support. Firefox and Safari won't work.
- **The dashboard**, in the [`dashboard`](dashboard) folder of this repo. Nothing needs installing: download the folder and open `dashboard-v3.html`. Keep the `assets` folder next to it, or the page loses its styling.
- **VS Code + the PlatformIO extension**, only if you need to re-flash the ESP32. The firmware is in [`Firmware`](Firmware).
- **A USB driver for the ESP32 board**, only if Windows doesn't show a COM port when you plug the rig in.

---

## Before Running the Rig

1. Plug the 20 V supply into the rig and switch on the master switch (and the fan switch).
2. Connect the ESP32 to your laptop with a USB cable. The ESP32 is powered over USB.
3. Release the emergency stop.
4. Check that neither end stop is pressed and the crosshead has room to travel in the test direction.


---

## How to Run a Test

The tutorial video below walks through a full tensile test, from connecting the rig to exporting the data.

https://github.com/user-attachments/assets/f8139a6f-b99d-4514-9027-6c3470473f7e



### Steps (as in the video)

1. **Connect.** Open `dashboard-v3.html` in Chrome or Edge, click **Connect rig**, pick the ESP32's USB serial port and press Connect. The header changes to *Rig online*. Connecting reboots the ESP32, and its boot report appears in the serial monitor.
2. **Zero with the specimen out.** Click **Zero load & position** before anything is gripped.
3. **Grip and pre-load.** Grip the specimen and jog until a small pre-load shows (about 10 N). This takes the slack out of the grips.
4. **Test setup.** Pick the standard, then enter the width, thickness and gauge length. The area fills in, the plot switches to stress–strain, and the crosshead speed is worked out from the standard. Leave closed-loop control on.
5. **Run the test.** Check that *Driver temperature*, *Linear pot* and *Load cell* all read OK, then click **Start tension test** (or **Start compression test**). The same button, or **Esc**, stops the test at any time.
6. **Speed up past yield** (E8 only). When the curve flattens out, press **Speed up** to switch to the post-yield speed. The dashboard ends the test by itself once the force drops below 15 % of the peak.
7. **Save the data.** **Download raw CSV** saves every sample plus the test settings. **Save graph as PDF** prints the plot. **Clear data** empties it for the next test.

---

## Dashboard Reference

| Panel | What it does |
|---|---|
| **Test setup** | Standard, specimen size and gauge length. Sets the test direction and crosshead speed, and turns closed-loop control on or off. |
| **Jog** | **Jog up/down** moves a set distance, and **Move up/down** runs until stopped. **Stop (Esc)** stops all motion. |
| **Force / Extension** | Live readings, Extension is the raw pot reading. |
| **Plot** | Stress–strain once the area and gauge length are filled in, otherwise force–extension. The x-axis has the machine's own stretch removed. |
| **Rig status** | Motion, control mode, tracking error, e-stop, end stops, driver temperature, pot and load cell. |
| **Results** | Peak force, peak stress (UTS), RMS and max tracking error, extension at peak and drive lost motion. |
| **Serial monitor** | Raw output from the ESP32, plus a command box. Type `HELP` for the full command list. |

### Test speeds

| Standard | Test | Crosshead speed | After yield |
|---|---|---|---|
| ASTM E8/E8M | Tension, metals | 0.015 /min × gauge length | 0.5 /min × gauge length (**Speed up**) |
| ASTM D638 | Tension, polymers | 5 mm/min | — |
| ASTM E9 | Compression, metals | 0.005 /min × gauge length | — |
| ASTM D695 | Compression, polymers | 1.3 mm/min | — |

The rig's speed range is 0.005–20 mm/min.

### CSV export

The header rows (starting with `#`) record the standard, specimen size, speeds, when **Speed up** was pressed, the control mode and the machine stiffness used. Below that is one row per 100 ms sample:

`time_s, extension_mm, force_N, commanded_mm, error_um, motor_mm_min, screw_deg, loop, motor_mm, lost_mm, specimen_ext_mm`

Force is positive in tension. `specimen_ext_mm` is the pot reading with the machine's stretch removed: `extension_mm − force_N / 1108`.

### Notes on the measurements

- The machine's measured stiffness is **1108 N/mm**, and the dashboard subtracts it from the extension. That correction is good enough for UTS and elongation, **but not for Young's modulus**: in the elastic region the rig stretches far more than the specimen does. Take modulus and strain from DIC.
- Closed-loop control keeps the crosshead on the set speed to within tens of µm. Open loop, the machine's stretch makes the real crosshead speed fall well below the set value.

---

## Repository Contents

| Folder | Contents |
|---|---|
| [`dashboard`](dashboard) | Browser dashboard (`dashboard-v3.html` + `assets`) |
| [`Firmware`](Firmware) | ESP32 firmware (PlatformIO, Arduino framework) |
| [`Electronics`](Electronics) | Circuit diagram |
| [`DIC Test Videos`](DIC%20Test%20Videos) | Videos of the DIC test runs |

### Firmware

Built with PlatformIO for an `esp32dev` board on the Arduino framework, with the libraries `teemuatlut/TMCStepper`, `adafruit/Adafruit ADS1X15` and `bogde/HX711`. Open the folder in VS Code with PlatformIO installed and click **Upload**. The serial monitor runs at 115200 baud. The sensor calibrations and the machine stiffness are constants at the top of `main.cpp`. If the stiffness is ever re-measured, change it there **and** in `dashboard-v3.html` (`MACHINE_K_N_MM`).

### Hardware

| Part | Component |
|---|---|
| Actuator | NEMA 23 stepper (57BYGH420), coupled directly to the screw |
| Driver | TMC2209 (Adafruit breakout), UART-configured, 256 microsteps |
| Screw | SFU1204 ball screw on FK10/FF10 supports, MGN9C linear rails |
| Load cell | PA6110 S-type, 50 kg, read by an HX711 |
| Displacement | Opkon SLPT-75 linear potentiometer, read by an ADS1115 (16-bit) |
| Controller | ESP32 dev board |
| Power | 20 V supply, MP4560 buck converter to 3.3 V |
