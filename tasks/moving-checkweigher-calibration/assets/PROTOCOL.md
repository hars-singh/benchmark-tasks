# Moving checkweigher calibration

This bench procedure follows the sequence used to calibrate an in-motion checkweigher. First capture zero with an empty pan, then place a known reference load to set span. A separate test load verifies the resulting calibration. [Thompson Scale calibration manual, section 3.3](https://thompsonscale.com/support/users-manuals/checkweighers/tsc-350-checkweigher/7-tsc-350-checkweigher-manual-ver-8-9/file)

## Set up and reset

Mount a 98 × 84 mm weighing pan on a guided carriage. Attach a load cell or force sensor between pan and carriage. Use the body positions and dimensions in [the geometry specification](geometry.json). The carriage travels along Y over 80 mm with a 7 s period. Start empty at mid-stroke, moving toward positive Y. Set the console to its uncalibrated state and put the 300 g reference and the separately measured 195 g verification pack in their starting locations.

Record carriage position, vertical load, console inputs and the robot from before the first calibration action. For each measurement, use the stated interval during which the carrier is moving, the robot's hands are clear and the pan supports the entire load. Use the blue control to capture the empty reading, yellow for the reference, and green to verify the separate pack. Check that the reference finishes back at rest near its start while the pack remains on the pan. The simulator represents the load cell with a linear gain and an offset applied to actual contact force. The score covers the zero and span procedure and the separate verification.
