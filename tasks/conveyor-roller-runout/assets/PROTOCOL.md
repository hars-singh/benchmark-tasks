# Conveyor roller radial runout inspection

This station measures how far the surface of a conveyor roller moves toward and away from an indicator while the roller turns around its shaft. The trial records the difference between the largest and smallest readings. Conveyor inspection uses a fixed radial indicator above the middle of the roller face. The 0.40 mm limit comes from the example work order.

## Hardware and reset

- Install a steel tube roller with a 50 mm nominal outside diameter, 150 mm measuring face and 8 mm shaft. Mount the shaft horizontally on the rigid bench stand. Leave the measuring face clean. The two scripted controls use geometry with 0.10 mm and 0.70 mm peak-to-peak runout. In a physical replication, confirm each standard independently with the indicator.
- Fit a freely turning polymer crank handle with 16 mm diameter at a 35 mm radial offset. Mount the crank on the near side of the stand so the gripper can approach down the handle axis. Start the grip below the rotation axis. The crank rotates the measuring face and stays clear of the plunger.
- Mount a 12.7 mm travel indicator by its stem above the middle of the roller face. The spindle should be vertical. Set the plunger to stay in contact for a complete turn and connect its output to a maximum-minus-minimum readout. Keep the stand stable during measurement.
- Place the separate display and physical pass/fail switches beside the stand at the dimensions in `geometry.json`. Zero the displayed range and reset the measurement recorder between trials. Start both switches raised and both grippers open.

## Measurement

1. Turn the crank continuously through at least 360 degrees. Move slowly and keep the plunger on the roller. Use the highest and lowest readings from the same complete turn.
2. Press the green button if the range is 0.40 mm or less. Press the orange button if it is higher. Release the crank and clear the work area as the roller stops.
3. For a real trial, log the range from the indicator, the crank rotation from video or a shaft encoder, the pressed switch and the final rest. A separate measurement before the trial provides the reference classification. Check the mechanical limits against the fixture's reassembled digital geometry.

The CEMA procedure shows fixed midpoint measurement during manual rotation. Interroll documents concentric deviation and its inspection. A max/min indicator supports the actual range display. [CEMA measurement procedure](https://cemanet.org/wp-content/uploads/2020/08/Conveyor-Pulley-Comm-Will-lower-run-out-improve-my-conveyor-performance.pdf), [Interroll roller catalog](https://www.interroll.com/fileadmin/Downloads/Catalogs/Rollers/Conveyor_Roller_Catalog_EN.pdf), [Mitutoyo maximum/minimum indicator](https://dev.pim.mitutoyo.com/products/small-tool-instruments-and-data-management/indicators/digimatic-indicators/absolute-digimatic-max-min-indicator-id-c/)
