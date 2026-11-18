# Two-stop conveyor feeding fixture

Sequential stops isolate one workpiece from a continuous feed and release it to the next station. This bench fixture uses two manually operated, spring-return rockers. The automation equivalent uses interlocked pins or dedicated accumulation zones. [SMC two-finger escapement](https://www.smcusa.com/products/miw-escapement-2-finger-type~20339). [Interroll singulated release](https://www.interroll.com/fileadmin/user_upload/Downloads__PDF_/User_Manuals/Controls/en_ZoneControl_V1_en_11-01-12.pdf).

## Hardware and reset

- Mount a flat 118 mm conveyor with the guide rails, rocker blocks and end stop positioned as shown in `geometry.json`. The SCAD model gives the nominal envelope and center positions. Use a functional rubber conveyor, real pivots and return springs, and rigid guides.
- Check that each 44 mm puck slides in the 52 mm lane and the narrow stop can return between neighboring bases. Fit the rocker springs so each stop returns to the blocking position under a queued load.
- Load the three pucks at the documented starting positions. Start both levers released and the belt at 70 mm/s. A visible outlet line may be drawn 90 mm past the orange pivot for physical evaluation.

## Trial and scoring

1. Press the blue lever until one puck enters the pocket between the two gates. Release blue and confirm that it blocks the next puck.
2. Press the orange lever to release the isolated puck. Let orange return before operating blue again. Repeat until the third puck passes the outlet line and all three rest against the downstream stop.
3. Record the gate and workpiece passages from a fixed camera. Use a ruler or feeler gauge to mark the two clearance limits on the lever travel before the trial. A fully open stop clears the top of a puck by 1 mm or more. A blocking stop reaches at least 3 mm below its top. The output camera records the workpiece spacing, outlet crossings and final rest.
