# Moving minimum-bore inspection

Use a smooth 12 mm Go plug to distinguish a bore that admits the plug from an undersize bore. A Go plug checks the lower size limit, so this two-sample exercise grades the minimum-diameter check. [Vermont Gage application guidance](https://www.vermontgage.com/support/detail/gage-tolerance-selection)

## Setup

Attach the two 19 mm-deep test rings to a guided carriage. Their nominal bores are 15.6 mm for the ivory ring and 10.2 mm for the blue ring. Use the dimensions, world poses, and materials in [the geometry specification](geometry.json). The prototype intentionally uses a clear separation between the two bore sizes. This is a functional fit check with relatively large clearances. For tighter tolerances on a production part, use the drawing and calibrated gages specified for that part.

The carriage travels sinusoidally along Y over a 76 mm stroke with a 7.5 s period. Set it at the center moving toward positive Y. Fit a 50 mm-long smooth cylindrical pin to a polymer grasp handle, then set the gage upright in its stand. Connect the two result buttons to an event recorder or record their travel with a camera and a scale.

## Record and score a trial

Record the carriage and the plug from views that show the entry face and the plug tip beneath the passing bore. The passing check requires the tip to travel through the entire bore under little axial resistance. An instrumented gage records the axial reaction at the undersize bore, where the tip stays near the entry face. Confirm the gage remains coaxial and the carriage is moving throughout each scoring interval. Record the chosen button and verify that both hands clear the seated, upright gage after its handle reaches the holder shoulder.

The simulator separately measures the carriage speed, tool axis, tool position, filtered axial contact reaction, physical button travel, and the contact supporting the returned gage. Ring surfaces use polygonal contact solids. The provided OpenSCAD model records fixture geometry and placement.
