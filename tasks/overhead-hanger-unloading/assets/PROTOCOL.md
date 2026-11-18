# Retained overhead hanger

This bench station represents a finishing-line load carried by a trolley. Its closed eye rides on a horizontal pin with a retaining collar. The arm lifts the eye, withdraws it axially, and seats the released workpiece in a separate cradle. [Overhead finishing conveyor examples](https://www.pacline.com/solutions/paint-finishing-conveyor-systems/powder-coating/).

## Setup and reset

Mount the horizontal 12 mm pin and its 20 mm end collar on a guided trolley. Use the mounting centers and solid dimensions in `geometry.json`. Make the cradle and closed eye as rigid, functional parts. Verify that the eye hangs freely behind the collar and clears the collar when lifted. Mount the cradle in the documented receiving position and start it empty. Reset the trolley to the middle of its 110 mm stroke at phase zero, moving toward positive Y. Start both grippers open in the ready pose.

## Recording

Use an overhead camera for carriage and withdrawal motion. Include a side view with graduations beside the eye and collar for vertical clearance. Record from before the first contact through release and rest in the cradle. Track trolley speed during pickup and the workpiece's trailing edge during withdrawal. A part held by the pin and visibly carried by the moving trolley gives the physical counterpart to the simulator's upward pin force measurement. A seated, unheld workpiece resting on the cradle floor gives the counterpart to the measured cradle reaction.
