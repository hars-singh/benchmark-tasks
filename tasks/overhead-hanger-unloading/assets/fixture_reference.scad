// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: bfbd315f7675a8993fbc2dd92130b8bdfcf04b3970a239cccc746edd8f1b655a
$fn=48;
module rail_frame(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([66, 362, 20],center=true);
translate([15, -158, -132.5]) rotate([0, 0, 0]) cube([32, 29, 265],center=true);
translate([15, 158, -132.5]) rotate([0, 0, 0]) cube([32, 29, 265],center=true);
translate([10, -158, -259]) rotate([0, 0, 0]) cube([104, 54, 12],center=true);
translate([10, 158, -259]) rotate([0, 0, 0]) cube([104, 54, 12],center=true);
translate([-21, 0, -14]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=4.5,h=353,center=true);
translate([21, 0, -14]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=4.5,h=353,center=true);
}
module trolley(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([38, 42, 27],center=true);
translate([-3, 0, -33]) rotate([0, 0, 0]) cube([16, 24, 50],center=true);
translate([-59, 0, -50]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=6,h=118,center=true);
translate([-116, 0, -50]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=10,h=4,center=true);
}
module workpiece(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([14, 64, 78],center=true);
translate([0, 0, 73]) rotate([0, 0, 0]) cube([14, 46, 8],center=true);
translate([0, -19.5, 52]) rotate([0, 0, 0]) cube([14, 7, 33],center=true);
translate([0, 19.5, 52]) rotate([0, 0, 0]) cube([14, 7, 33],center=true);
translate([-19, 0, 23]) rotate([0, 0, 0]) cube([34, 15, 16],center=true);
translate([-43, 0, 23]) rotate([0, 0, 0]) cube([27, 28, 17],center=true);
}
module unload_cradle(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([110, 95, 12],center=true);
translate([-14, 0, 29]) rotate([0, 0, 0]) cube([8, 78, 46],center=true);
translate([14, 0, 29]) rotate([0, 0, 0]) cube([8, 78, 46],center=true);
}
translate([367, 95, 1012.5]) rotate([0,0,0]) rail_frame();
translate([367, 95, 977.5]) rotate([0,0,0]) trolley();
translate([282, 95, 864.5]) rotate([0,0,0]) workpiece();
translate([202, 235, 753.5]) rotate([0,0,0]) unload_cradle();
