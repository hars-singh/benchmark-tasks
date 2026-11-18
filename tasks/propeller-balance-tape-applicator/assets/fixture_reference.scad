// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 8b96dedb1c2f85fbb0d14d824020c0c2b3ab92e54d9b9f7108995de31b6dab03
$fn=48;
module balancer(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([160, 183, 15],center=true);
translate([0, -58, 69]) rotate([0, 0, 0]) cube([26, 14, 123],center=true);
translate([0, 58, 69]) rotate([0, 0, 0]) cube([26, 14, 123],center=true);
translate([-9, -58, 128]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=15,h=9,center=true);
translate([-9, 58, 128]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=15,h=9,center=true);
translate([9, -58, 128]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=15,h=9,center=true);
translate([9, 58, 128]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=15,h=9,center=true);
}
module propeller(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=10.16,h=9.652,center=true);
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=1.8,h=139.7,center=true);
translate([-23, 0, 0]) rotate([-24.0642273955, 0, 0]) cube([35, 22, 2.5],center=true);
translate([-56, 0, 0]) rotate([-17.7616916491, 0, 0]) cube([35, 23, 2.5],center=true);
translate([-91, 0, 0]) rotate([-13.178029288, 0, 0]) cube([36, 20, 2.5],center=true);
translate([-117, 0, 0]) rotate([-10.3132403124, 0, 0]) cube([20, 13, 2.5],center=true);
translate([23, 0, 0]) rotate([24.0642273955, 0, 0]) cube([35, 22, 2.5],center=true);
translate([56, 0, 0]) rotate([17.7616916491, 0, 0]) cube([35, 23, 2.5],center=true);
translate([91, 0, 0]) rotate([13.178029288, 0, 0]) cube([36, 20, 2.5],center=true);
translate([117, 0, 0]) rotate([10.3132403124, 0, 0]) cube([20, 13, 2.5],center=true);
}
module excess_mass(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([15, 17, 0.14],center=true);
}
module blade_backing(){
translate([0, 0, 0]) rotate([-13.178029288, 0, 0]) cube([16, 19, 4],center=true);
}
module tamp_stand(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([78, 61, 18],center=true);
translate([-23, 0, 25.5]) rotate([0, 0, 0]) cube([13, 42, 33],center=true);
translate([23, 0, 25.5]) rotate([0, 0, 0]) cube([13, 42, 33],center=true);
}
module balancing_tape(){
translate([0, 0, 0]) rotate([-13.178029288, 0, 0]) cube([15, 17, 0.14],center=true);
}
module tamp_tool(){
translate([0, 0, 11.5]) rotate([0, 0, 0]) cube([32, 23, 23],center=true);
translate([0, 0, -7]) rotate([0, 0, 0]) cube([52, 30, 6],center=true);
translate([0, 0, -13.5]) rotate([0, 0, 0]) cube([10, 10, 9],center=true);
translate([0, -0.27, -19.55]) rotate([-13.178029288, 0, 0]) cube([8, 6, 2.4],center=true);
}
translate([337, 63, 755]) rotate([0,0,0]) balancer();
translate([337, 63, 897.5]) rotate([0,0,0]) propeller();
translate([438, 63, 897.5]) rotate([0,0,0]) excess_mass();
translate([236, 63, 893.95]) rotate([0,0,0]) blade_backing();
translate([181, 225, 756.5]) rotate([0,0,0]) tamp_stand();
translate([181, 225, 787]) rotate([0,0,0]) balancing_tape();
translate([181, 225, 808.5]) rotate([0,0,0]) tamp_tool();
