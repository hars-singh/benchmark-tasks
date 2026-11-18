// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: b040230ad5eb6bb28a0434d9c64c26f6c051276830b4e9eb3276bc2c7a6e7396
$fn=48;
module enclosure_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([200, 226, 8],center=true);
translate([-80.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-69.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-75, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-75, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-75, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-90, -103, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-90, 103, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([69.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([80.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([75, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([75, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([75, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([90, -103, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([90, 103, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([7, 4, 33]) rotate([0, 0, 0]) cube([126, 120, 34],center=true);
translate([7, 4, 53]) rotate([0, 0, 0]) cube([121, 115, 6],center=true);
translate([7, 4, 58]) rotate([0, 0, 0]) cube([118, 112, 3],center=true);
translate([-31, -60, 54.5]) rotate([0, 0, 0]) cube([20, 18, 79],center=true);
translate([-31, -60, 91]) rotate([0, 0, 0]) cube([16, 14, 18],center=true);
}
module spring_lid(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([116, 110, 12],center=true);
translate([34, 27, 6]) rotate([0, 0, 0]) cube([36, 29, 1.8],center=true);
translate([-38, -23, 6]) rotate([0, 0, 0]) cube([29, 31, 2],center=true);
}
module eccentric_latch(){
translate([0, -73, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=4.5,h=162,center=true);
translate([10.4225395509, 0, -4.86011001002]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=9,h=18,center=true);
translate([10.9880748053, -145, 23.564002463]) rotate([0, -65, 0]) cube([54, 13, 10],center=true);
translate([23.2440043957, -145, 49.846928287]) rotate([0, 0, 0]) sphere(r=10);
}
module enclosure_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=344,center=true);
}
module enclosure_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=344,center=true);
}
module enclosure_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([235, 379, 2.5],center=true);
}
module enclosure_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([235, 15, 22],center=true);
}
module enclosure_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([235, 15, 22],center=true);
}
translate([326, 40, 762.5]) rotate([0,0,0]) enclosure_carriage();
translate([333, 44, 832.5]) rotate([0,0,0]) spring_lid();
translate([295, 21, 853.5]) rotate([0,0,0]) eccentric_latch();
translate([251, 40, 757.5]) rotate([0,0,0]) enclosure_carriage_rail_left();
translate([401, 40, 757.5]) rotate([0,0,0]) enclosure_carriage_rail_right();
translate([326, 40, 748.75]) rotate([0,0,0]) enclosure_carriage_base();
translate([326, -139.5, 758.5]) rotate([0,0,0]) enclosure_carriage_end_front();
translate([326, 219.5, 758.5]) rotate([0,0,0]) enclosure_carriage_end_back();
