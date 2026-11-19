// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 2a0534bf03a3a449a10b4e73da6e93da54e76e3affcbfef08af7dff53fa675af
$fn=48;
module entry_deck(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([196, 223, 58],center=true);
translate([0, -14.5, 32.5]) rotate([0, 0, 0]) cube([196, 195, 7],center=true);
translate([0, 97, 25]) rotate([0, 0, 0]) cube([196, 28, 8],center=true);
translate([-88, 0, 39]) rotate([0, 0, 0]) cube([8, 190, 9],center=true);
translate([88, 0, 39]) rotate([0, 0, 0]) cube([8, 190, 9],center=true);
}
module exit_deck(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([196, 223, 58],center=true);
translate([0, 14.5, 32.5]) rotate([0, 0, 0]) cube([196, 195, 7],center=true);
translate([0, -97, 25]) rotate([0, 0, 0]) cube([196, 28, 8],center=true);
translate([-88, 0, 39]) rotate([0, 0, 0]) cube([8, 190, 9],center=true);
translate([88, 0, 39]) rotate([0, 0, 0]) cube([8, 190, 9],center=true);
}
module cart_stop(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([158, 12, 40],center=true);
}
module bridge_rest(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([131, 104, 14],center=true);
translate([-51, 0, 15]) rotate([0, 0, 0]) cube([20, 95, 15],center=true);
translate([51, 0, 15]) rotate([0, 0, 0]) cube([20, 95, 15],center=true);
}
module bridge_plate(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([124, 94, 7],center=true);
translate([-30, 0, -9]) rotate([0, 0, 0]) cube([14, 48, 11],center=true);
translate([30, 0, -9]) rotate([0, 0, 0]) cube([14, 48, 11],center=true);
translate([0, 0, 9]) rotate([0, 0, 0]) cube([16, 23, 14],center=true);
}
module loaded_cart(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([104, 107, 15],center=true);
translate([-44, 0, 10]) rotate([0, 0, 0]) cube([8, 78, 17],center=true);
translate([44, 0, 10]) rotate([0, 0, 0]) cube([8, 78, 17],center=true);
translate([0, -47, 10]) rotate([0, 0, 0]) cube([80, 8, 17],center=true);
translate([0, 47, 10]) rotate([0, 0, 0]) cube([80, 8, 17],center=true);
translate([0, -65, -6]) rotate([0, 0, 0]) cube([25, 50, 14],center=true);
translate([0, -77, -6]) rotate([0, 0, 0]) cube([22, 22, 23],center=true);
}
module front_axle(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=5,h=86,center=true);
translate([-37, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=19,h=16,center=true);
translate([37, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=19,h=16,center=true);
}
module rear_axle(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=5,h=86,center=true);
translate([-37, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=19,h=16,center=true);
translate([37, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=19,h=16,center=true);
}
module cart_load(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([58, 55, 40],center=true);
}
translate([296, -140, 777.5]) rotate([0,0,0]) entry_deck();
translate([296, 140, 777.5]) rotate([0,0,0]) exit_deck();
translate([296, 169, 833.5]) rotate([0,0,0]) cart_stop();
translate([120, 142, 754.5]) rotate([0,0,0]) bridge_rest();
translate([120, 142, 780.5]) rotate([0,0,0]) bridge_plate();
translate([296, -151, 857.5]) rotate([0,0,0]) loaded_cart();
translate([296, -111, 832.5]) rotate([0,0,0]) front_axle();
translate([296, -191, 832.5]) rotate([0,0,0]) rear_axle();
translate([296, -151, 885.5]) rotate([0,0,0]) cart_load();
