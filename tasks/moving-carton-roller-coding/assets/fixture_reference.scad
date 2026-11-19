// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 557c3e71fa862d0f59774b9e9199c600353eeb77178381039d288e899a6e2241
$fn=48;
module carton_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([155, 160, 8],center=true);
translate([-58, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-47, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-52.5, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-52.5, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-52.5, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-67.5, -70, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-67.5, 70, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([47, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([58, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([52.5, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([52.5, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([52.5, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([67.5, -70, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([67.5, 70, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
}
module carton(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([133, 131, 54],center=true);
}
module coder_rest(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([107, 75, 13],center=true);
translate([-39, 0, 23.75]) rotate([0, 0, 0]) cube([17, 56, 30.5],center=true);
translate([39, 0, 23.75]) rotate([0, 0, 0]) cube([17, 56, 30.5],center=true);
}
module coder_handle(){
translate([0, 0, 17]) rotate([0, 0, 0]) cube([23, 26, 27],center=true);
translate([0, 0, -1]) rotate([0, 0, 0]) cube([86, 30, 8],center=true);
translate([-38, 0, -17]) rotate([0, 0, 0]) cube([9, 23, 37],center=true);
translate([38, 0, -17]) rotate([0, 0, 0]) cube([9, 23, 37],center=true);
translate([-38, 0, -25]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=5,h=8,center=true);
translate([38, 0, -25]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=5,h=8,center=true);
}
module print_wheel(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=25.4,h=42,center=true);
}
module carton_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=336,center=true);
}
module carton_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=336,center=true);
}
module carton_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([190, 371, 2.5],center=true);
}
module carton_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([190, 15, 22],center=true);
}
module carton_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([190, 15, 22],center=true);
}
translate([363, 60, 762.5]) rotate([0,0,0]) carton_carriage();
translate([359, 60, 804.5]) rotate([0,0,0]) carton();
translate([181, 201, 754]) rotate([0,0,0]) coder_rest();
translate([181, 201, 828.5]) rotate([0,0,0]) coder_handle();
translate([181, 201, 803.5]) rotate([0,0,0]) print_wheel();
translate([310.5, 60, 757.5]) rotate([0,0,0]) carton_carriage_rail_left();
translate([415.5, 60, 757.5]) rotate([0,0,0]) carton_carriage_rail_right();
translate([363, 60, 748.75]) rotate([0,0,0]) carton_carriage_base();
translate([363, -115.5, 758.5]) rotate([0,0,0]) carton_carriage_end_front();
translate([363, 235.5, 758.5]) rotate([0,0,0]) carton_carriage_end_back();
