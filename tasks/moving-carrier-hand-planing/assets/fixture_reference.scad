// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 6430ccab270873468d173f421dac0eac9b92e960ab43b40e52f5614bdb567816
$fn=48;
module stage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([140, 320, 8],center=true);
translate([-50.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-39.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-45, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-45, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-45, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-60, -150, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-60, 150, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([39.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([50.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([45, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([45, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([45, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([60, -150, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([60, 150, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
}
module part(){
translate([0, -51.7824855787, 0]) rotate([0, 0, 0]) cube([65, 166.435028843, 6],center=true);
translate([0, 85.2175144213, 0]) rotate([0, 0, 0]) cube([65, 99.5649711575, 6],center=true);
translate([-30.5, 0, 12.5]) rotate([0, 0, 0]) cube([4, 270, 25],center=true);
translate([30.5, 0, 12.5]) rotate([0, 0, 0]) cube([4, 270, 25],center=true);
translate([0, 20, 11.7492424049]) rotate([-45, 0, 0]) cube([51, 40, 2],center=true);
translate([0, 100, 14]) rotate([0, 0, 0]) cylinder(r=14,h=22,center=true);
translate([0, 100, 40]) rotate([0, 0, 0]) sphere(r=22);
translate([0, -100, 67]) rotate([0, 0, 0]) cube([22, 45, 20],center=true);
translate([0, -122, 38]) rotate([14.3239448783, 0, 0]) cube([22, 20, 55],center=true);
translate([0, -80, 25]) rotate([0, 0, 0]) cube([25, 25, 35],center=true);
translate([0, -25, 30]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=8,h=12,center=true);
}
module stock(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([60, 400, 25],center=true);
}
module vise_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([100, 420, 50],center=true);
}
module vise_jaw_0(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([10, 400, 20],center=true);
}
module vise_jaw_1(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([10, 400, 20],center=true);
}
module stage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=420,center=true);
}
module stage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=420,center=true);
}
module stage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([175, 455, 2.5],center=true);
}
module stage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([175, 15, 22],center=true);
}
module stage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([175, 15, 22],center=true);
}
translate([300, 0, 762.5]) rotate([0,0,0]) stage();
translate([300, 0, 780.6]) rotate([0,0,0]) part();
translate([150, 0, 810]) rotate([0,0,0]) stock();
translate([150, 0, 772.5]) rotate([0,0,0]) vise_base();
translate([115, 0, 807.5]) rotate([0,0,0]) vise_jaw_0();
translate([185, 0, 807.5]) rotate([0,0,0]) vise_jaw_1();
translate([255, 0, 757.5]) rotate([0,0,0]) stage_rail_left();
translate([345, 0, 757.5]) rotate([0,0,0]) stage_rail_right();
translate([300, 0, 748.75]) rotate([0,0,0]) stage_base();
translate([300, -217.5, 758.5]) rotate([0,0,0]) stage_end_front();
translate([300, 217.5, 758.5]) rotate([0,0,0]) stage_end_back();
