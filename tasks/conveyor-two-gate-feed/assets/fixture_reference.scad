// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 72c6402de2b21fa23d5d90a6426f832f1362ee4bd938c994dcea73a2997fefc5
$fn=48;
module belt(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([118, 568, 30],center=true);
}
module belt_frame_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([12, 588, 22],center=true);
}
module belt_bearing_left_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([16, 26, 24],center=true);
}
module belt_bearing_left_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([16, 26, 24],center=true);
}
module belt_frame_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([12, 588, 22],center=true);
}
module belt_bearing_right_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([16, 26, 24],center=true);
}
module belt_bearing_right_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([16, 26, 24],center=true);
}
module puck_guide_west(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([8, 536, 10],center=true);
}
module puck_guide_east(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([8, 536, 10],center=true);
}
module collector_stop(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([65, 12, 20],center=true);
}
module puck_1(){
translate([0, 0, -2]) rotate([0, 0, 0]) cylinder(r=22,h=10,center=true);
translate([0, 0, 5]) rotate([0, 0, 0]) cylinder(r=16,h=4,center=true);
}
module puck_2(){
translate([0, 0, -2]) rotate([0, 0, 0]) cylinder(r=22,h=10,center=true);
translate([0, 0, 5]) rotate([0, 0, 0]) cylinder(r=16,h=4,center=true);
}
module puck_3(){
translate([0, 0, -2]) rotate([0, 0, 0]) cylinder(r=22,h=10,center=true);
translate([0, 0, 5]) rotate([0, 0, 0]) cylinder(r=16,h=4,center=true);
}
module exit_lever_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([44, 118, 10],center=true);
translate([11, -47, 25]) rotate([0, 0, 0]) cube([21, 14, 40],center=true);
translate([11, 47, 25]) rotate([0, 0, 0]) cube([21, 14, 40],center=true);
translate([-40, 0, 37]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=7,h=108,center=true);
translate([-17, -47, 37]) rotate([0, 0, 0]) cube([63, 12, 15],center=true);
translate([-17, 47, 37]) rotate([0, 0, 0]) cube([63, 12, 15],center=true);
}
module exit_lever(){
translate([27, 0, 3]) rotate([0, 0, 0]) cube([54, 11, 8],center=true);
translate([-22, 0, 15]) rotate([0, 0, 0]) cube([50, 11, 8],center=true);
translate([0, 0, 8]) rotate([0, 0, 0]) cube([12, 11, 22],center=true);
translate([-41, 0, 6.5]) rotate([0, 0, 0]) cube([11, 4, 22],center=true);
translate([-41, 0, -3]) rotate([0, 0, 0]) cube([10, 4, 16],center=true);
translate([55, 0, 20]) rotate([0, 0, 0]) cube([12, 11, 40],center=true);
translate([55, 0, 39]) rotate([0, 0, 0]) cube([24, 24, 8],center=true);
}
module queue_lever_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([44, 118, 10],center=true);
translate([-11, -47, 25]) rotate([0, 0, 0]) cube([21, 14, 40],center=true);
translate([-11, 47, 25]) rotate([0, 0, 0]) cube([21, 14, 40],center=true);
translate([40, 0, 37]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=7,h=108,center=true);
translate([17, -47, 37]) rotate([0, 0, 0]) cube([63, 12, 15],center=true);
translate([17, 47, 37]) rotate([0, 0, 0]) cube([63, 12, 15],center=true);
}
module queue_lever(){
translate([-27, 0, 3]) rotate([0, 0, 0]) cube([54, 11, 8],center=true);
translate([22, 0, 15]) rotate([0, 0, 0]) cube([50, 11, 8],center=true);
translate([0, 0, 8]) rotate([0, 0, 0]) cube([12, 11, 22],center=true);
translate([41, 0, 6.5]) rotate([0, 0, 0]) cube([11, 4, 22],center=true);
translate([41, 0, -3]) rotate([0, 0, 0]) cube([10, 4, 16],center=true);
translate([-55, 0, 20]) rotate([0, 0, 0]) cube([12, 11, 40],center=true);
translate([-55, 0, 39]) rotate([0, 0, 0]) cube([24, 24, 8],center=true);
}
translate([248, -39, 762.5]) rotate([0,0,0]) belt();
translate([182, -39, 761.5]) rotate([0,0,0]) belt_frame_left();
translate([180, -308, 762.5]) rotate([0,0,0]) belt_bearing_left_front();
translate([180, 230, 762.5]) rotate([0,0,0]) belt_bearing_left_back();
translate([314, -39, 761.5]) rotate([0,0,0]) belt_frame_right();
translate([316, -308, 762.5]) rotate([0,0,0]) belt_bearing_right_front();
translate([316, 230, 762.5]) rotate([0,0,0]) belt_bearing_right_back();
translate([218, -39, 782.5]) rotate([0,0,0]) puck_guide_west();
translate([278, -39, 782.5]) rotate([0,0,0]) puck_guide_east();
translate([248, -313, 787.5]) rotate([0,0,0]) collector_stop();
translate([248, 96, 784.5]) rotate([0,0,0]) puck_1();
translate([248, 147, 784.5]) rotate([0,0,0]) puck_2();
translate([248, 198, 784.5]) rotate([0,0,0]) puck_3();
translate([346, -62, 752.5]) rotate([0,0,0]) exit_lever_base();
translate([306, -62, 789.5]) rotate([0,0,0]) exit_lever();
translate([150, -18, 752.5]) rotate([0,0,0]) queue_lever_base();
translate([190, -18, 789.5]) rotate([0,0,0]) queue_lever();
