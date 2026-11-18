// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 7838cb8b78dc14102535e830cea22ec27e1e11ca7ee88feacd9d7d66b1213c78
$fn=48;
module switch_test_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([205, 247, 8],center=true);
translate([-83, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-72, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-77.5, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-77.5, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-77.5, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-92.5, -113.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-92.5, 113.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([72, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([83, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([77.5, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([77.5, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([77.5, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([92.5, -113.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([92.5, 113.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([60, -39, 50.5]) rotate([0, 0, 0]) cube([39, 145, 73],center=true);
translate([-16, -39, 49]) rotate([0, 0, 0]) cube([17, 138, 70],center=true);
translate([-18, -43, 87]) rotate([0, 0, 0]) cube([12, 100, 3],center=true);
}
module indexed_cam_dog(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([92, 90, 10],center=true);
translate([-6, -10, -11]) rotate([0, 0, 0]) cube([21, 14, 12],center=true);
translate([-6, 11, -11]) rotate([0, 0, 0]) cube([21, 14, 12],center=true);
translate([-6, 33, -11]) rotate([0, 0, 0]) cube([21, 14, 12],center=true);
translate([-31, -55, 13]) rotate([0, 0, 0]) cube([30, 13, 30],center=true);
translate([55, 0, -8]) rotate([0, 0, 0]) cube([101, 25, 9],center=true);
translate([94, 0, -17]) rotate([0, 0, 0]) cube([25, 16, 8],center=true);
translate([94, -16, -13.5]) rotate([-23.7494944929, 0, 0]) cube([25, 18, 7],center=true);
translate([94, 16, -13.5]) rotate([23.7494944929, 0, 0]) cube([25, 18, 7],center=true);
}
module spring_index_pin(){
translate([0, 0, 2.5]) rotate([0, 0, 0]) cube([11, 5, 29],center=true);
translate([-33.5, 0, -8]) rotate([0, 0, 0]) cube([67, 11, 8],center=true);
translate([-68, 43.5, -8]) rotate([0, 0, 0]) cube([11, 91, 8],center=true);
translate([-68, 87, 7]) rotate([0, 0, 0]) cube([11, 11, 29],center=true);
translate([-68, 87, 21]) rotate([0, 0, 0]) cube([27, 25, 5],center=true);
}
module roller_switch_enclosure(){
translate([0, 0, -26]) rotate([0, 0, 0]) cube([58, 48, 16],center=true);
translate([-23, 0, 9]) rotate([0, 0, 0]) cube([38, 32, 54],center=true);
translate([-24, 0, 36]) rotate([0, 0, 0]) cube([23, 21, 8],center=true);
}
module switch_roller_plunger(){
translate([9.5, 0, -12]) rotate([0, 0, 0]) cube([3, 7, 23],center=true);
translate([5, 0, -21]) rotate([0, 0, 0]) cube([8, 7, 16],center=true);
}
module switch_roller_wheel(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=6,h=18,center=true);
}
module switch_test_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=373,center=true);
}
module switch_test_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=373,center=true);
}
module switch_test_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([240, 408, 2.5],center=true);
}
module switch_test_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([240, 15, 22],center=true);
}
module switch_test_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([240, 15, 22],center=true);
}
translate([315, 25, 762.5]) rotate([0,0,0]) switch_test_carriage();
translate([350, -18, 864.5]) rotate([0,0,0]) indexed_cam_dog();
translate([344, 4, 839.5]) rotate([0,0,0]) spring_index_pin();
translate([468, 22, 781.5]) rotate([0,0,0]) roller_switch_enclosure();
translate([444, 22, 841.5]) rotate([0,0,0]) switch_roller_plunger();
translate([444, 22, 841.5]) rotate([0,0,0]) switch_roller_wheel();
translate([237.5, 25, 757.5]) rotate([0,0,0]) switch_test_carriage_rail_left();
translate([392.5, 25, 757.5]) rotate([0,0,0]) switch_test_carriage_rail_right();
translate([315, 25, 748.75]) rotate([0,0,0]) switch_test_carriage_base();
translate([315, -169, 758.5]) rotate([0,0,0]) switch_test_carriage_end_front();
translate([315, 219, 758.5]) rotate([0,0,0]) switch_test_carriage_end_back();
