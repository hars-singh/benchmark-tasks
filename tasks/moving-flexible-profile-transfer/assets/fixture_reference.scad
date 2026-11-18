// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 94f34a7056a9d24379da1dac9f9b9d27ac03e0cb14ce7981b4b7693647e8a8c4
$fn=48;
module profile_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([94, 382, 8],center=true);
translate([-27.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-16.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-22, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-22, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-22, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-37, -181, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-37, 181, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([16.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([27.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([22, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([22, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([22, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([37, -181, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([37, 181, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([0, -86, 44.25]) rotate([0, 0, 0]) cube([45, 24, 58.5],center=true);
translate([-20, -86, 77.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([20, -86, 77.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([0, 0, 44.25]) rotate([0, 0, 0]) cube([45, 24, 58.5],center=true);
translate([-20, 0, 77.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([20, 0, 77.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([0, 86, 44.25]) rotate([0, 0, 0]) cube([45, 24, 58.5],center=true);
translate([-20, 86, 77.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([20, 86, 77.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
}
module profile_center(){
translate([0, 0, -1.5]) rotate([0, 0, 0]) cube([20, 131, 4],center=true);
translate([-8, 0, 2]) rotate([0, 0, 0]) cube([4, 131, 3],center=true);
translate([8, 0, 2]) rotate([0, 0, 0]) cube([4, 131, 3],center=true);
}
module profile_positive_end(){
translate([0, 0, -1.5]) rotate([0, 0, 0]) cube([20, 113, 4],center=true);
translate([-8, 0, 2]) rotate([0, 0, 0]) cube([4, 113, 3],center=true);
translate([8, 0, 2]) rotate([0, 0, 0]) cube([4, 113, 3],center=true);
}
module profile_negative_end(){
translate([0, 0, -1.5]) rotate([0, 0, 0]) cube([20, 113, 4],center=true);
translate([-8, 0, 2]) rotate([0, 0, 0]) cube([4, 113, 3],center=true);
translate([8, 0, 2]) rotate([0, 0, 0]) cube([4, 113, 3],center=true);
}
module receiving_saddles(){
translate([0, 0, -28]) rotate([0, 0, 0]) cube([67, 322, 12],center=true);
translate([0, -95, 16.25]) rotate([0, 0, 0]) cube([45, 24, 76.5],center=true);
translate([-20, -95, 58.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([20, -95, 58.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([0, 95, 16.25]) rotate([0, 0, 0]) cube([45, 24, 76.5],center=true);
translate([-20, 95, 58.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([20, 95, 58.5]) rotate([0, 0, 0]) cube([6, 23, 10],center=true);
translate([0, 0, 9]) rotate([0, 0, 0]) cube([45, 24, 62],center=true);
}
module spring_middle_saddle(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([45, 24, 7],center=true);
}
module profile_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=476,center=true);
}
module profile_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=476,center=true);
}
module profile_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([129, 511, 2.5],center=true);
}
module profile_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([129, 15, 22],center=true);
}
module profile_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([129, 15, 22],center=true);
}
translate([269, 0, 762.5]) rotate([0,0,0]) profile_carriage();
translate([269, 0, 839.5]) rotate([0,0,0]) profile_center();
translate([269, 123, 839.5]) rotate([0,0,0]) profile_positive_end();
translate([269, -123, 839.5]) rotate([0,0,0]) profile_negative_end();
translate([386, 0, 781.5]) rotate([0,0,0]) receiving_saddles();
translate([386, 0, 832.95]) rotate([0,0,0]) spring_middle_saddle();
translate([247, 0, 757.5]) rotate([0,0,0]) profile_carriage_rail_left();
translate([291, 0, 757.5]) rotate([0,0,0]) profile_carriage_rail_right();
translate([269, 0, 748.75]) rotate([0,0,0]) profile_carriage_base();
translate([269, -245.5, 758.5]) rotate([0,0,0]) profile_carriage_end_front();
translate([269, 245.5, 758.5]) rotate([0,0,0]) profile_carriage_end_back();
