// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 1e75e14972626e525cb9eaea6f7035d3df9ec993e67947920a7be6a4e04561a7
$fn=48;
module check_stand(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([230, 145, 20],center=true);
translate([-80, 0, 65]) rotate([0, 0, 0]) cube([12, 54, 110],center=true);
translate([92, 0, 65]) rotate([0, 0, 0]) cube([12, 54, 110],center=true);
translate([9, 0, 111]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=4,h=181,center=true);
translate([9, 61, 109]) rotate([0, 0, 0]) cube([18, 15, 199],center=true);
translate([9, 30, 193]) rotate([0, 0, 0]) cube([18, 71, 15],center=true);
translate([9, 3, 189]) rotate([0, 0, 0]) cube([22, 24, 29],center=true);
}
module roller(){
translate([0, 0, 0.35]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=25,h=150,center=true);
translate([-107.5, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=5,h=81,center=true);
translate([-146, 0, -16]) rotate([0, 0, 0]) cube([6, 11, 40],center=true);
}
module crank_grip(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=8,h=30,center=true);
}
module indicator_plunger(){
translate([0, 0, 0]) rotate([0, 0, 0]) sphere(r=1.5);
translate([0, 0, 25]) rotate([0, 0, 0]) cylinder(r=3.5,h=48,center=true);
}
module result_panel(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([180, 135, 32],center=true);
translate([56, 0, 25]) rotate([0, 0, 0]) cube([51, 75, 21],center=true);
translate([56, 0, 36]) rotate([0, 0, 0]) cube([40, 57, 1],center=true);
}
module pass_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([30, 30, 18],center=true);
}
module fail_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([30, 30, 18],center=true);
}
translate([520, -50, 757.5]) rotate([0,0,0]) check_stand();
translate([529, -50, 868.5]) rotate([0,0,0]) roller();
translate([374, -50, 833.5]) rotate([0,0,0]) crank_grip();
translate([529, -50, 895.35]) rotate([0,0,0]) indicator_plunger();
translate([160, 205, 763.5]) rotate([0,0,0]) result_panel();
translate([160, 239, 785.5]) rotate([0,0,0]) pass_button();
translate([160, 171, 785.5]) rotate([0,0,0]) fail_button();
