// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 650e873026d24b6a89a60aee12245d45feec5f8dad7a2ad750bedfeb6c092b5f
$fn=48;
module cam_test_stand(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([208, 162, 15],center=true);
translate([-65, -48, 96]) rotate([0, 0, 0]) cube([28, 39, 177],center=true);
translate([65, -48, 96]) rotate([0, 0, 0]) cube([28, 39, 177],center=true);
translate([-64, 0, 30]) rotate([0, 0, 0]) cube([21, 71, 51],center=true);
translate([64, 0, 30]) rotate([0, 0, 0]) cube([21, 71, 51],center=true);
translate([-64, 0, 56.5]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=9,h=14,center=true);
translate([64, 0, 56.5]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=9,h=14,center=true);
translate([0, -48, 178.5]) rotate([0, 0, 0]) cube([156, 24, 12],center=true);
}
module eccentric_test_cam(){
translate([0, 0, 3.6]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=27,h=36,center=true);
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=5,h=149,center=true);
translate([-18.6, 0, 13]) rotate([0, 0, 0]) cube([1, 4, 19],center=true);
}
module preload_shoe(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([43, 41, 12],center=true);
}
module follower_stem(){
translate([0, 0, 28]) rotate([0, 0, 0]) cube([9, 13, 43],center=true);
translate([0, 0, 15]) rotate([0, 0, 0]) cube([47, 14, 8],center=true);
translate([-21, 0, 6]) rotate([0, 0, 0]) cube([4, 14, 22],center=true);
translate([21, 0, 6]) rotate([0, 0, 0]) cube([4, 14, 22],center=true);
}
module preloaded_roller(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=11,h=23,center=true);
}
module preload_toggle(){
translate([0, 13.5, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=17,h=31,center=true);
translate([-18, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=4.5,h=161,center=true);
translate([-93, 22.6284049341, 4.11768018879]) rotate([10.3132403124, 0, 0]) cube([10, 50, 9],center=true);
translate([-93, 47.2244972538, 8.59341952444]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=10,h=22,center=true);
}
translate([324, 103, 755]) rotate([0,0,0]) cam_test_stand();
translate([324, 103, 811.5]) rotate([0,0,0]) eccentric_test_cam();
translate([324, 103, 915.5]) rotate([0,0,0]) preload_shoe();
translate([324, 103, 863.5]) rotate([0,0,0]) follower_stem();
translate([324, 103, 863.5]) rotate([0,0,0]) preloaded_roller();
translate([324, 103, 938.5]) rotate([0,0,0]) preload_toggle();
