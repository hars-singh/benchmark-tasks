// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 767e694ef7efa85c557d7b4bd129a62651a0d63392bff9b625783f0b582e92d5
$fn=48;
module test_runway(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([227, 427, 8],center=true);
}
module service_cart(){
translate([0, -12, 0]) rotate([0, 0, 0]) cube([119, 113, 17],center=true);
translate([0, 52, -5]) rotate([0, 0, 0]) cube([14, 42, 11],center=true);
translate([0, 56, -14]) rotate([0, 0, 0]) cylinder(r=9,h=8,center=true);
translate([0, -65, 5]) rotate([0, 0, 0]) cube([30, 50, 14],center=true);
translate([0, -77, 15]) rotate([0, 0, 0]) cube([24, 21, 22],center=true);
}
module sprung_caster_mount(){
translate([0, 0, 11]) rotate([0, 0, 0]) cylinder(r=11,h=10,center=true);
}
module swivel_fork(){
translate([0, 0, -3]) rotate([0, 0, 0]) cylinder(r=15,h=10,center=true);
translate([-11, -16, -11]) rotate([0, 0, 0]) cube([5, 25, 27],center=true);
translate([11, -16, -11]) rotate([0, 0, 0]) cube([5, 25, 27],center=true);
translate([8.03635207967, 19.4014701827, 3.5]) rotate([0, 0, -22.5]) cube([4.16, 8, 9],center=true);
translate([11.6669748934, 17.4608618584, 3.5]) rotate([0, 0, -33.75]) cube([4.16, 8, 9],center=true);
translate([14.8492424049, 14.8492424049, 3.5]) rotate([0, 0, -45]) cube([4.16, 8, 9],center=true);
translate([17.4608618584, 11.6669748934, 3.5]) rotate([0, 0, -56.25]) cube([4.16, 8, 9],center=true);
translate([19.4014701827, 8.03635207967, 3.5]) rotate([0, 0, -67.5]) cube([4.16, 8, 9],center=true);
translate([20.5964908885, 4.09689676234, 3.5]) rotate([0, 0, -78.75]) cube([4.16, 8, 9],center=true);
translate([21, 1.2858791391e-15, 3.5]) rotate([0, 0, -90]) cube([4.16, 8, 9],center=true);
translate([20.5964908885, -4.09689676234, 3.5]) rotate([0, 0, -101.25]) cube([4.16, 8, 9],center=true);
translate([19.4014701827, -8.03635207967, 3.5]) rotate([0, 0, -112.5]) cube([4.16, 8, 9],center=true);
translate([17.4608618584, -11.6669748934, 3.5]) rotate([0, 0, -123.75]) cube([4.16, 8, 9],center=true);
translate([14.8492424049, -14.8492424049, 3.5]) rotate([0, 0, -135]) cube([4.16, 8, 9],center=true);
translate([11.6669748934, -17.4608618584, 3.5]) rotate([0, 0, -146.25]) cube([4.16, 8, 9],center=true);
translate([8.03635207967, -19.4014701827, 3.5]) rotate([0, 0, -157.5]) cube([4.16, 8, 9],center=true);
translate([4.09689676234, -20.5964908885, 3.5]) rotate([0, 0, -168.75]) cube([4.16, 8, 9],center=true);
translate([2.57175827821e-15, -21, 3.5]) rotate([0, 0, -180]) cube([4.16, 8, 9],center=true);
translate([-4.09689676234, -20.5964908885, 3.5]) rotate([0, 0, -191.25]) cube([4.16, 8, 9],center=true);
translate([-8.03635207967, -19.4014701827, 3.5]) rotate([0, 0, -202.5]) cube([4.16, 8, 9],center=true);
translate([-11.6669748934, -17.4608618584, 3.5]) rotate([0, 0, -213.75]) cube([4.16, 8, 9],center=true);
translate([-14.8492424049, -14.8492424049, 3.5]) rotate([0, 0, -225]) cube([4.16, 8, 9],center=true);
translate([-17.4608618584, -11.6669748934, 3.5]) rotate([0, 0, -236.25]) cube([4.16, 8, 9],center=true);
translate([-19.4014701827, -8.03635207967, 3.5]) rotate([0, 0, -247.5]) cube([4.16, 8, 9],center=true);
translate([-20.5964908885, -4.09689676234, 3.5]) rotate([0, 0, -258.75]) cube([4.16, 8, 9],center=true);
translate([-21, -3.85763741731e-15, 3.5]) rotate([0, 0, -270]) cube([4.16, 8, 9],center=true);
translate([-20.5964908885, 4.09689676234, 3.5]) rotate([0, 0, -281.25]) cube([4.16, 8, 9],center=true);
translate([-19.4014701827, 8.03635207967, 3.5]) rotate([0, 0, -292.5]) cube([4.16, 8, 9],center=true);
translate([-17.4608618584, 11.6669748934, 3.5]) rotate([0, 0, -303.75]) cube([4.16, 8, 9],center=true);
translate([-14.8492424049, 14.8492424049, 3.5]) rotate([0, 0, -315]) cube([4.16, 8, 9],center=true);
translate([-11.6669748934, 17.4608618584, 3.5]) rotate([0, 0, -326.25]) cube([4.16, 8, 9],center=true);
translate([-8.03635207967, 19.4014701827, 3.5]) rotate([0, 0, -337.5]) cube([4.16, 8, 9],center=true);
}
module caster_wheel(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=18,h=15,center=true);
}
module fixed_axle(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=4,h=98,center=true);
translate([-47, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=18,h=16,center=true);
translate([47, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=18,h=16,center=true);
}
module direction_pin(){
translate([0, 0, 0]) rotate([0, 0, 0]) cylinder(r=3.1,h=20,center=true);
translate([-17, 0, 8]) rotate([0, 0, 0]) cube([46, 9, 6],center=true);
translate([-39, 0, 15.5]) rotate([0, 0, 0]) cube([20, 19, 7],center=true);
}
translate([297, 9, 751.5]) rotate([0,0,0]) test_runway();
translate([297, -73, 811.5]) rotate([0,0,0]) service_cart();
translate([297, -17, 791.5]) rotate([0,0,0]) sprung_caster_mount();
translate([297, -17, 791.5]) rotate([0,0,33.2315521176]) swivel_fork();
translate([308.508502673, -34.5657156482, 773.5]) rotate([0,0,33.2315521176]) caster_wheel();
translate([297, -123, 773.5]) rotate([0,0,0]) fixed_axle();
translate([297, 4, 816.5]) rotate([0,0,0]) direction_pin();
