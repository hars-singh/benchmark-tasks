// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 5085aa3cb36f58e1aa7ef021e1dedad03f858bbd1e3898d52b2e66d4b0d053b4
$fn=48;
module fixture(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([126, 115, 12],center=true);
translate([8, -15, 41.5]) rotate([0, 0, 0]) cylinder(r=4,h=71,center=true);
translate([8, 15, 41.5]) rotate([0, 0, 0]) cylinder(r=4,h=71,center=true);
translate([28, -15, 41.5]) rotate([0, 0, 0]) cylinder(r=4,h=71,center=true);
translate([28, 15, 41.5]) rotate([0, 0, 0]) cylinder(r=4,h=71,center=true);
translate([-25, 0, 35]) rotate([0, 0, 0]) cube([27, 52, 58],center=true);
}
module board(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([39, 42, 1.6],center=true);
}
module housing(){
translate([0, 0, -1.22]) rotate([0, 0, 0]) cube([5.5, 20.8, 0.26],center=true);
translate([0, 0, 0.965]) rotate([0, 0, 0]) cube([5.5, 20.8, 0.77],center=true);
translate([0, -9.275, -0.2]) rotate([0, 0, 0]) cube([5.5, 2.25, 1.58],center=true);
translate([0, 9.275, -0.2]) rotate([0, 0, 0]) cube([5.5, 2.25, 1.58],center=true);
translate([0.77, 0, 0.14]) rotate([0, 0, 0]) cube([0.34, 16.3, 0.43],center=true);
}
module cam_guide(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([8.4, 15.8, 0.22],center=true);
}
module slide_wedge(){
translate([1, 0, -0.03]) rotate([0, 24, 0]) cube([2.2, 15.5, 0.16],center=true);
translate([0, -9.55, 0.67]) rotate([0, 0, 0]) cube([3.3, 1.6, 1.25],center=true);
translate([0, 9.55, 0.67]) rotate([0, 0, 0]) cube([3.3, 1.6, 1.25],center=true);
translate([1, -6.6, -0.49]) rotate([0, 0, 0]) cube([3.7, 1.1, 0.085],center=true);
translate([1, 6.6, -0.49]) rotate([0, 0, 0]) cube([3.7, 1.1, 0.085],center=true);
}
module contact_follower(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([2.2, 15.6, 0.18],center=true);
translate([0, 0, -0.19]) rotate([0, 0, 0]) sphere(r=0.22);
}
module upper_contact_leaf(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([2.9, 14.7, 0.18],center=true);
}
module conductor_tip(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([7, 16, 0.3],center=true);
}
module pull_clip(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([3, 17, 1.5],center=true);
}
module clip_jaw(){
translate([0, 0, -0.33]) rotate([0, 0, 0]) cube([2, 18, 0.3],center=true);
translate([0, 0, 0.33]) rotate([0, 0, 0]) cube([2, 18, 0.3],center=true);
}
module latch_pusher(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([10, 33, 14],center=true);
translate([18.5, -9.55, 0]) rotate([0, 0, 0]) cube([28, 1.05, 1.05],center=true);
translate([18.5, 9.55, 0]) rotate([0, 0, 0]) cube([28, 1.05, 1.05],center=true);
}
module cable_seater(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([12, 28, 14],center=true);
translate([0, 0, -23]) rotate([0, 0, 0]) cube([10, 12, 41],center=true);
translate([15, 0, -44]) rotate([0, 0, 0]) cube([33, 9, 7],center=true);
translate([24, -32.5, -44]) rotate([0, 0, 0]) cube([9, 67, 6],center=true);
translate([32, -65, -44]) rotate([0, 0, 0]) cube([21, 9, 6],center=true);
}
module carriage_rails(){
translate([-72, 0, 0]) rotate([0, 0, 0]) cube([9, 183, 8],center=true);
translate([72, 0, 0]) rotate([0, 0, 0]) cube([9, 183, 8],center=true);
}
translate([286, 0, 753.5]) rotate([0,0,0]) fixture();
translate([302, 0, 831.7]) rotate([0,0,0]) board();
translate([284.75, 0, 833.85]) rotate([0,0,0]) housing();
translate([283.8, 0, 832.65]) rotate([0,0,0]) cam_guide();
translate([281.4, 0, 833.32]) rotate([0,0,0]) slide_wedge();
translate([283.15, 0, 833.66]) rotate([0,0,0]) contact_follower();
translate([283.85, 0, 834.255]) rotate([0,0,0]) upper_contact_leaf();
translate([277.7, 0, 834]) rotate([0,0,0]) conductor_tip();
translate([272.7, 0, 835.5]) rotate([0,0,0]) pull_clip();
translate([274.7, 0, 834]) rotate([0,0,0]) clip_jaw();
translate([247, 0, 833.99]) rotate([0,0,0]) latch_pusher();
translate([232, 65, 866.5]) rotate([0,0,0]) cable_seater();
translate([286, 0, 751.5]) rotate([0,0,0]) carriage_rails();
