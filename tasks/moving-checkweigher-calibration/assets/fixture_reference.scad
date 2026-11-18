// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 32715d75c7106ffe74180650626e3089851e002b0ab85a81887b5b09d376a55f
$fn=48;
module scale_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([134, 114, 8],center=true);
translate([-47.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-36.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-42, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-42, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-42, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-57, -47, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-57, 47, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([36.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([47.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([42, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([42, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([42, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([57, -47, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([57, 47, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-30, -23, 21]) rotate([0, 0, 0]) cube([18, 14, 12],center=true);
translate([-30, 23, 21]) rotate([0, 0, 0]) cube([18, 14, 12],center=true);
translate([30, -23, 21]) rotate([0, 0, 0]) cube([18, 14, 12],center=true);
translate([30, 23, 21]) rotate([0, 0, 0]) cube([18, 14, 12],center=true);
}
module weighing_pan(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([98, 84, 8],center=true);
}
module reference_weight(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([36, 32, 40],center=true);
}
module verification_pack(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([47, 38, 35],center=true);
}
module scale_panel(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([102, 165, 34],center=true);
translate([29, 0, 19.5]) rotate([0, 0, 0]) cube([24, 144, 5],center=true);
}
module zero_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([28, 30, 18],center=true);
}
module span_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([28, 30, 18],center=true);
}
module verify_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([28, 30, 18],center=true);
}
module scale_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=234,center=true);
}
module scale_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=234,center=true);
}
module scale_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([169, 269, 2.5],center=true);
}
module scale_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([169, 15, 22],center=true);
}
module scale_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([169, 15, 22],center=true);
}
translate([310, 71, 762.5]) rotate([0,0,0]) scale_carriage();
translate([310, 71, 793.5]) rotate([0,0,0]) weighing_pan();
translate([172, 188, 767.5]) rotate([0,0,0]) reference_weight();
translate([208, 276, 765]) rotate([0,0,0]) verification_pack();
translate([191, -181, 764.5]) rotate([0,0,0]) scale_panel();
translate([175, -136, 787.5]) rotate([0,0,0]) zero_button();
translate([175, -181, 787.5]) rotate([0,0,0]) span_button();
translate([175, -226, 787.5]) rotate([0,0,0]) verify_button();
translate([268, 71, 757.5]) rotate([0,0,0]) scale_carriage_rail_left();
translate([352, 71, 757.5]) rotate([0,0,0]) scale_carriage_rail_right();
translate([310, 71, 748.75]) rotate([0,0,0]) scale_carriage_base();
translate([310, -53.5, 758.5]) rotate([0,0,0]) scale_carriage_end_front();
translate([310, 195.5, 758.5]) rotate([0,0,0]) scale_carriage_end_back();
