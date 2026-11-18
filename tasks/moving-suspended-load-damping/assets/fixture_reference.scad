// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: ab407ee35668ee6f7083258a5336b76c34a7b636b36c661e7134b7ea4c32d786
$fn=48;
module gantry_rail(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([105, 483, 18],center=true);
translate([33, -219, -134]) rotate([0, 0, 0]) cube([32, 29, 266],center=true);
translate([33, 219, -134]) rotate([0, 0, 0]) cube([32, 29, 266],center=true);
translate([25, -219, -269]) rotate([0, 0, 0]) cube([91, 60, 12],center=true);
translate([25, 219, -269]) rotate([0, 0, 0]) cube([91, 60, 12],center=true);
translate([-46, 0, -13]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=5,h=460,center=true);
translate([46, 0, -13]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=5,h=460,center=true);
}
module beam_trolley(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([66, 42, 26],center=true);
translate([-87, 0, -10]) rotate([0, 0, 0]) cube([202, 36, 12],center=true);
translate([-161, 0, -21]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=8,h=29,center=true);
}
module suspended_fixture(){
translate([0, 0, 0]) rotate([16, 0, 0]) cube([55, 148, 52],center=true);
translate([0, -21.4997137537, 74.9784122832]) rotate([16, 0, 0]) cube([10, 11, 122],center=true);
translate([0, -6.89093389542, 24.0315423985]) rotate([16, 0, 0]) cube([29, 162, 10],center=true);
translate([-17, 79.9121322925, 33.3174295596]) rotate([16, 0, 0]) cube([18, 36, 17],center=true);
translate([-17, -85.4248794089, -14.0921956409]) rotate([16, 0, 0]) cube([18, 36, 17],center=true);
}
translate([433, 0, 1022.5]) rotate([0,0,0]) gantry_rail();
translate([433, 0, 994.5]) rotate([0,0,0]) beam_trolley();
translate([272, 39.140504526, 837.000839177]) rotate([0,0,0]) suspended_fixture();
