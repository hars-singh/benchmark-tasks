// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 9f11b6fcbf2e031f8a630d4644880b47588a5d6d0045ba4b46db71ce9387776f
$fn=48;
module extractor_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([260, 244, 8],center=true);
translate([-110.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-99.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-105, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-105, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-105, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-120, -112, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-120, 112, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([99.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([110.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([105, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([105, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([105, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([120, -112, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([120, 112, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([69, 60, 31]) rotate([0, 0, 0]) cube([76, 51, 40],center=true);
translate([24.8284625326, -128, 41.5]) rotate([0, 0, 0]) cube([37, 43, 55],center=true);
translate([45, -42, 32]) rotate([0, 0, 0]) cube([31, 155, 19],center=true);
}
module motor(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=18.5,h=66,center=true);
translate([-44, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=3,h=22,center=true);
}
module pinion(){
translate([0, 0, 5.38]) rotate([-0, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 1.39244646265, 5.19668094544]) rotate([-15, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 2.69, 4.65921667236]) rotate([-30, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 3.80423448278, 3.80423448278]) rotate([-45, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 4.65921667236, 2.69]) rotate([-60, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 5.19668094544, 1.39244646265]) rotate([-75, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 5.38, 3.29429988971e-16]) rotate([-90, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 5.19668094544, -1.39244646265]) rotate([-105, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 4.65921667236, -2.69]) rotate([-120, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 3.80423448278, -3.80423448278]) rotate([-135, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 2.69, -4.65921667236]) rotate([-150, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 1.39244646265, -5.19668094544]) rotate([-165, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 6.58859977941e-16, -5.38]) rotate([-180, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -1.39244646265, -5.19668094544]) rotate([-195, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -2.69, -4.65921667236]) rotate([-210, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -3.80423448278, -3.80423448278]) rotate([-225, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -4.65921667236, -2.69]) rotate([-240, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -5.19668094544, -1.39244646265]) rotate([-255, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -5.38, -9.88289966912e-16]) rotate([-270, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -5.19668094544, 1.39244646265]) rotate([-285, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -4.65921667236, 2.69]) rotate([-300, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -3.80423448278, 3.80423448278]) rotate([-315, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -2.69, 4.65921667236]) rotate([-330, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, -1.39244646265, 5.19668094544]) rotate([-345, 0, 0]) cube([8, 2.1, 4],center=true);
translate([0, 0, 8.1]) rotate([-0, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 3.09973580216, 7.48342421334]) rotate([-22.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 5.72756492761, 5.72756492761]) rotate([-45, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 7.48342421334, 3.09973580216]) rotate([-67.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 8.1, 4.95981953655e-16]) rotate([-90, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 7.48342421334, -3.09973580216]) rotate([-112.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 5.72756492761, -5.72756492761]) rotate([-135, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 3.09973580216, -7.48342421334]) rotate([-157.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, 9.91963907309e-16, -8.1]) rotate([-180, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -3.09973580216, -7.48342421334]) rotate([-202.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -5.72756492761, -5.72756492761]) rotate([-225, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -7.48342421334, -3.09973580216]) rotate([-247.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -8.1, -1.48794586096e-15]) rotate([-270, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -7.48342421334, 3.09973580216]) rotate([-292.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -5.72756492761, 5.72756492761]) rotate([-315, 0, 0]) cube([8, 2.1, 1.8],center=true);
translate([0, -3.09973580216, 7.48342421334]) rotate([-337.5, 0, 0]) cube([8, 2.1, 1.8],center=true);
}
module puller_crosshead(){
translate([-5, 19, -23]) rotate([0, 0, 0]) cube([94, 153, 12],center=true);
translate([32, 111, -23]) rotate([0, 0, 0]) cube([22, 62, 12],center=true);
translate([32, 136, 19]) rotate([0, 0, 0]) cube([17, 13, 65],center=true);
translate([-38, -78, 0]) rotate([0, 0, 0]) cube([18, 18, 23],center=true);
translate([-38, -78, 10]) rotate([0, 0, 0]) sphere(r=10);
}
module pinion_fork(){
translate([0, 0, 2]) rotate([0, 0, 0]) cube([3.5, 23, 7],center=true);
translate([0, -6.5, -9.5]) rotate([0, 0, 0]) cube([3.5, 6, 19],center=true);
translate([0, 6.5, -9.5]) rotate([0, 0, 0]) cube([3.5, 6, 19],center=true);
translate([0, 16, 2]) rotate([0, 0, 0]) cube([14, 36, 8],center=true);
translate([0, 20, -7]) rotate([0, 0, 0]) cube([17, 4, 11],center=true);
translate([0, 46, -7]) rotate([0, 0, 0]) cube([17, 4, 11],center=true);
}
module fork_press_roller(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=7,h=22,center=true);
}
module wedge_ram(){
translate([-2.97315382255, 0, 8.49472520729]) rotate([0, -19.2900462192, 0]) cube([8, 25, 76],center=true);
translate([10, 0, 27]) rotate([0, 0, 0]) cube([36, 35, 8],center=true);
}
module pinion_tray(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([72, 61, 8],center=true);
translate([-34, 0, 10]) rotate([0, 0, 0]) cube([4, 61, 16],center=true);
translate([34, 0, 10]) rotate([0, 0, 0]) cube([4, 61, 16],center=true);
translate([0, -28.5, 10]) rotate([0, 0, 0]) cube([64, 4, 16],center=true);
translate([0, 28.5, 10]) rotate([0, 0, 0]) cube([64, 4, 16],center=true);
}
module extractor_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=344,center=true);
}
module extractor_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=344,center=true);
}
module extractor_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([295, 379, 2.5],center=true);
}
module extractor_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([295, 15, 22],center=true);
}
module extractor_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([295, 15, 22],center=true);
}
translate([335, 25, 762.5]) rotate([0,0,0]) extractor_carriage();
translate([405, 85, 843.5]) rotate([0,0,0]) motor();
translate([357, 85, 843.5]) rotate([0,0,0]) pinion();
translate([364, -25, 829.5]) rotate([0,0,0]) puller_crosshead();
translate([364, 85, 875.5]) rotate([0,0,0]) pinion_fork();
translate([364, 118, 872]) rotate([0,0,0]) fork_press_roller();
translate([337.828462533, -103, 855.5]) rotate([0,0,0]) wedge_ram();
translate([325, 85, 781.5]) rotate([0,0,0]) pinion_tray();
translate([230, 25, 757.5]) rotate([0,0,0]) extractor_carriage_rail_left();
translate([440, 25, 757.5]) rotate([0,0,0]) extractor_carriage_rail_right();
translate([335, 25, 748.75]) rotate([0,0,0]) extractor_carriage_base();
translate([335, -154.5, 758.5]) rotate([0,0,0]) extractor_carriage_end_front();
translate([335, 204.5, 758.5]) rotate([0,0,0]) extractor_carriage_end_back();
