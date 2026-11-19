// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 493e0b693f89c281eb8da39abb29a7e02fd7501da94857ba81c94d8b38a3dbba
$fn=48;
module stage(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([120, 100, 30],center=true);
translate([0, 0, 29.25]) rotate([0, 0, 0]) cube([30, 30, 28.5],center=true);
translate([-33, -32, 32]) rotate([0, 0, 0]) cube([9, 18, 34],center=true);
translate([33, -32, 32]) rotate([0, 0, 0]) cube([9, 18, 34],center=true);
translate([-33, -27, 49]) rotate([0, 0, 0]) cube([9, 18, 4],center=true);
translate([33, -27, 49]) rotate([0, 0, 0]) cube([9, 18, 4],center=true);
}
module board(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([80, 60, 3],center=true);
translate([-5.3, 0, 1.55]) rotate([0, 0, 0]) cube([3.18, 13.2, 0.1],center=true);
translate([5.3, 0, 1.55]) rotate([0, 0, 0]) cube([3.18, 13.2, 0.1],center=true);
translate([-25, 18, 2.5]) rotate([0, 0, 0]) cube([6.3, 3.2, 2],center=true);
translate([-25, 0, 2.5]) rotate([0, 0, 0]) cube([6.3, 3.2, 2],center=true);
translate([-25, -18, 2.5]) rotate([0, 0, 0]) cube([6.3, 3.2, 2],center=true);
}
module part(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([15.2, 16.2, 9.8],center=true);
translate([-5.3, 0, -5]) rotate([0, 0, 0]) cube([3, 12.5, 0.2],center=true);
translate([5.3, 0, -5]) rotate([0, 0, 0]) cube([3, 12.5, 0.2],center=true);
}
module hot_air(){
translate([0, 0, 0]) rotate([-32, 0, 0]) rotate([90,0,0]) cylinder(r=12,h=90,center=true);
translate([0, 76.3243286541, -47.692733781]) rotate([-32, 0, 0]) rotate([90,0,0]) cylinder(r=9,h=90,center=true);
translate([0, 144.168176347, -90.0862749196]) rotate([-32, 0, 0]) rotate([90,0,0]) cylinder(r=4,h=70,center=true);
}
module gun_stand_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([50, 6, 148.370597094],center=true);
translate([-15.5, 0, 80.1852985471]) rotate([0, 0, 0]) cube([6, 6, 12],center=true);
translate([15.5, 0, 80.1852985471]) rotate([0, 0, 0]) cube([6, 6, 12],center=true);
}
module gun_stand_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([50, 6, 111.276248598],center=true);
translate([-15.5, 0, 61.638124299]) rotate([0, 0, 0]) cube([6, 6, 12],center=true);
translate([15.5, 0, 61.638124299]) rotate([0, 0, 0]) cube([6, 6, 12],center=true);
}
module tray_floor(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([100, 100, 10],center=true);
}
module tray_x_low(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([10, 120, 50],center=true);
}
module tray_x_high(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([10, 120, 50],center=true);
}
module tray_y_low(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([100, 10, 50],center=true);
}
module tray_y_high(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([100, 10, 50],center=true);
}
translate([240, 0, 762.5]) rotate([0,0,0]) stage();
translate([240, 0, 807.5]) rotate([0,0,0]) board();
translate([240, 0, 814.2]) rotate([0,0,0]) part();
translate([300, -180, 887.5]) rotate([0,0,0]) hot_air();
translate([300, -209.681683365, 821.685298547]) rotate([0,0,0]) gun_stand_back();
translate([300, -150.318316635, 803.138124299]) rotate([0,0,0]) gun_stand_front();
translate([350, 200, 752.5]) rotate([0,0,0]) tray_floor();
translate([295, 200, 772.5]) rotate([0,0,0]) tray_x_low();
translate([405, 200, 772.5]) rotate([0,0,0]) tray_x_high();
translate([350, 145, 772.5]) rotate([0,0,0]) tray_y_low();
translate([350, 255, 772.5]) rotate([0,0,0]) tray_y_high();
