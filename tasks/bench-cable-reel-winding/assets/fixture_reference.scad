// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 6433176990bbfbdbc0bf075edc1e33a52d60b6af312cb5e94ff78ca6a9788bb4
$fn=48;
module reel_mount(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([224, 382, 16],center=true);
translate([-91, 0, 103]) rotate([0, 0, 0]) cube([22, 78, 194],center=true);
translate([91, 0, 103]) rotate([0, 0, 0]) cube([22, 78, 194],center=true);
translate([-89, 0, 188]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=21,h=24,center=true);
translate([89, 0, 188]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=21,h=24,center=true);
}
module drum(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=76,h=122,center=true);
translate([-63.5, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=178,h=5,center=true);
translate([63.5, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=178,h=5,center=true);
translate([-25, 0, 0]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=12,h=220,center=true);
}
module cable_guide(){
translate([0, 0, -101]) rotate([0, 0, 0]) cube([104, 100, 12],center=true);
translate([-39, 0, 4]) rotate([0, 0, 0]) cube([14, 60, 208],center=true);
translate([39, 0, 4]) rotate([0, 0, 0]) cube([14, 60, 208],center=true);
translate([0, 0, 115]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=30,h=65,center=true);
translate([-32, 0, 137]) rotate([0, 0, 0]) cube([8, 38, 56],center=true);
translate([32, 0, 137]) rotate([0, 0, 0]) cube([8, 38, 56],center=true);
}
module guide_retainer(){
translate([-48, 0, 5]) rotate([0, 0, 0]) rotate([0,90,0]) cylinder(r=9,h=44,center=true);
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=7,h=71,center=true);
translate([-90, 0, 24]) rotate([0, 0, 0]) cube([190, 17, 12],center=true);
translate([-165, 125, 24]) rotate([0, 0, 0]) cube([15, 264, 12],center=true);
translate([-165, 252, -45]) rotate([0, 0, 0]) cube([14, 21, 138],center=true);
translate([-245, 252, -114]) rotate([0, 0, 0]) cube([170, 25, 10],center=true);
translate([-325, 252, -114]) rotate([0, 0, 0]) cube([22, 25, 21],center=true);
}
module reel_switch_box(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([62, 64, 22],center=true);
}
module motor_start_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([26, 26, 12],center=true);
}
module cable_00(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_01(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_02(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_03(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_04(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_05(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_06(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_07(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_08(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_09(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_10(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_11(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_12(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_13(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_14(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_15(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_16(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_17(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_18(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_19(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_20(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_21(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_22(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_23(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_24(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_25(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_26(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_27(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_28(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_29(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_30(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_31(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_32(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_33(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_34(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_35(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_36(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_37(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_38(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_39(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_40(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_41(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
}
module cable_42(){
translate([0, 0, 0]) rotate([0, 0, 0]) hull() { translate([-6.5,0,0]) sphere(r=3); translate([6.5,0,0]) sphere(r=3); }
translate([0, 0, 0]) rotate([0, 0, 0]) cube([19, 15, 12],center=true);
}
translate([570, -278, 755.5]) rotate([0,0,0]) reel_mount();
translate([570, -278, 943.5]) rotate([0,0,0]) drum();
translate([570, -104, 854.5]) rotate([0,0,0]) cable_guide();
translate([618, -104, 1031.5]) rotate([0,0,0]) guide_retainer();
translate([265, -164, 758.5]) rotate([0,0,0]) reel_switch_box();
translate([265, -164, 785.5]) rotate([0,0,0]) motor_start_button();
translate([570, -268.5, 1023.1]) rotate([0,0,90]) cable_00();
translate([570, -249.5, 1023.1]) rotate([0,0,90]) cable_01();
translate([570, -230.5, 1023.1]) rotate([0,0,90]) cable_02();
translate([570, -211.5, 1023.1]) rotate([0,0,90]) cable_03();
translate([570, -192.5, 1023.1]) rotate([0,0,90]) cable_04();
translate([570, -173.5, 1023.1]) rotate([0,0,90]) cable_05();
translate([570, -154.5, 1023.1]) rotate([0,0,90]) cable_06();
translate([570, -135.5, 1023.1]) rotate([0,0,90]) cable_07();
translate([570, -116.5, 1023.1]) rotate([0,0,90]) cable_08();
translate([570, -97.5, 1023.1]) rotate([0,0,90]) cable_09();
translate([570, -78.5, 1023.1]) rotate([0,0,90]) cable_10();
translate([570, -59.5, 1023.1]) rotate([0,0,90]) cable_11();
translate([570, -40.5, 1023.1]) rotate([0,0,90]) cable_12();
translate([570, -21.5, 1023.1]) rotate([0,0,90]) cable_13();
translate([570, -2.5, 1023.1]) rotate([0,0,90]) cable_14();
translate([570, 16.5, 1023.1]) rotate([0,0,90]) cable_15();
translate([570, 35.5, 1023.1]) rotate([0,0,90]) cable_16();
translate([570, 54.5, 1023.1]) rotate([0,0,90]) cable_17();
translate([570, 73.5, 1023.1]) rotate([0,0,90]) cable_18();
translate([570, 92.5, 1023.1]) rotate([0,0,90]) cable_19();
translate([570, 111.5, 1023.1]) rotate([0,0,90]) cable_20();
translate([570, 130.5, 1023.1]) rotate([0,0,90]) cable_21();
translate([570, 149.5, 1023.1]) rotate([0,0,90]) cable_22();
translate([570, 168.5, 1023.1]) rotate([0,0,90]) cable_23();
translate([570, 187.5, 1023.1]) rotate([0,0,90]) cable_24();
translate([570, 206.5, 1023.1]) rotate([0,0,90]) cable_25();
translate([570, 225.5, 1023.1]) rotate([0,0,90]) cable_26();
translate([570, 244.5, 1023.1]) rotate([0,0,90]) cable_27();
translate([570, 263.5, 1023.1]) rotate([0,0,90]) cable_28();
translate([570, 282.5, 1023.1]) rotate([0,0,90]) cable_29();
translate([570, 301.5, 1023.1]) rotate([0,0,90]) cable_30();
translate([570, 320.5, 1023.1]) rotate([0,0,90]) cable_31();
translate([570, 339.5, 1023.1]) rotate([0,0,90]) cable_32();
translate([570, 358.5, 1023.1]) rotate([0,0,90]) cable_33();
translate([570, 377.5, 1023.1]) rotate([0,0,90]) cable_34();
translate([570, 396.5, 1023.1]) rotate([0,0,90]) cable_35();
translate([570, 415.5, 1023.1]) rotate([0,0,90]) cable_36();
translate([570, 434.5, 1023.1]) rotate([0,0,90]) cable_37();
translate([570, 453.5, 1023.1]) rotate([0,0,90]) cable_38();
translate([570, 472.5, 1023.1]) rotate([0,0,90]) cable_39();
translate([570, 491.5, 1023.1]) rotate([0,0,90]) cable_40();
translate([570, 510.5, 1023.1]) rotate([0,0,90]) cable_41();
translate([570, 529.5, 1023.1]) rotate([0,0,90]) cable_42();
