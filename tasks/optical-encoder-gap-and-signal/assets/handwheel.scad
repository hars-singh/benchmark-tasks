// Optional 176 mm handwheel for an unpowered 6.35 mm motor shaft.
// Print the wheel flat. Use part="spinner" for the separate finger knob.

part = "wheel";
$fn = 128;
spinner_x = -22;
spinner_y = -85;

module handwheel() {
    difference() {
        union() {
            difference() {
                cylinder(h=9, r=88);
                translate([0,0,-1]) cylinder(h=11, r=78);
            }
            cylinder(h=9, r=17);
            for (angle=[0,120,240])
                rotate([0,0,angle])
                    translate([12,-6,0]) cube([69,12,9]);
            translate([spinner_x,spinner_y,0]) cylinder(h=9, r=11);
        }
        translate([0,0,-1]) cylinder(h=11, d=6.8);
        for (x=[-8,8])
            translate([x,0,-1]) cylinder(h=11, d=4.4);
        translate([spinner_x,spinner_y,-1]) cylinder(h=11, d=4.4);
        for (angle=[0,45])
            rotate([0,0,angle])
                translate([80,-0.55,8.6]) cube([7,1.1,0.8]);
    }
}

module spinner() {
    difference() {
        cylinder(h=20, d=20);
        translate([0,0,-1]) cylinder(h=22, d=4.6);
    }
}

if (part == "wheel") handwheel();
if (part == "spinner") spinner();
