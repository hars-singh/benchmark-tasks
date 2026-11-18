// Dimensions in millimetres. Geometry reference for the guide-mounted fixture.
// MIT licensed. This does not specify a functioning cutting assembly.
// Install an ordinary manual-feed crank sharpener on a guided, ballasted base
// for real trials; cutters remain enclosed in the commercial housing.
$fn=64;
part="assembly"; // assembly, base, post, case, crank, knob, cradle
module base(){cube([180,140,30],center=true);}
module post(){cube([25,50,55],center=true);}
module case(){difference(){cube([60,70,70],center=true);cube([62,9,9],center=true);}}
module crank(){
 rotate([0,90,0])cylinder(h=18,r=8,center=true);
 translate([0,0,-17.5])cube([8,12,43],center=true);
 translate([20,0,-35])rotate([0,90,0])cylinder(h=40,r=3,center=true);
}
module knob(){rotate([0,90,0])difference(){cylinder(h=30,r=8,center=true);cylinder(h=32,r=3.2,center=true);}}
module cradle(){cube([16,40,30],center=true);}
if(part=="base")base();
else if(part=="post")post();
else if(part=="case")case();
else if(part=="crank")crank();
else if(part=="knob")knob();
else if(part=="cradle")cradle();
else {
 translate([245,-40,15])base();
 translate([245,-40,57.5])post();
 translate([245,-40,120])case();
 translate([285,-40,120])crank();
 translate([331,-40,85])knob();
 translate([55,100,15])cradle();
 translate([160,100,15])cradle();
}
