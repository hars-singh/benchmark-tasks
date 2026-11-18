// Millimetre geometric reference; use real functional tools, not printed heated or cutting equipment.
// Source SHA-256: 245e83c228b5464fea7a8c2a8f7801f9cc6a310927b54ac8192c838a67afe414
$fn=48;
module inspection_carriage(){
translate([0, 0, 11]) rotate([0, 0, 0]) cube([120, 157, 8],center=true);
translate([-40.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-29.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([-35, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-35, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([-35, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([-50, -68.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-50, 68.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([29.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([40.5, 0, -5]) rotate([0, 0, 0]) cube([3, 50, 14],center=true);
translate([35, 0, -10.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([35, 0, 0.5]) rotate([0, 0, 0]) cube([8, 50, 3],center=true);
translate([35, 0, 4.5]) rotate([0, 0, 0]) cube([14, 50, 6],center=true);
translate([50, -68.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([50, 68.5, 15.5]) rotate([0, 0, 0]) cylinder(r=2.5,h=1,center=true);
translate([-19, 47, 24.5]) rotate([0, 0, 0]) cube([12, 27, 19],center=true);
translate([19, 47, 24.5]) rotate([0, 0, 0]) cube([12, 27, 19],center=true);
translate([-19, -47, 24.5]) rotate([0, 0, 0]) cube([12, 27, 19],center=true);
translate([19, -47, 24.5]) rotate([0, 0, 0]) cube([12, 27, 19],center=true);
}
module plug_gage(){
translate([0, 0, 0]) rotate([0, 0, 0]) cylinder(r=12.5,h=40,center=true);
translate([0, 0, -40]) rotate([0, 0, 0]) cylinder(r=6,h=50,center=true);
}
module gage_stand(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([58, 62, 8],center=true);
translate([-12, 0, 23.5]) rotate([0, 0, 0]) cube([8, 26, 39],center=true);
translate([12, 0, 23.5]) rotate([0, 0, 0]) cube([8, 26, 39],center=true);
translate([0, -12, 23.5]) rotate([0, 0, 0]) cube([16, 8, 39],center=true);
translate([0, 12, 23.5]) rotate([0, 0, 0]) cube([16, 8, 39],center=true);
translate([12.25, 0, 45.4]) rotate([0, 0, 0]) cube([8.5, 4.74453242038, 8],center=true);
translate([11.832591372, 3.17053330251, 45.4]) rotate([0, 0, 15]) cube([8.5, 4.74453242038, 8],center=true);
translate([10.6088111964, 6.125, 45.4]) rotate([0, 0, 30]) cube([8.5, 4.74453242038, 8],center=true);
translate([8.66205806954, 8.66205806954, 45.4]) rotate([0, 0, 45]) cube([8.5, 4.74453242038, 8],center=true);
translate([6.125, 10.6088111964, 45.4]) rotate([0, 0, 60]) cube([8.5, 4.74453242038, 8],center=true);
translate([3.17053330251, 11.832591372, 45.4]) rotate([0, 0, 75]) cube([8.5, 4.74453242038, 8],center=true);
translate([7.50096164478e-16, 12.25, 45.4]) rotate([0, 0, 90]) cube([8.5, 4.74453242038, 8],center=true);
translate([-3.17053330251, 11.832591372, 45.4]) rotate([0, 0, 105]) cube([8.5, 4.74453242038, 8],center=true);
translate([-6.125, 10.6088111964, 45.4]) rotate([0, 0, 120]) cube([8.5, 4.74453242038, 8],center=true);
translate([-8.66205806954, 8.66205806954, 45.4]) rotate([0, 0, 135]) cube([8.5, 4.74453242038, 8],center=true);
translate([-10.6088111964, 6.125, 45.4]) rotate([0, 0, 150]) cube([8.5, 4.74453242038, 8],center=true);
translate([-11.832591372, 3.17053330251, 45.4]) rotate([0, 0, 165]) cube([8.5, 4.74453242038, 8],center=true);
translate([-12.25, 1.50019232896e-15, 45.4]) rotate([0, 0, 180]) cube([8.5, 4.74453242038, 8],center=true);
translate([-11.832591372, -3.17053330251, 45.4]) rotate([0, 0, 195]) cube([8.5, 4.74453242038, 8],center=true);
translate([-10.6088111964, -6.125, 45.4]) rotate([0, 0, 210]) cube([8.5, 4.74453242038, 8],center=true);
translate([-8.66205806954, -8.66205806954, 45.4]) rotate([0, 0, 225]) cube([8.5, 4.74453242038, 8],center=true);
translate([-6.125, -10.6088111964, 45.4]) rotate([0, 0, 240]) cube([8.5, 4.74453242038, 8],center=true);
translate([-3.17053330251, -11.832591372, 45.4]) rotate([0, 0, 255]) cube([8.5, 4.74453242038, 8],center=true);
translate([-2.25028849343e-15, -12.25, 45.4]) rotate([0, 0, 270]) cube([8.5, 4.74453242038, 8],center=true);
translate([3.17053330251, -11.832591372, 45.4]) rotate([0, 0, 285]) cube([8.5, 4.74453242038, 8],center=true);
translate([6.125, -10.6088111964, 45.4]) rotate([0, 0, 300]) cube([8.5, 4.74453242038, 8],center=true);
translate([8.66205806954, -8.66205806954, 45.4]) rotate([0, 0, 315]) cube([8.5, 4.74453242038, 8],center=true);
translate([10.6088111964, -6.125, 45.4]) rotate([0, 0, 330]) cube([8.5, 4.74453242038, 8],center=true);
translate([11.832591372, -3.17053330251, 45.4]) rotate([0, 0, 345]) cube([8.5, 4.74453242038, 8],center=true);
}
module result_panel(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([99, 160, 34],center=true);
translate([37, 0, 19.5]) rotate([0, 0, 0]) cube([24, 160, 10],center=true);
}
module ivory_bore(){
translate([10.8, 0, 0]) rotate([0, 0, 0]) cube([6, 4.03360893341, 19],center=true);
translate([10.4319989239, 2.79524568711, 0]) rotate([0, 0, 15]) cube([6, 4.03360893341, 19],center=true);
translate([9.35307436087, 5.4, 0]) rotate([0, 0, 30]) cube([6, 4.03360893341, 19],center=true);
translate([7.63675323681, 7.63675323681, 0]) rotate([0, 0, 45]) cube([6, 4.03360893341, 19],center=true);
translate([5.4, 9.35307436087, 0]) rotate([0, 0, 60]) cube([6, 4.03360893341, 19],center=true);
translate([2.79524568711, 10.4319989239, 0]) rotate([0, 0, 75]) cube([6, 4.03360893341, 19],center=true);
translate([6.6130927154e-16, 10.8, 0]) rotate([0, 0, 90]) cube([6, 4.03360893341, 19],center=true);
translate([-2.79524568711, 10.4319989239, 0]) rotate([0, 0, 105]) cube([6, 4.03360893341, 19],center=true);
translate([-5.4, 9.35307436087, 0]) rotate([0, 0, 120]) cube([6, 4.03360893341, 19],center=true);
translate([-7.63675323681, 7.63675323681, 0]) rotate([0, 0, 135]) cube([6, 4.03360893341, 19],center=true);
translate([-9.35307436087, 5.4, 0]) rotate([0, 0, 150]) cube([6, 4.03360893341, 19],center=true);
translate([-10.4319989239, 2.79524568711, 0]) rotate([0, 0, 165]) cube([6, 4.03360893341, 19],center=true);
translate([-10.8, 1.32261854308e-15, 0]) rotate([0, 0, 180]) cube([6, 4.03360893341, 19],center=true);
translate([-10.4319989239, -2.79524568711, 0]) rotate([0, 0, 195]) cube([6, 4.03360893341, 19],center=true);
translate([-9.35307436087, -5.4, 0]) rotate([0, 0, 210]) cube([6, 4.03360893341, 19],center=true);
translate([-7.63675323681, -7.63675323681, 0]) rotate([0, 0, 225]) cube([6, 4.03360893341, 19],center=true);
translate([-5.4, -9.35307436087, 0]) rotate([0, 0, 240]) cube([6, 4.03360893341, 19],center=true);
translate([-2.79524568711, -10.4319989239, 0]) rotate([0, 0, 255]) cube([6, 4.03360893341, 19],center=true);
translate([-1.98392781462e-15, -10.8, 0]) rotate([0, 0, 270]) cube([6, 4.03360893341, 19],center=true);
translate([2.79524568711, -10.4319989239, 0]) rotate([0, 0, 285]) cube([6, 4.03360893341, 19],center=true);
translate([5.4, -9.35307436087, 0]) rotate([0, 0, 300]) cube([6, 4.03360893341, 19],center=true);
translate([7.63675323681, -7.63675323681, 0]) rotate([0, 0, 315]) cube([6, 4.03360893341, 19],center=true);
translate([9.35307436087, -5.4, 0]) rotate([0, 0, 330]) cube([6, 4.03360893341, 19],center=true);
translate([10.4319989239, -2.79524568711, 0]) rotate([0, 0, 345]) cube([6, 4.03360893341, 19],center=true);
}
module blue_bore(){
translate([9.45, 0, 0]) rotate([0, 0, 0]) cube([8.7, 4.03360893341, 19],center=true);
translate([9.12799905843, 2.44583997622, 0]) rotate([0, 0, 15]) cube([8.7, 4.03360893341, 19],center=true);
translate([8.18394006576, 4.725, 0]) rotate([0, 0, 30]) cube([8.7, 4.03360893341, 19],center=true);
translate([6.68215908221, 6.68215908221, 0]) rotate([0, 0, 45]) cube([8.7, 4.03360893341, 19],center=true);
translate([4.725, 8.18394006576, 0]) rotate([0, 0, 60]) cube([8.7, 4.03360893341, 19],center=true);
translate([2.44583997622, 9.12799905843, 0]) rotate([0, 0, 75]) cube([8.7, 4.03360893341, 19],center=true);
translate([5.78645612597e-16, 9.45, 0]) rotate([0, 0, 90]) cube([8.7, 4.03360893341, 19],center=true);
translate([-2.44583997622, 9.12799905843, 0]) rotate([0, 0, 105]) cube([8.7, 4.03360893341, 19],center=true);
translate([-4.725, 8.18394006576, 0]) rotate([0, 0, 120]) cube([8.7, 4.03360893341, 19],center=true);
translate([-6.68215908221, 6.68215908221, 0]) rotate([0, 0, 135]) cube([8.7, 4.03360893341, 19],center=true);
translate([-8.18394006576, 4.725, 0]) rotate([0, 0, 150]) cube([8.7, 4.03360893341, 19],center=true);
translate([-9.12799905843, 2.44583997622, 0]) rotate([0, 0, 165]) cube([8.7, 4.03360893341, 19],center=true);
translate([-9.45, 1.15729122519e-15, 0]) rotate([0, 0, 180]) cube([8.7, 4.03360893341, 19],center=true);
translate([-9.12799905843, -2.44583997622, 0]) rotate([0, 0, 195]) cube([8.7, 4.03360893341, 19],center=true);
translate([-8.18394006576, -4.725, 0]) rotate([0, 0, 210]) cube([8.7, 4.03360893341, 19],center=true);
translate([-6.68215908221, -6.68215908221, 0]) rotate([0, 0, 225]) cube([8.7, 4.03360893341, 19],center=true);
translate([-4.725, -8.18394006576, 0]) rotate([0, 0, 240]) cube([8.7, 4.03360893341, 19],center=true);
translate([-2.44583997622, -9.12799905843, 0]) rotate([0, 0, 255]) cube([8.7, 4.03360893341, 19],center=true);
translate([-1.73593683779e-15, -9.45, 0]) rotate([0, 0, 270]) cube([8.7, 4.03360893341, 19],center=true);
translate([2.44583997622, -9.12799905843, 0]) rotate([0, 0, 285]) cube([8.7, 4.03360893341, 19],center=true);
translate([4.725, -8.18394006576, 0]) rotate([0, 0, 300]) cube([8.7, 4.03360893341, 19],center=true);
translate([6.68215908221, -6.68215908221, 0]) rotate([0, 0, 315]) cube([8.7, 4.03360893341, 19],center=true);
translate([8.18394006576, -4.725, 0]) rotate([0, 0, 330]) cube([8.7, 4.03360893341, 19],center=true);
translate([9.12799905843, -2.44583997622, 0]) rotate([0, 0, 345]) cube([8.7, 4.03360893341, 19],center=true);
}
module ivory_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([33, 31, 18],center=true);
}
module blue_button(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([33, 31, 18],center=true);
}
module inspection_carriage_rail_left(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=273,center=true);
}
module inspection_carriage_rail_right(){
translate([0, 0, 0]) rotate([0, 0, 0]) rotate([90,0,0]) cylinder(r=3.5,h=273,center=true);
}
module inspection_carriage_base(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([155, 308, 2.5],center=true);
}
module inspection_carriage_end_front(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([155, 15, 22],center=true);
}
module inspection_carriage_end_back(){
translate([0, 0, 0]) rotate([0, 0, 0]) cube([155, 15, 22],center=true);
}
translate([323, 84, 762.5]) rotate([0,0,0]) inspection_carriage();
translate([184, 233, 820.5]) rotate([0,0,0]) plug_gage();
translate([184, 233, 751.5]) rotate([0,0,0]) gage_stand();
translate([205, -185, 764.5]) rotate([0,0,0]) result_panel();
translate([323, 131, 806]) rotate([0,0,0]) ivory_bore();
translate([323, 37, 806]) rotate([0,0,0]) blue_bore();
translate([205, -144, 787.5]) rotate([0,0,0]) ivory_button();
translate([205, -226, 787.5]) rotate([0,0,0]) blue_button();
translate([288, 84, 757.5]) rotate([0,0,0]) inspection_carriage_rail_left();
translate([358, 84, 757.5]) rotate([0,0,0]) inspection_carriage_rail_right();
translate([323, 84, 748.75]) rotate([0,0,0]) inspection_carriage_base();
translate([323, -60, 758.5]) rotate([0,0,0]) inspection_carriage_end_front();
translate([323, 228, 758.5]) rotate([0,0,0]) inspection_carriage_end_back();
