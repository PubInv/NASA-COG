//translate([0,0,-5])
//linear_extrude(height = 5, center = false, convexity = 10)
//import("C:/Users/Loki-VR/Documents/Control v1-brd.svg", convexity=3);

//37mm tall
board_height = 37;
standoff_width=7;
hole_diameter = 3.5;
hole_locations = [[13.3,5,0],[13.3,185,0],[143.3,5,0],[143.3,185,0]];
$fn= $preview ? 32 : 64;
//for(i = hole_locations)
//translate(i)
difference(){
cylinder(h=board_height,d=standoff_width);
    translate([0,0,-0.5])
cylinder(h=board_height+1,d=hole_diameter);
}