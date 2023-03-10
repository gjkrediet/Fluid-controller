// copied and adapted from https://www.thingiverse.com/brunobruno

shaft_height = 8;
shaft_depth = 10;

width=30;
diameter_bottom_finish = 1;

extra_height = 0;
toprounding=-3;

shaft_clear = shaft_height-shaft_depth;
n = 12;
min_cap = 2;
height = shaft_height + min_cap + extra_height;
diameterBottom = width + diameter_bottom_finish;

$fn = 100;

difference(){
	intersection(){
		union(){
			cylinder(d=width,h=height,$fn=n);
			rotate(a=360/n/2,v=[0,0,1]) cylinder(d=width,h=height,$fn=n);
			// bottom finish
            cylinder(d=diameterBottom,h=2,$fn=50);
			translate([0,0,2]) cylinder(d1=diameterBottom,d2=0,h=10,$fn=50);
		};
		// top finish
        cylinder(d1=60,d2=width+toprounding,h=height);
	};
    // Shaft
#	difference(){
		union(){
			cylinder(d=6.2,h=shaft_height,$fn=50);
			cylinder(d1=6.7,d2=0,h=6.5,$fn=50);
		}
        translate([0,0, shaft_clear])
		translate([7,0,6]) cube([10,10,shaft_height-shaft_clear],center=true);
	}
}