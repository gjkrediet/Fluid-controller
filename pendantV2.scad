$fn=100;

$l=102;
$w=72;
$h1=19;
$d=6;
$dbat=20;

difference() {
    hull() {
        translate ([0,0,-10]) rotate([0,90,0]) cylinder (d=$dbat+4,h=70,center=true);
        translate ([0,7,($h1)/2]) hull () {
            translate ([($l-$d)/2, ($w-$d)/2,-($h1-$d)/2]) sphere(d=$d+4);
            translate ([($l-$d)/2, -($w-$d)/2,-($h1-$d)/2]) sphere(d=$d+4);
            translate ([-($l-$d)/2, ($w-$d)/2,-($h1-$d)/2]) sphere(d=$d+4);
            translate ([-($l-$d)/2, -($w-$d)/2,-($h1-$d)/2]) sphere(d=$d+4);
            translate ([($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=0.01,center=true);
            translate ([($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=0.01,center=true);
            translate ([-($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=0.01,center=true);
            translate ([-($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=0.01,center=true);
        }
    }
    difference() {
        hull() {
            translate ([0,0,-7]) rotate([0,90,0]) cylinder (d=$dbat,h=70,center=true);
            translate ([0,7,($h1)/2+0.1]) hull () {
                translate ([($l-$d)/2, ($w-$d)/2,-($h1-$d)/2]) sphere(d=$d);
                translate ([($l-$d)/2, -($w-$d)/2,-($h1-$d)/2]) sphere(d=$d);
                translate ([-($l-$d)/2, ($w-$d)/2,-($h1-$d)/2]) sphere(d=$d);
                translate ([-($l-$d)/2, -($w-$d)/2,-($h1-$d)/2]) sphere(d=$d);
                translate ([($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d,h=0.01,center=true);
                translate ([($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d,h=0.01,center=true);
                translate ([-($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d,h=0.01,center=true);
                translate ([-($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d,h=0.01,center=true);
            }
        }
    }
         //uitspring usb en print
    translate ([$l/2-1,-1.7,20.6]) cube([10,10,3],center=true);
    translate ([$l/2,-1.7,18]) cube([2,26,3],center=true);

}

//gaten schroeven
translate ([0,7,0]) difference() {
    union () {
        hull() {
            translate ([$l/2-3, $w/2-3,14.9]) cylinder(d=10,h=12,center=true);
            translate ([$l/2-1, $w/2-1,0]) cylinder(d=1,h=1,center=true);
        }
        hull() {
            translate ([$l/2-3, -($w/2-3),14.9]) cylinder(d=10,h=12,center=true);
            translate ([$l/2-1, -($w/2-1),0]) cylinder(d=1,h=1,center=true);
        }
        hull() {
            translate ([-($l/2-3), -($w/2-3),14.9]) cylinder(d=10,h=12,center=true);
            translate ([-($l/2-1), -($w/2-1),0]) cylinder(d=1,h=1,center=true);
        }
        hull() {
            translate ([-($l/2-3), ($w/2-3),14.9]) cylinder(d=10,h=12,center=true);
            translate ([-($l/2-1), ($w/2-1),0]) cylinder(d=1,h=1,center=true);
        }
    }
    union () {
        translate ([$l/2-4, $w/2-4,11]) cylinder(d=2.8,h=20,center=true);
        translate ([-($l/2-4), $w/2-4,11]) cylinder(d=2.8,h=20,center=true);
        translate ([$l/2-4, -($w/2-4),11]) cylinder(d=2.8,h=20,center=true);
        translate ([-($l/2-4), -($w/2-4),11]) cylinder(d=2.8,h=20,center=true);
    }

}
//deksel 3mm dik
union() {
translate ([0,7,9.5]) rotate([0,0,180]) difference() {
    union() {
//delsel opdek deel
        translate ([0,0,1]) hull() {
            translate ([($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=2,center=true);
            translate ([($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=2,center=true);
            translate ([-($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=2,center=true);
            translate ([-($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d+4,h=2,center=true);
        }   
//deksel ingelaten deel
        hull() {
            translate ([($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d-0.3,h=2,center=true);
            translate ([($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d-0.3,h=2,center=true);
            translate ([-($l-$d)/2, ($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d-0.3,h=2,center=true);
            translate ([-($l-$d)/2, -($w-$d)/2,($h1-$d)/2+6]) cylinder(d=$d-0.3,h=2,center=true);
        }

    }
//gaten schroeven
    
    translate ([$l/2-4, $w/2-4,13]) cylinder(d1=3.2,d2=8,h=3.1,center=true);
    translate ([-($l/2-4), $w/2-4,13]) cylinder(d1=3.2,d2=8,h=3.1,center=true);
    translate ([$l/2-4, -($w/2-4),13]) cylinder(d1=3.2,d2=8,h=3.1,center=true);
    translate ([-($l/2-4), -($w/2-4),13]) cylinder(d1=3.2,d2=8,h=3.1,center=true);

//rotary encoders
    translate ([15,14,13]) cylinder(d=7,h=3.1,center=true);

//venster scherm
    translate ([-21.2,8.7,11.5]) hull () {
        cube([26,16,0.1],center=true);
        translate ([0,0,3.1]) cube([30,20,0.1],center=true);
    }
//gat joystick
    translate ([25.2,-19.3,13]) cylinder (d1=12,d2=16,h=3.1,center=true);
//gaten drukknoppen
    translate ([-5.8,-19.3,13]) cylinder (d=12.4,h=3.1,center=true);
    translate ([-22.8,-19.3,13]) cylinder (d=12.4,h=3.1,center=true);
    translate ([-39.8,-19.3,13]) cylinder (d=12.4,h=3.1,center=true);
 //uitspring usb en knopjes
    translate ([-$l/2+2.8,8.7,12]) cube([10,10,3],center=true);
    translate ([-$l/2+5,8.7,12]) cube([6,22,3],center=true);

}
//pilaren joystick
translate ([26.2,-13.3,14.8]) union() {
    translate ([13,9.75,0]) difference () {
        cylinder (d=8,h=12.6,center=true);
        cylinder (d=2.8,h=13.1,center=true);
    }
    translate ([13,-9.75,0]) difference () {
        cylinder (d=8,h=12.6,center=true);
        cylinder (d=2.8,h=13.1,center=true);
    }
    translate ([-13,-9.75,0]) difference () {
        cylinder (d=8,h=12.6,center=true);
        cylinder (d=2.8,h=13.1,center=true);
    }
    translate ([-13,9.75,0]) difference () {
        cylinder (d=8,h=12.6,center=true);
        cylinder (d=2.8,h=13.1,center=true);
    }
}

//pilaren printplaat
translate ([0,7,13.8]) union() {
    translate ([22.3,24.6,0]) difference () {
        cylinder (d=8,h=14.5,center=true);
        cylinder (d=2.8,h=14.6,center=true);
    }
    translate ([22.3,2.6,0]) difference () {
        cylinder (d=8,h=14.5,center=true);
        cylinder (d=2.8,h=14.6,center=true);
    }
    translate ([2.9,8.7,0]) difference () {
        cylinder (d=8,h=14.5,center=true);
        cylinder (d=2.8,h=14.6,center=true);
    }
    translate ([-41.2,-9.3,0]) difference () {
        cylinder (d=8,h=14.5,center=true);
        cylinder (d=2.8,h=14.6,center=true);
    }
    translate ([-41.3,24.6,0]) difference () {
        cylinder (d=8,h=14.5,center=true);
        cylinder (d=2.8,h=14.6,center=true);
    }
}
}

//    translate ([0,7,13]) cube ([110,60,18], center=true);
//    translate ([0,0,-7]) rotate([0,90,0]) cylinder (d=$dbat,h=110,center=true);
