$fn=150;

intersection() {
    translate ([0,0,10]) cylinder (d=16,h=3,center=true);
    translate ([0,0,3]) sphere(d=20);
}

translate ([0,0,5]) difference() {
    cylinder (d1=7,d2=10,h=8,center=true);
    translate ([0,0,-1]) cylinder (d1=4.4,d2=4.2,h=6.1,center=true);
}
