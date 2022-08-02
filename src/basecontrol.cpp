#include "main.h"

void Base::create(
    int l1, int l2, int l3, int l4, int r1, int r2, int r3, int r4, //motor declaration
    char le, char re, char me, int inertial, //encd declaration
    double wd, double bw, //wd is wheeldiameter and bw is Basewidth
    double mr, //mr is max rpm (rpm x gearing)
    double kps, double kis, double kds, //pid straight declaration
    double kpt, double kit, double kdt, //pid turn declaration
    double beta, double zeta //ramsete terms declaration
) {
    if (l1 != 0) {Base::left1 = l1;}
    if (l2 != 0) {Base::left2 = l2;}
    if (l3 != 0) {Base::left3 = l3;}
    if (l4 != 0) {Base::left4 = l4;}
    if (r1 != 0) {Base::right1 = r1;}
    if (r2 != 0) {Base::right2 = r2;}
    if (r3 != 0) {Base::right3 = r3;}
    if (r4 != 0) {Base::right4 = r4;}
    MotorGroup bleft ({left1, left2, left3, left4});
    MotorGroup bright ({right1, right2, right3, right4});
    Base::wd = wd; Base::bw = bw; Base::mr = mr;

    Base::kps = kps; Base::kis = kis; Base::kds = kds; 
    Base::kpt = kpt; Base::kit = kit; Base::kdt = kdt;  
    Base::beta = beta; Base::zeta = zeta;
    
    if (le == 0 && re == 0 && me == 0 && inertial == 0) Base::tracktype = motorencd;
    else if (le == 0 && re == 0 && me == 0 && inertial != 0) Base::tracktype = motorencdi;
    else if (le != 0 && re != 0 && me == 0 && inertial == 0) Base::tracktype = twowheel;
    else if (le != 0 && re != 0 && me == 0 && inertial != 0) Base::tracktype = twowheeli;
    else if (le != 0 && re != 0 && me != 0 && inertial == 0) Base::tracktype = threewheel;
    Base::le = le; Base::re = re; Base::me = me; Base::inertial = inertial;
}

void Base::PIDengine() {
}

void Base::odomengine() {
}

void Base::start() {
    pros::Task PIDengine ([this] {
        this -> Base::PIDengine();
    } );
    pros::Task odomengine ([this] {
        this -> Base::odomengine();
    } );
}

Base Bot;