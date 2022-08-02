#ifndef _BASECONTROL_HPP_
#define _BASECONTROL_HPP_

//the point of creating a base class is for constructors and movement controls 
//in effect, all "generated" velocities will be passed to basecontrol to apply to motors

enum btracktype {twowheel, twowheeli, threewheel, motorencd, motorencdi};

class Base {
    protected:

    int left1, left2, left3, left4, right1, right2, right3, right4;
    char le, re, me; int inertial;
    int wd, bw, mr;
    double kps, kis, kds, kpt, kit, kdt;
    double beta, zeta;

    btracktype tracktype;

    public:

    void create(
        int l1, int l2, int l3, int l4, int r1, int r2, int r3, int r4, //motor declaration
        char le, char re, char me, int inertial, //encd declaration
        double wd, double bw, //wd is wheeldiameter and bw is basewidth
        double mr, //mr is max rpm (rpm x gearing)
        double kps, double kis, double kds, //pid straight declaration
        double kpt, double kit, double kdt, //pid turn declaration
        double beta, double zeta
    );

    void PIDengine();

    void odomengine();

    void start();
};

#endif