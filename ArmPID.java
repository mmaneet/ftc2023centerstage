package org.firstinspires.ftc.teamcode;

public class ArmPID {

    public double target;
    private double cPower;

    public ArmPID(double encp, double powp) {
        target = encp;
        cPower = powp;
    }

    private boolean isMoving() {
        return (cPower != 0);
    }

    public int updatePID(int encp, double powp) {
        //if power = 0, keep the encoder value static at the last position. set the enc value to the last thing it was when moving
        //if power != 0, change enc target by power
        cPower = powp;
        if (encp < -3000 || encp > 3000) {
            return encp;
        }
        if (isMoving()) {
            target += cPower * 100;
        }
        return (int) target;

    }
}

