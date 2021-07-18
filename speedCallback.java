package org.firstinspires.ftc.teamcode;

public interface speedCallback {
    int speed();
    boolean Running();
    void canFire(boolean can, int speed);
    boolean fire();
}
