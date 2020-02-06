package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class SensorData {
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);

    public static void resetAngle() {
        gyro.reset();
    }

    // returns gyro "angle"
    public static double getYaw() {
        return -gyro.getYaw();
    }

    public static double getPitch() {
        return gyro.getPitch();
    }
}
