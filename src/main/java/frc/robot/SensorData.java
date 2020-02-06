package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SensorData implements Loggable {
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);
    public static void resetAngle() {
        gyro.reset();
    }

    // returns gyro "angle"
    @Log
    public static double getYaw() {
        return -gyro.getYaw();
    }

    @Log
    public static double getPitch() {
        return gyro.getPitch();
    }
}
