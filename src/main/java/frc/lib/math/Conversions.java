

package frc.lib.math;

public class Conversions {

    /**
     * @param positionCounts CANCoder Rotations
     * @param gearRatio      Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Rotations
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    /**
     * @param rotations Falcon Rotations
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double rotations, double gearRatio) {
        return rotations * (360.0 / gearRatio);
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Rotations
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    /**
     * @param FalconRPM      Falcon RPM
     * @param gearRatio      Gear Ratio between Falcon and Mechanism
     * @return RPM of Mechanism
     */
    public static double falconToWheelRPM(double FalconRPM, double gearRatio) {
        return FalconRPM / gearRatio;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return RPM of Falcon
     */
    public static double wheelRPMToFalcon(double RPM, double gearRatio) {
        return RPM * gearRatio;
    }

    /**
     * @param FalconRPM      Falcon RPM
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism
     * @return Meters Per Second
     */
    public static double falconToMPS(double FalconRPM, double circumference, double gearRatio) {
        double wheelRPM = falconToWheelRPM(FalconRPM, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism
     * @return Falcon RPM
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        return wheelRPMToFalcon(wheelRPM, gearRatio);
    }

    /**
     * @param rotations      Falcon Rotations
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * (circumference / gearRatio);
    }

    /**
     * @param meters        Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Wheel
     * @return Falcon Rotations
     */
    public static double MetersToFalcon(double meters, double circumference, double gearRatio) {
        return meters / (circumference / gearRatio);
    }
}