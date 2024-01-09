package frc.lib.math;

public class NEOConversions {
    /**
     * @param revolutions    NEO Revolutions
     * @param gearRatio Gear Ratio between NEO and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double NEOToDegrees(double revolutions, double gearRatio) {
        return revolutions * (360 / gearRatio);
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between NEO and Mechanism
     * @return NEO rotations
     */
    public static double degreesToNEO(double degrees, double gearRatio) {
        return degrees / (360 / gearRatio);
    }

    /**
     * @param NEORPM         NEO RPM
     * @param gearRatio      Gear Ratio between NEO and Mechanism
     * @return RPM of Mechanism
     */
    public static double NEOToMechanismRPM(double NEORPM, double gearRatio) {
        return NEORPM / gearRatio;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between NEO and Mechanism
     * @return RPM of NEO
     */
    public static double MechanismRPMToNEO(double RPM, double gearRatio) {
        return RPM * gearRatio;
    }

    /**
     * @param NEORPM         NEO RPM
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between NEO and Mechanism 
     * @return Meters Per Second
     */
    public static double NEOToMPS(double NEORPM, double circumference, double gearRatio) {
        double wheelRPM = NEOToMechanismRPM(NEORPM, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between NEO and Mechanism (set to 1 for
     *                      NEO MPS)
     * @return NEO RPM
     */
    public static double MPSToNEO(double velocity, double circumference, double gearRatio){
        double wheelRPM = (velocity * 60) / circumference;
        return MechanismRPMToNEO(wheelRPM, gearRatio);
    }

    /**
     * @param rotations      NEO rotations
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between NEO and Wheel
     * @return Meters
     */
    public static double NEOToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * (circumference / gearRatio);
    }

    /**
     * @param meters        Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between NEO and Wheel
     * @return NEO Revolutions
     */
    public static double metersToNEO(double meters, double circumference, double gearRatio) {
        return meters / (circumference / gearRatio);
    }
}
