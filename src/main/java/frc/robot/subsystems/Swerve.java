package frc.robot.subsystems;


import com.ctre.phoenix6.SignalLogger;
import frc.robot.SwerveModule;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Power;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    //for logging pose data during simulation
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();

    // StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    //     .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish(); for multiple poses

    private SwerveDrivePoseEstimator swerveOdometry;
    private Pigeon2 gyro;
    private SwerveModule[] mSwerveMods ;
    public Supplier<Pose2d> poseSupplier = () -> getPose();
    public Consumer<Pose2d> poseConsumer = a -> {resetOdometry(a);};
    public BooleanSupplier inPosition = () -> inPosition();
    public Consumer<ChassisSpeeds> chassisConsumer = a -> {
        autoDrive(a, true);
    };
    public BooleanSupplier isPathRunningSupplier = () -> pathInProgress();
    public Pose2d goalPose = new Pose2d();
    public double xTolerance = 1.0;
    public double yTolerance = 1.0;
    public double rotTolerance = 30;

    private final MutableMeasure<Voltage> m_appliedVoltage = edu.wpi.first.units.MutableMeasure.mutable(edu.wpi.first.units.Units.Volts.of(0));
    private final MutableMeasure<Distance> m_distance = edu.wpi.first.units.MutableMeasure.mutable(edu.wpi.first.units.Units.Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = edu.wpi.first.units.MutableMeasure.mutable(edu.wpi.first.units.Units.MetersPerSecond.of(0));

    SysIdRoutine routine;

    //TODO: kalman filters here

    // initializes the swerve modules
    public Swerve() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());



        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        swerveOdometry = 
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        routine = 
        new SysIdRoutine(
            // empty config defaults to 1 volt/second ramp rate and 7 volt step voltage
            new SysIdRoutine.Config(null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    for (SwerveModule mod : mSwerveMods) {
                        mod.setDriveVoltage(volts.in(edu.wpi.first.units.Units.Volts));
                    }
                },
                log -> {
                    for (SwerveModule mod : mSwerveMods) {
                        log.motor("Mod " + mod.moduleNumber)
                            .voltage(
                                m_appliedVoltage.mut_replace(
                                    mod.getDriveMotor().get() * RobotController.getBatteryVoltage(), edu.wpi.first.units.Units.Volts))
                            .linearPosition(m_distance.mut_replace(mod.getDistance(), edu.wpi.first.units.Units.Meters))
                            .linearVelocity(m_velocity.mut_replace(mod.getVelocity(), edu.wpi.first.units.Units.MetersPerSecond));
                    }
                } , this));
    }

        // used to apply any driving movement to the swerve modules
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        Math.copySign(Math.pow(translation.getX(), 2), translation.getX()), // square inputs for robert testing
                        Math.copySign(Math.pow(translation.getY(), 2), translation.getY()), // square inputs for robert testing
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

        // never used but might be useful
    public void autoDrive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * 
     * @return pose of the robot
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * 
     * @param pose resets the on-robot odometry to the given pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
        gyro.setYaw(pose.getRotation().getDegrees());
    }

    /**
     * 
     * @return gets the state of the swerve modules
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * 
     * @return gets the voltage and current of the swerve modules for simulation purposes. Doesn't work atm.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    public Object[] getModulePowers(){
        Object[] powers = new Object[4];
        for(SwerveModule mod : mSwerveMods){
            powers[mod.moduleNumber] = mod.getAnglePower();
        }
        return powers;
    } 
    
    /**
     * sets gyro's yaw to zero
     */
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /**
     * 
     * @return the yaw from the gyro (Pidgeon 2)
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(normalize(gyro.getYaw().getValue()));
    }

    /**
     * 
     * @return gets the pitch from the gyro (Pidgeon 2)
     */
    public double getPitch() {
        return gyro.getRoll().getValueAsDouble();
    }

    // gyro reports angle cumalitvely not in 360s so this method gets the 360 
    // rotation and tranforms it to the 180 to -180 rotation that is used by the
    // pose class in WPIlib
    /**
     * gyro reports angle cumalitvely not in 360s so this method gets the 360 
     * rotation and tranforms it to the 180 to -180 rotation that is used by the
     * pose class in WPIlib
     * @param deg degree from gyro
     * @return normalized degrees for WPIlib use
     */
    public static double normalize(double deg) {
        double angle = deg % 360;
        if (angle < -180) {
            angle = 180 - (Math.abs(angle) - 180);
        } else if (angle > 180) {
            angle = -180 + (Math.abs(angle) - 180);
        }
        return angle;
    }

    /**
     * 
     * @return the positions of the swerve modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * 
     * @return whether the swerve is being used for an auto generated path like OTF or auto
     */
    public boolean pathInProgress() {
        return !getDefaultCommand().isScheduled();
    }   

    // these two methods below are used in auto to set the parameters for the goal pose range and execute commands in
    // auto when the robot gets within that range of the goal pose
    public void goalPoseParameters(Pose2d goalPose, double xTolerance, double yTolerance, double rotTolerance) {
        this.goalPose = goalPose;
        this.xTolerance = xTolerance;
        this.yTolerance = yTolerance;
    }

    /**
     * 
     * @return if the robot is close enough to the desired 2d pose. Used in autonomous
     */
    public boolean inPosition() {
        return (Math.abs(getPose().getX() - goalPose.getX()) < xTolerance)
                && (Math.abs(getPose().getY() - goalPose.getY()) < yTolerance)
                && (Math.abs(getPose().getRotation().getDegrees()
                        - goalPose.getRotation().getDegrees()) < rotTolerance);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Voltage", mod.getDrivePower().voltage);

        }
        SmartDashboard.putBoolean("True", false);
        SmartDashboard.putNumber("Can Coder", mSwerveMods[0].angleEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon", getYaw().getDegrees());
        SmartDashboard.updateValues();

        swerveOdometry.update(getYaw(), getModulePositions());
    //   Pose2d currPose = getPose();
    //   publisher.set(currPose);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        Pose2d currPose = getPose();
        publisher.set(currPose);
        swerveOdometry.update(getYaw(), getModulePositions());
    }
}
