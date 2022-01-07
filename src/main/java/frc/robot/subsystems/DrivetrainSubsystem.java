package frc.robot.subsystems;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;




import ExternalLib.JackInTheBotLib.control.*;
import ExternalLib.JackInTheBotLib.drivers.Gyroscope;
import ExternalLib.JackInTheBotLib.kinematics.ChassisVelocity;
import ExternalLib.JackInTheBotLib.kinematics.SwerveKinematics;
import ExternalLib.JackInTheBotLib.kinematics.SwerveOdometry;
import ExternalLib.JackInTheBotLib.math.RigidTransform2;
import ExternalLib.JackInTheBotLib.math.Rotation2;
import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.JackInTheBotLib.robot.UpdateManager;
import ExternalLib.JackInTheBotLib.util.*;

import ExternalLib.NorthwoodLib.MathWrappers.*;
import java.util.Optional;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {

    public static final double TRACKWIDTH = 1.0;
    public static final double WHEELBASE = 1.0;


    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
        0.042746,
        0.0032181,
        0.30764
        );
        
    private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;
    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
        new PidConstants(0.4, 0.0, 0.025),
        new PidConstants(5.0, 0.0, 0.0),
        new HolonomicFeedforward(FEEDFORWARD_CONSTANTS)
);



    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
        new NWTranslation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         //front left
        new NWTranslation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        //front right
        new NWTranslation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),       //back left
        new NWTranslation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        //back right
);
private final SwerveDriveKinematics wpi_driveKinematics = new SwerveDriveKinematics(
    new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), //front left
    new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), //front right
    new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // back left
    new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
);



private final SwerveModule[] modules;




private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private final AHRS gyroscope = new AHRS(SPI.Port.kMXP);
    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, NWPose2d.ZERO);
    @GuardedBy("kinematicsLock")
    private NWPose2d pose = NWPose2d.ZERO;
    @GuardedBy("kinematicsLock")
    private final InterpolatingTreeMap<InterpolatingDouble, NWPose2d> latencyCompensationMap = new InterpolatingTreeMap<>();
    @GuardedBy("kinematicsLock")
    private NWTranslation2d velocity = NWTranslation2d.ZERO;
    @GuardedBy("kinematicsLock")
    private double angularVelocity = 0.0;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;



    public DrivetrainSubsystem() {
        synchronized (sensorLock) {
            
        }




ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
    tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withPosition(2, 0)
        .withSize(2, 4),
    Mk4SwerveModuleHelper.GearRatio.L2,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET
        );
        
SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
    tab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withPosition(4, 0)
        .withSize(2, 4),
    Mk4SwerveModuleHelper.GearRatio.L2,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
        Constants.DrivetrainIDs.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET
        );
SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
    tab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withPosition(6, 0)
        .withSize(2, 4),
    Mk4SwerveModuleHelper.GearRatio.L2,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET
        );
SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
    tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withPosition(8, 0)
        .withSize(2, 4),
    Mk4SwerveModuleHelper.GearRatio.L2,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
        Constants.DrivetrainIDs.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET
        );

    modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    odometryXEntry = tab.add("X", 0.0)
    .withPosition(0, 0)
    .withSize(1, 1)
    .getEntry();
odometryYEntry = tab.add("Y", 0.0)
    .withPosition(0, 1)
    .withSize(1, 1)
    .getEntry();
odometryAngleEntry = tab.add("Angle", 0.0)
    .withPosition(0, 2)
    .withSize(1, 1)
    .getEntry();

    tab.addNumber("Rotation Voltage", () -> {
        HolonomicDriveSignal signal;
        synchronized (stateLock) {
            signal = driveSignal;
        }

        if (signal == null) {
            return 0.0;
        }

        return signal.getRotation() * RobotController.getBatteryVoltage();
    });

    tab.addNumber("Average Velocity", this::getAverageAbsoluteValueVelocity);
    }
    

    
    public NWPose2d getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public NWTranslation2d getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    
    public void drive(Translation2d translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

        public void resetPose(NWPose2d pose) {
            synchronized (kinematicsLock) {
                this.pose = pose;
                swerveOdometry.resetPose(pose);
            }
        }
    
        public void resetGyroAngle(Rotation2d angle) {
            synchronized (sensorLock) {
                gyroscope.reset();

            }
        }
        public double getAverageAbsoluteValueVelocity() {
            double averageVelocity = 0;
            for (var module : modules) {
                averageVelocity += Math.abs(module.getDriveVelocity());
            }
            return averageVelocity / 4;
        }
    
        private void updateOdometry(double time, double dt) {
            Vector2[] moduleVelocities = new Vector2[modules.length];
            for (int i = 0; i < modules.length; i++) {
                var module = modules[i];
    
                moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getSteerAngle())).scale(module.getDriveVelocity() * 39.37008);
            }
    
            Rotation2d angle;
            double angularVelocity;
            synchronized (sensorLock) {
                angle = NWRotation2d.fromDegrees(gyroscope.getAngle());
                angularVelocity = gyroscope.getRate();
            }
    
            ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);
    
            synchronized (kinematicsLock) {
    
                this.pose = (NWPose2d) swerveOdometry.update((NWRotation2d) angle, dt, moduleVelocities);
                if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                    latencyCompensationMap.remove(latencyCompensationMap.firstKey());
                }
                latencyCompensationMap.put(new InterpolatingDouble(time), pose);
                this.velocity = (NWTranslation2d) velocity.getTranslationalVelocity();
                this.angularVelocity = angularVelocity;
            }
        }
    
        private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
            ChassisVelocity chassisVelocity;
            if (driveSignal == null) {
                chassisVelocity = new ChassisVelocity(NWTranslation2d.ZERO, 0.0);
            } else if (driveSignal.isFieldOriented()) {
                chassisVelocity = new ChassisVelocity(
                        (NWTranslation2d) driveSignal.getTranslation().rotateBy(getPose().getRotation()),
                        driveSignal.getRotation()
                );
            } else {
                chassisVelocity = new ChassisVelocity(
                        (NWTranslation2d) driveSignal.getTranslation(),
                        driveSignal.getRotation()
                );
            }
    
            NWTranslation2d[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
            SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
            for (int i = 0; i < moduleOutputs.length; i++) {
                var module = modules[i];
                module.set(moduleOutputs[i].length * 12.0, moduleOutputs[i].getAngle().getRadians());
            }
        }
    
        public Pose2d getPoseAtTime(double timestamp) {
            synchronized (kinematicsLock) {
                if (latencyCompensationMap.isEmpty()) {
                    return NWPose2d.ZERO;
                }
                return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
            }
        }
    
        @Override
        public void update(double time, double dt) {
            updateOdometry(time, dt);
    
            HolonomicDriveSignal driveSignal;
            Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                    getPose(),
                    getVelocity(),
                    getAngularVelocity(),
                    time,
                    dt
            );
            if (trajectorySignal.isPresent()) {
                driveSignal = trajectorySignal.get();
                driveSignal = new HolonomicDriveSignal(
                        driveSignal.getTranslation().times(1.0 / RobotController.getBatteryVoltage()),
                        driveSignal.getRotation() / RobotController.getBatteryVoltage(),
                        driveSignal.isFieldOriented()
                );
            } else {
                synchronized (stateLock) {
                    driveSignal = this.driveSignal;
                }
            }
    
            updateModules(driveSignal, dt);
        }
    
        @Override
        public void periodic() {
            NWPose2d pose = getPose();
            odometryXEntry.setDouble(pose.getX());
            odometryYEntry.setDouble(pose.getY());
            odometryAngleEntry.setDouble(getPose().getRotation().getDegrees());
        }
    
        public HolonomicMotionProfiledTrajectoryFollower getFollower() {
            return follower;
        }





        
    

    








    
}
