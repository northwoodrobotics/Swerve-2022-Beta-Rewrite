package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import ExternalLib.JackInTheBotLib.math.RigidTransform2;
import ExternalLib.JackInTheBotLib.math.Rotation2;
import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.NorthwoodLib.MathWrappers.NWPose2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeDrivetrainCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    private final NetworkTableEntry autoSpeedEntry =
            NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdAutoSpeed");
    private final NetworkTableEntry telemetryEntry =
            NetworkTableInstance.getDefault().getEntry("/SmartDashboard/SysIdTelemetry");

    private List<Double> telemetryData = new ArrayList<>();


    private double priorAutospeed = 0.0;

    public CharacterizeDrivetrainCommand(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetGyroAngle(NWRotation2d.ZERO);
        drivetrain.resetPose(NWPose2d.ZERO);

        NetworkTableInstance.getDefault().setUpdateRate(10.0e-3);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double position = drivetrain.getPose().getTranslation().getX();
        double velocity = drivetrain.getVelocity().getX();

        double battery = RobotController.getBatteryVoltage();
        double motorVoltage = battery * Math.abs(priorAutospeed);

        double autospeed = autoSpeedEntry.getDouble(0.0);
        priorAutospeed = autospeed;

        drivetrain.drive(new Translation2d(autospeed, 0.0), 0.0, false);

        telemetryData.add(now);
        telemetryData.add(autospeed * RobotController.getInputVoltage());
        telemetryData.add(position);
        telemetryData.add(velocity);

    }

    @Override
    public void end(boolean interrupted) {
        StringBuilder b = new StringBuilder();
        for (int i = 0; i < telemetryData.size(); ++i) {
            if (i != 0)
                b.append(", ");
            b.append(telemetryData.get(i));
        }

        telemetryEntry.setString(b.toString());

        drivetrain.drive(NWTranslation2d.ZERO, 0.0, false);
    }
}