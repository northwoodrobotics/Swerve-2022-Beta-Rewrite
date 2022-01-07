package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.JackInTheBotLib.robot.input.Axis;
import ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase{
    private DrivetrainSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }
        @Override
        public void execute() {
            drivetrainSubsystem.drive(new NWTranslation2d(forward.get(true), strafe.get(true)), rotation.get(true), true);
        }
    
        @Override
        public void end(boolean interrupted) {
            drivetrainSubsystem.drive(NWTranslation2d.ZERO, 0, false);
        }
    

    
}
