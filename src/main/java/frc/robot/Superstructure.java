package frc.robot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Superstructure {
    public Superstructure() {}

    public double getCurrentPressure() {
        double busVoltage = RobotController.getVoltage5V();

        return 250.0 * (busVoltage) - 25.0;
    }

}
