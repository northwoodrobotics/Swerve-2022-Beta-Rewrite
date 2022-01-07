package ExternalLib.JackInTheBotLib.util;

import ExternalLib.JackInTheBotLib.math.Vector2;
import edu.wpi.first.math.geometry.Translation2d;

public class HolonomicDriveSignal {
    private final Translation2d translation;
    private final double rotation;
    private final boolean fieldOriented;

    public HolonomicDriveSignal(Translation2d translation, double rotation, boolean fieldOriented) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isFieldOriented() {
        return fieldOriented;
    }
}
