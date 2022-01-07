package ExternalLib.NorthwoodLib.MathWrappers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import ExternalLib.JackInTheBotLib.util.Interpolable;

import java.io.Serializable;


public class NWRotation2d extends Rotation2d implements Serializable, Interpolable<NWRotation2d>{
    public static final NWRotation2d ZERO = new NWRotation2d(0.0);






    public NWRotation2d(double value){
       super(value);

    }

    public NWRotation2d(double x, double y){
        super(x, y);
    }





   
    





    public NWRotation2d inverse(NWRotation2d flip) {
        NWRotation2d inverse = (NWRotation2d) flip.unaryMinus();
        return inverse;
        
    }
    @Override
    public NWRotation2d interpolate(NWRotation2d other, double t) {
        if (t <= 0.0) {
            return this;
        } else if (t >= 1.0) {
            return other;
        }
        

        double from;
        from = getRadians();

        double to = other.getRadians();

        double diff = Math.abs(from - to);
        if (diff > Math.PI) {
            if (from < to) {
                from += 2 * Math.PI;
            } else {
                to += 2 * Math.PI;
            }
        }

        return (NWRotation2d) NWRotation2d.fromDegrees(from + ((to - from) * t));
    }





    
}
