package ExternalLib.NorthwoodLib.MathWrappers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import ExternalLib.JackInTheBotLib.util.Interpolable;

import java.io.Serializable;
import java.text.DecimalFormat;
import java.util.Objects;


public final class NWTranslation2d extends Translation2d implements Serializable, Interpolable<NWTranslation2d>{
    public static final NWTranslation2d ZERO = new NWTranslation2d(0.0, 0.0);
    double L;
    

    public NWTranslation2d(double x,double y){
        super(x, y);

    }









    





    @Override 
    public NWTranslation2d interpolate(NWTranslation2d other, double t){
        if (t <= 0.0) {
			return this;
		} else if (t >= 1.0) {
			return other;
		} else {
			return extrapolate(other, t);
		}
    }
  


    public NWTranslation2d extrapolate(NWTranslation2d other, double t){
        NWTranslation2d delta = (NWTranslation2d) other.minus(this);
        return (NWTranslation2d) this.plus((delta.times(t)));
    }









    
}
