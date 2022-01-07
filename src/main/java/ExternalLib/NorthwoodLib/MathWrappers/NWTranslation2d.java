package ExternalLib.NorthwoodLib.MathWrappers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import ExternalLib.JackInTheBotLib.math.MathUtils;
import ExternalLib.JackInTheBotLib.util.Interpolable;

import java.io.Serializable;
import java.text.DecimalFormat;
import java.util.Objects;


public final class NWTranslation2d extends Translation2d implements Serializable, Interpolable<NWTranslation2d>{
    public static final NWTranslation2d ZERO = new NWTranslation2d(0.0, 0.0);
    double L;

    public final double m_x;
    public final double m_y;
    public final double length;

    

    public NWTranslation2d(double x,double y){
        this.m_x = x;
        this.m_y = y;
        this.length = Math.hypot(x, y);

    }


    public NWTranslation2d add(double x, double y){
        return new NWTranslation2d(this.m_x+x, this.m_y+y);
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



    @Override
    public boolean equals(Object obj){
        if(!(obj instanceof NWTranslation2d)){
            return false;
        }
        return equals((NWTranslation2d) obj, MathUtils.EPSILON);
    }


    public boolean equals(NWTranslation2d other, double allowableError){
        return MathUtils.
    }









    
}
