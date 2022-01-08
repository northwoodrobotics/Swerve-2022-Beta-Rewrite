package ExternalLib.JackInTheBotLib.control;

import ExternalLib.JackInTheBotLib.math.Rotation2;
import ExternalLib.JackInTheBotLib.math.Vector2;
import ExternalLib.JackInTheBotLib.math.spline.CubicBezierSpline;
import ExternalLib.JackInTheBotLib.math.spline.CubicHermiteSpline;
import ExternalLib.JackInTheBotLib.math.spline.Spline;
import ExternalLib.NorthwoodLib.MathWrappers.NWRotation2d;
import ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

import java.util.*;

public final class SplinePathBuilder {
    private List<PathSegment> segmentList = new ArrayList<>();
    private Map<Double, NWRotation2d> rotationMap = new TreeMap<>();
    private double length = 0.0;

    private PathSegment.State lastState;

    public SplinePathBuilder(NWTranslation2d initialPosition, NWRotation2d initialHeading, NWRotation2d initialRotation) {
        lastState = new PathSegment.State(initialPosition, initialHeading, 0.0);
        rotationMap.put(0.0, initialRotation);
    }

    private void addSpline(Spline spline) {
        SplinePathSegment segment = new SplinePathSegment(spline);
        segmentList.add(segment);
        lastState = segment.getEnd();
        length += segment.getLength();
    }

    public Path build() {
        return new Path(segmentList.toArray(new PathSegment[0]), rotationMap);
    }

    public SplinePathBuilder bezier(NWTranslation2d controlPoint1, NWTranslation2d controlPoint2, NWTranslation2d end) {
        addSpline(new CubicBezierSpline(
                lastState.getPosition(),
                controlPoint1,
                controlPoint2,
                end
        ));
        return this;
    }

    public SplinePathBuilder bezier(NWTranslation2d controlPoint1, NWTranslation2d controlPoint2, NWTranslation2d end, NWRotation2d rotation) {
        bezier(controlPoint1, controlPoint2, end);
        rotationMap.put(length, rotation);
        return this;
    }

    public SplinePathBuilder hermite(NWTranslation2d position, NWRotation2d  heading) {
        addSpline(new CubicHermiteSpline(
                lastState.getPosition(), lastState.getHeading(),
                position, heading
        ));
        return this;
    }

    public SplinePathBuilder hermite(NWTranslation2d position, NWRotation2d heading, NWRotation2d  rotation) {
        hermite(position, heading);
        rotationMap.put(length, rotation);
        return this;
    }
}
