package cataclysm.constraints;

/**
 * Repr�sente une contraine sur les rotations d'un solide.
 * @author Briac
 *
 */
public abstract class AngularConstraint extends SimpleConstraint {

	public AngularConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
	}

}
