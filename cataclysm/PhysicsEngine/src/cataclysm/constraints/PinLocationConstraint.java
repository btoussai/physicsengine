package cataclysm.constraints;

/**
 * Cette contrainte bloque entièrement les translations entre deux solides. La
 * rotation reste libre. Plus précisément, il s'agit d'une
 * {@link DistanceConstraint} avec une longueur de zéro.
 * 
 * @author Briac
 *
 */
public class PinLocationConstraint extends DistanceConstraint {

	/**
	 * Cette contrainte bloque entièrement les translations entre deux solides. La
	 * rotation reste libre.
	 * 
	 * @param pointA
	 * @param pointB
	 */
	public PinLocationConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		this.setLength(0);
	}

}
