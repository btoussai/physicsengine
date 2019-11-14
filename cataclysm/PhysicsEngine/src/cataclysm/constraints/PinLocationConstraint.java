package cataclysm.constraints;

/**
 * Cette contrainte bloque enti�rement les translations entre deux solides. La
 * rotation reste libre. Plus pr�cis�ment, il s'agit d'une
 * {@link DistanceConstraint} avec une longueur de z�ro.
 * 
 * @author Briac
 *
 */
public class PinLocationConstraint extends DistanceConstraint {

	/**
	 * Cette contrainte bloque enti�rement les translations entre deux solides. La
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
