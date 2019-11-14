package cataclysm.constraints;

/**
 * Repr�sente un groupement de {@link SimpleConstraint} partageant les m�mes points d'ancrage.
 * @author Briac
 *
 */
public abstract class CompoundConstraint extends AbstractConstraint{

	public CompoundConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		if (pointA.getConstraint() != this || pointB.getConstraint() != this) {
				throw new IllegalStateException("Erreur, ce point d'ancrage appartient d�j� � une contrainte.");
		}
	}

}
