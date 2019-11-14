package cataclysm.constraints;

/**
 * Représente un groupement de {@link SimpleConstraint} partageant les mêmes points d'ancrage.
 * @author Briac
 *
 */
public abstract class CompoundConstraint extends AbstractConstraint{

	public CompoundConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		if (pointA.getConstraint() != this || pointB.getConstraint() != this) {
				throw new IllegalStateException("Erreur, ce point d'ancrage appartient déjà à une contrainte.");
		}
	}

}
