package cataclysm.constraints;

import cataclysm.wrappers.RigidBody;
import math.vector.Vector3f;

/**
 * Repr�sente une contrainte entre deux solides ou entre un solide et un point
 * fixe en world-space. <br>
 * 
 * @author Briac
 *
 */
public abstract class AbstractConstraint{

	/**
	 * Le point d'ancrage de la contrainte sur le solide A.
	 */
	protected final AnchorPoint pointA;

	/**
	 * Le point d'ancrage de la contrainte sur le solide B.
	 */
	protected final AnchorPoint pointB;

	/**
	 * Indique si les deux corps rigides li�s par la contrainte peuvent entrer en
	 * collision ou non.
	 */
	private boolean collideConnected = false;

	
	public AbstractConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		this.pointA = pointA;
		this.pointB = pointB;

		if (pointA.isStatic() && pointB.isStatic()) {
			throw new IllegalArgumentException(
					"Erreur, les deux points d'ancrage de la contrainte appartiennent au décor.");
		}
		if (pointA.getBody() == pointB.getBody()) {
			throw new IllegalArgumentException(
					"Erreur, les deux points d'ancrage de la contrainte appartiennent au même corps rigide");
		}

		this.pointA.setConstraint(this);
		this.pointB.setConstraint(this);
	}

	/**
	 * R�sout les erreurs de vitesse pour la contrainte.
	 * 
	 * @param firstIteration
	 */
	protected abstract void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp);

	/**
	 * R�sout les erreurs de position pour la contrainte, la fonction n'est appel�e
	 * que si {@link #hasPositionCorrection()} retourne true.
	 * 
	 * @param firstIteration
	 */
	protected abstract void solvePosition(boolean firstIteration, float timeStep, Vector3f temp);

	/**
	 * Indique si la contrainte n�cessite une �tape de r�solution au niveau de la
	 * position.
	 * 
	 * @return
	 */
	protected abstract boolean hasPositionCorrection();

	/**
	 * @return true si les solides connect�s par la contrainte peuvent entrer en
	 *         collision.
	 */
	public boolean shouldCollide() {
		return collideConnected;
	}

	/**
	 * Change le comportement des solides reli�s par la contrainte.
	 * 
	 * @param collideConnected true: les solides peuvent entrer en collision. <br>
	 *                         false: les solides ne peuvent pas g�n�rer de contact
	 *                         entre eux.
	 */
	public void setCollideConnected(boolean collideConnected) {
		this.collideConnected = collideConnected;
	}

	/**
	 * @return Le point d'ancrage de la contrainte sur le solide A.
	 */
	public AnchorPoint getPointA() {
		return pointA;
	}

	/**
	 * @return Le point d'ancrage de la contrainte sur le solide B.
	 */
	public AnchorPoint getPointB() {
		return pointB;
	}

	/**
	 * Teste si cette contrainte relie les solides A et B.
	 * 
	 * @param A
	 * @param B
	 * @return true si la contrainte relie les solides A et B.
	 */
	public boolean checkConnected(RigidBody A, RigidBody B) {
		return (pointA.getBody() == A && pointB.getBody() == B) || (pointB.getBody() == A && pointA.getBody() == B);
	}

}
