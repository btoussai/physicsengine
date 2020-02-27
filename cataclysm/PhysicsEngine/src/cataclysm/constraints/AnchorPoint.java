package cataclysm.constraints;

import cataclysm.wrappers.RigidBody;
import math.vector.Vector3f;

/**
 * Repr�sente un point d'ancrage d'une contrainte. Un point d'ancrage peut �tre
 * une coordonn�e sur un RigidBody exprim�e en model-space ou bien une
 * coordonn�e fixe en world-space. Un point d'ancrage ne peut appartenir qu'�
 * une seule contrainte.
 * 
 * @author Briac
 *
 */
public class AnchorPoint {

	private final Vector3f bodySpacePos = new Vector3f();
	private final Vector3f worldSpacePos = new Vector3f();
	private final RigidBody body;
	private final boolean isStatic;
	private AbstractConstraint constraint;

	/**
	 * Construit un nouveau point d'ancrage pour une contrainte. <br>
	 * Ce point d'ancrage est consid�r� comme statique, c'est � dire qu'il ne peut
	 * pas bouger et la position doit �tre exprim�e en world-space.
	 * 
	 * @param position
	 */
	public AnchorPoint(Vector3f position) {
		this.bodySpacePos.set(position);
		this.worldSpacePos.set(position);
		this.body = null;
		this.isStatic = true;
	}

	/**
	 * Construit un nouveau point d'ancrage pour une contrainte. <br>
	 * Ce point d'ancrage est fixe dans le rep�re du solide, mais le solide peut
	 * tr�s bien se d�placer. La position doit �tre exprim�e en model-space.
	 * 
	 * @param position
	 * @param body
	 */
	public AnchorPoint(Vector3f position, RigidBody body) {
		this.bodySpacePos.set(position);
		this.body = body;
		this.isStatic = false;

		if (body == null) {
			throw new NullPointerException(
					"Erreur, cr�ation d'un point d'ancrage dynamique avec un solide valant null.");
		}

		body.vertexToWorldSpace(bodySpacePos, worldSpacePos);
	}

	/**
	 * Ce constructeur permet de dupliquer un point d'ancrage. Ceci permet de
	 * construire une nouvelle contrainte ayant un point d'ancrage localis� au m�me
	 * endroit qu'un contrainte construite pr�c�demment.
	 * 
	 * @param other
	 */
	public AnchorPoint(AnchorPoint other) {
		this.bodySpacePos.set(other.bodySpacePos);
		this.worldSpacePos.set(other.worldSpacePos);
		this.body = other.body;
		this.isStatic = other.isStatic;
	}

	public Vector3f getBodySpacePosition() {
		return bodySpacePos;
	}

	public Vector3f getWorldSpacePosition() {
		return worldSpacePos;
	}

	/**
	 * Calcule la distance entre les deux points d'ancrage.
	 * 
	 * @param pointA
	 * @param pointB
	 * @return
	 */
	public static float computeDistance(AnchorPoint pointA, AnchorPoint pointB) {
		Vector3f wsA = pointA.worldSpacePos;
		Vector3f wsB = pointB.worldSpacePos;

		float dx = wsA.x - wsB.x;
		float dy = wsA.y - wsB.y;
		float dz = wsA.z - wsB.z;

		return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	public RigidBody getBody() {
		return body;
	}

	public boolean isStatic() {
		return isStatic;
	}

	@Override
	public String toString() {

		String str = "AnchorPoint: static=" + isStatic;
		if (isStatic) {
			str += " position_ws=" + worldSpacePos;
		} else {
			str += " position_ms=" + bodySpacePos + " position_ws=" + worldSpacePos;
		}
		return str;
	}

	/**
	 * @return La contrainte � laquelle ce point d'ancrage appartient.
	 */
	public AbstractConstraint getConstraint() {
		return constraint;
	}

	/**
	 * D�finit l'unique constrainte poss�dant ce point d'ancrage.
	 * 
	 * @param constraint
	 * @throws IllegalStateException si ce point d'ancrage appartient d�j� � une
	 *                               contrainte.
	 * 
	 * @see #AnchorPoint(AnchorPoint) pour dupliquer un point d'ancrage.
	 */
	public void setConstraint(AbstractConstraint constraint) throws IllegalStateException {
		if (this.constraint == null) {
			this.constraint = constraint;
		} else if (!(this.constraint instanceof CompoundConstraint)) {
			throw new IllegalStateException("Erreur, ce point d'ancrage appartient d�j� � une contrainte.");

		}
	}
	
	/**
	 * 
	 * @return une copie de ce point d'ancrage.
	 */
	public AnchorPoint copy() {
		return new AnchorPoint(this);
	}

}
