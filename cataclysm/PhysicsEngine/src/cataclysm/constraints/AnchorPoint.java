package cataclysm.constraints;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.wrappers.RigidBody;

/**
 * Représente un point d'ancrage d'une contrainte. Un point d'ancrage peut être
 * une coordonnée sur un RigidBody exprimée en model-space ou bien une
 * coordonnée fixe en world-space. Un point d'ancrage ne peut appartenir qu'à
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
	 * Ce point d'ancrage est considéré comme statique, c'est à dire qu'il ne peut
	 * pas bouger et la position doit être exprimée en world-space.
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
	 * Ce point d'ancrage est fixe dans le repère du solide, mais le solide peut
	 * très bien se déplacer. La position doit être exprimée en model-space.
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
					"Erreur, création d'un point d'ancrage dynamique avec un solide valant null.");
		}

		body.vertexToWorldSpace(bodySpacePos, worldSpacePos);
	}

	/**
	 * Ce constructeur permet de dupliquer un point d'ancrage. Ceci permet de
	 * construire une nouvelle contrainte ayant un point d'ancrage localisé au même
	 * endroit qu'un contrainte construite précédemment.
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
	 * @return La contrainte à laquelle ce point d'ancrage appartient.
	 */
	public AbstractConstraint getConstraint() {
		return constraint;
	}

	/**
	 * Définit l'unique constrainte possédant ce point d'ancrage.
	 * 
	 * @param constraint
	 * @throws IllegalStateException si ce point d'ancrage appartient déjà à une
	 *                               contrainte.
	 * 
	 * @see #AnchorPoint(AnchorPoint) pour dupliquer un point d'ancrage.
	 */
	public void setConstraint(AbstractConstraint constraint) throws IllegalStateException {
		if (this.constraint == null) {
			this.constraint = constraint;
		} else if (!(this.constraint instanceof CompoundConstraint)) {
			throw new IllegalStateException("Erreur, ce point d'ancrage appartient déjà à une contrainte.");

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
