package cataclysm.constraints;

import cataclysm.wrappers.RigidBody;
import math.vector.Vector3f;

/**
 * Defines an anchor point for a constraint. An anchor point can be given in
 * body-space or in world-space. An instance of an anchor point cannot be shared
 * between constraints.
 * 
 * @author Briac Toussaint
 *
 */
public class AnchorPoint {

	private final Vector3f bodySpacePos = new Vector3f();
	private final Vector3f worldSpacePos = new Vector3f();
	private final RigidBody body;
	private final boolean isStatic;
	private AbstractConstraint constraint;

	/**
	 * Builds a new anchor point for a constraint. <br>
	 * This anchor point is static, it means that it is given in world space and
	 * that it cannot move.
	 * 
	 * @param position the position of the anchor point in world-space.
	 */
	public AnchorPoint(Vector3f position) {
		this.bodySpacePos.set(position);
		this.worldSpacePos.set(position);
		this.body = null;
		this.isStatic = true;
	}

	/**
	 * Builds a new anchor point for a constraint. <br>
	 * The position of this anchor point is fixed in the reference frame of the
	 * rigid body it is attached to.
	 * 
	 * @param position the position of the anchor point in body-space.
	 * @param body     the rigid body onto which the anchor point is attached.
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
	 * Duplicates an anchor point.
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
	 * Compute the distance between two anchor points.
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

	/**
	 * @return true if the anchor point is static, that is to say, the anchor point
	 *         is attached to a fixed location in world-space.
	 */
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
	 * @return The constraint owning this anchor point.
	 */
	public AbstractConstraint getConstraint() {
		return constraint;
	}

	/**
	 * Sets the constraint owning this anchor point.
	 * Use {@link #AnchorPoint(AnchorPoint)} or {@link #copy()} to duplicate an anchor point.
	 * 
	 * @param constraint
	 * @throws IllegalStateException If this anchor point is already in use.
	 * 
	 */
	public void setConstraint(AbstractConstraint constraint) throws IllegalStateException {
		if (this.constraint == null) {
			this.constraint = constraint;
		} else if (!(this.constraint instanceof CompoundConstraint)) {
			throw new IllegalStateException("Erreur, ce point d'ancrage appartient d�j� � une contrainte.");

		}
	}

	/**
	 * @return a copy of this anchor point.
	 */
	public AnchorPoint copy() {
		return new AnchorPoint(this);
	}

}
