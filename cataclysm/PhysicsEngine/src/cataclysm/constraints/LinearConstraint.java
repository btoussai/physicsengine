package cataclysm.constraints;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

/**
 * Représente une contrainte sur les translations d'un solide.
 * 
 * @author Briac
 *
 */
public abstract class LinearConstraint extends SimpleConstraint {

	public LinearConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
	}

	@Override
	protected float computeVelocityInvMass(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		float inv_mass = 0;
		if (!pointA.isStatic()) {
			Matrix3f.transform(pointA.getBody().getInvIws(), RaxN, temp);
			inv_mass += pointA.getBody().getInvMass() * (1.0f + Vector3f.dot(RaxN, temp));
		}
		if (!pointB.isStatic()) {
			Matrix3f.transform(pointB.getBody().getInvIws(), RbxN, temp);
			inv_mass += pointB.getBody().getInvMass() * (1.0f + Vector3f.dot(RbxN, temp));
		}
		return inv_mass;
	}

	@Override
	protected void applyImpulse(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp, float applied_impulse) {
		if (!pointA.isStatic()) {
			pointA.getBody().applyImpulse(N, RaxN, applied_impulse, temp);
		}
		if (!pointB.isStatic()) {
			pointB.getBody().applyImpulse(N, RbxN, -applied_impulse, temp);
		}
	}

	@Override
	protected float computePositionInvMass(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		float inv_mass = 0;
		if (!pointA.isStatic()) {
			Matrix3f.transform(pointA.getBody().getInvIws(), RaxN, temp);
			inv_mass += pointA.getBody().getInvMass() * (1.0f + Vector3f.dot(RaxN, temp));
		}
		if (!pointB.isStatic()) {
			Matrix3f.transform(pointB.getBody().getInvIws(), RbxN, temp);
			inv_mass += pointB.getBody().getInvMass() * (1.0f + Vector3f.dot(RbxN, temp));
		}
		return inv_mass;
	}

	@Override
	protected void applyPseudoImpulse(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp,
			float applied_impulse) {
		if (!pointA.isStatic()) {
			pointA.getBody().applyPseudoImpulse(N, RaxN, applied_impulse, temp);
		}
		if (!pointB.isStatic()) {
			pointB.getBody().applyPseudoImpulse(N, RbxN, -applied_impulse, temp);
		}
	}

}
