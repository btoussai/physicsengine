package cataclysm.constraints;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;

/**
 * Représente un contrainte de distance entre deux solides.
 * 
 * @author Briac
 *
 */
public class DistanceConstraint extends LinearConstraint {

	private float length;

	public enum DistancePolicy {
		EQUAL, LESS_THAN, GREATER_THAN;
	}
	
	/**
	 * Construit une nouvelle contrainte de distance. La distance est calculé
	 * d'après la position des points d'ancrages. Cette distance est complétement
	 * rigide.
	 * 
	 * @param pointA
	 * @param pointB
	 */
	public DistanceConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		this.length = AnchorPoint.computeDistance(pointA, pointB);
	}

	/**
	 * Construit une nouvelle contrainte de distance. La distance est calculé
	 * d'après la position des points d'ancrages. Cette distance est complétement
	 * rigide.
	 * 
	 * @param pointA
	 * @param pointB
	 * @param policy
	 */
	public DistanceConstraint(AnchorPoint pointA, AnchorPoint pointB, DistancePolicy policy) {
		super(pointA, pointB);
		this.length = AnchorPoint.computeDistance(pointA, pointB);

		switch (policy) {
		case EQUAL:
			break;
		case GREATER_THAN:
			setForceLimits(0, Float.POSITIVE_INFINITY);
			break;
		case LESS_THAN:
			setForceLimits(Float.NEGATIVE_INFINITY, 0);
			break;
		}
	}

	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
		Vector3f temp) {
		Ra.set(pointA.getWorldSpacePosition());
		Rb.set(pointB.getWorldSpacePosition());
		Vector3f.sub(Ra, Rb, temp);

		float distance = temp.length();
		if (distance > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / distance;
			N.set(temp.x * one_over_dist, temp.y * one_over_dist, temp.z * one_over_dist);
		} else {
			N.set(0, 0, 0);
		}

		if (!pointA.isStatic()) {
			Vector3f.sub(Ra, pointA.getBody().getPosition(), Ra);
			Vector3f.cross(Ra, N, RaxN);
		}
		if (!pointB.isStatic()) {
			Vector3f.sub(Rb, pointB.getBody().getPosition(), Rb);
			Vector3f.cross(Rb, N, RbxN);
		}
		
		return computeVelocityError(N, RaxN, RbxN, temp);
	}

	@Override
	protected float computeVelocityError(Vector3f N, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		float deltaV = 0;
		if (!pointA.isStatic()) {
			deltaV += Vector3f.dot(pointA.getBody().getVelocity(), N);
			deltaV += Vector3f.dot(pointA.getBody().getAngularVelocity(), RaxN);
		}
		if (!pointB.isStatic()) {
			deltaV -= Vector3f.dot(pointB.getBody().getVelocity(), N);
			deltaV -= Vector3f.dot(pointB.getBody().getAngularVelocity(), RbxN);
		}
		return deltaV;
	}

	@Override
	protected float buildPositionJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		Ra.set(pointA.getWorldSpacePosition());
		Rb.set(pointB.getWorldSpacePosition());
		Vector3f.sub(Ra, Rb, temp);

		float distance = temp.length();
		if (distance > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / distance;
			N.set(temp.x * one_over_dist, temp.y * one_over_dist, temp.z * one_over_dist);
		} else {
			N.set(0, 0, 0);
		}

		if (!pointA.isStatic()) {
			Vector3f.sub(Ra, pointA.getBody().getPosition(), Ra);
			Vector3f.cross(Ra, N, RaxN);
		}
		if (!pointB.isStatic()) {
			Vector3f.sub(Rb, pointB.getBody().getPosition(), Rb);
			Vector3f.cross(Rb, N, RbxN);
		}

		return distance - length;
	}

	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		Vector3f.sub(pointA.getWorldSpacePosition(), pointB.getWorldSpacePosition(), temp);
		return Vector3f.dot(temp, N) - length;
	}

	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}
	
	public float getLength() {
		return this.length;
	}
	
	public void setLength(float length) {
		this.length = length;
	}

}
