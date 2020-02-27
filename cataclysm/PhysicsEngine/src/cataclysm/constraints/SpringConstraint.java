package cataclysm.constraints;

import cataclysm.Epsilons;
import math.vector.Vector3f;

/**
 * Cette contrainte repr�sente un ressort entre deux solides.
 * 
 * @author Briac
 *
 */
public class SpringConstraint extends LinearConstraint {

	/**
	 * La longueur du ressort au repos.
	 */
	private float length;
	
	/**
	 * La longueur actuelle du ressort.
	 */
	private float currentLength;

	/**
	 * Un facteur d'amortissement pour la contrainte.
	 * 
	 * @see Constraint#setSoftnessParameters(float, float)
	 */
	private float damping_ratio;

	/**
	 * Un facteur d'amortissement pour la contrainte.
	 * 
	 * @see Constraint#setSoftnessParameters(float, float)
	 */
	private float angular_frequency;

	/**
	 * Construit un ressort entre les deux solides.
	 * @param pointA
	 * @param pointB
	 */
	public SpringConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		this.length = AnchorPoint.computeDistance(pointA, pointB);
		setSoftnessParameters(0, 1);
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
		
		this.currentLength = distance;
		
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
	protected float computeVelocityImpulse(float velocity_error, float inv_mass, float timeStep) {
		float k = angular_frequency * angular_frequency / inv_mass;
		float c = 2.0f * damping_ratio * angular_frequency / inv_mass;

		float beta = k / (c + timeStep * k);
		float gamma = 1.0f / (timeStep * (c + timeStep * k));

		return -(velocity_error + beta * (currentLength - length)) / (inv_mass + gamma);
	}

	@Override
	protected float buildPositionJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		return 0;
	}

	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		return 0;
	}

	@Override
	protected boolean hasPositionCorrection() {
		return false;
	}
	
	/**
	 * Modifie les param�tres de rigidit� du ressort.
	 * 
	 * @param damping_ratio Un facteur d'amortissement. <br>
	 *                      0.0f --> pas d'amortissement <br>
	 *                      1.0f --> aucune oscillation
	 * @param frequency     Le nombre d'oscillation par seconde. Une fr�quence �l�v�e
	 *                      correspond � une contrainte plus rigide, une fr�quence
	 *                      faible correspond � une contrainte plus souple. <br>
	 *                      0.1f --> tr�s souple <br>
	 *                      30.0f --> tr�s raide <br>
	 * 
	 */
	public void setSoftnessParameters(float damping_ratio, float frequency) {
		this.damping_ratio = damping_ratio;
		this.angular_frequency = 2.0f * (float) Math.PI * frequency;
	}

	

}
