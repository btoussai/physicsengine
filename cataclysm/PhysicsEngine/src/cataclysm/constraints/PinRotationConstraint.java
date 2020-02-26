package cataclysm.constraints;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import math.MatrixOps;

/**
 * Cette contrainte permet de verrouiller l'orientation relative de deux
 * solides. La position des points d'ancrage n'a pas d'importance. <br>
 * Cela signifie qu'une rotation sur un solide sera accompagn�e par une rotation
 * de l'autre et r�ciproquement. Les translations n'auront quant � elles aucune
 * influence.
 * 
 * @author Briac
 *
 */
public class PinRotationConstraint extends AngularConstraint {

	/**
	 * L'orientation initiale de A.
	 */
	protected final Matrix3f RA0 = new Matrix3f();

	/**
	 * L'orientation initiale de B.
	 */
	protected final Matrix3f RB0 = new Matrix3f();

	/**
	 * RA0 * t( RA )
	 */
	protected final Matrix3f deltaRA = new Matrix3f();

	/**
	 * RB0 * t( RB )
	 */
	protected final Matrix3f deltaRB = new Matrix3f();

	/**
	 * La diff�rence actuelle d'orientation entre A et B.<br>
	 * t( deltaRA ) * deltaRB
	 */
	protected final Matrix3f currentDelta = new Matrix3f();

	public PinRotationConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		if (!pointA.isStatic()) {
			Matrix3f.load(pointA.getBody().getOrientation(), RA0);
		}
		if (!pointB.isStatic()) {
			Matrix3f.load(pointB.getBody().getOrientation(), RB0);
		}

	}

	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		// On cherche � r�duire la vitesse angulaire relative des deux solides.
		N.set(0, 0, 0);
		if (!pointA.isStatic()) {
			Vector3f.add(N, pointA.getBody().getAngularVelocity(), N);
		}
		if (!pointB.isStatic()) {
			Vector3f.sub(N, pointB.getBody().getAngularVelocity(), N);
		}
		float distance = N.length();
		if (distance > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / distance;
			N.set(N.x * one_over_dist, N.y * one_over_dist, N.z * one_over_dist);
		} else {
			N.set(0, 1, 0);
		}

		return distance;
	}

	@Override
	protected float computeVelocityInvMass(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		float inv_mass = 0;
		if (!pointA.isStatic()) {
			Matrix3f.transform(pointA.getBody().getInvIws(), N, temp);
			inv_mass += pointA.getBody().getInvMass() * Vector3f.dot(N, temp);
		}
		if (!pointB.isStatic()) {
			Matrix3f.transform(pointB.getBody().getInvIws(), N, temp);
			inv_mass += pointB.getBody().getInvMass() * Vector3f.dot(N, temp);
		}
		return inv_mass;
	}

	@Override
	protected float computeVelocityError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		float deltaV = 0;
		if (!pointA.isStatic()) {
			deltaV += Vector3f.dot(N, pointA.getBody().getAngularVelocity());
		}
		if (!pointB.isStatic()) {
			deltaV -= Vector3f.dot(N, pointB.getBody().getAngularVelocity());
		}

		return deltaV;
	}

	@Override
	protected void applyImpulse(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp, float applied_impulse) {
		if (!pointA.isStatic()) {
			pointA.getBody().applyImpulseTorque(N, applied_impulse);
		}
		if (!pointB.isStatic()) {
			pointB.getBody().applyImpulseTorque(N, -applied_impulse);
		}
	}

	@Override
	protected float buildPositionJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		// On cherche � r�duire l'erreur d'orientation entre les deux solides.
		computeRotationDelta(currentDelta);

		float angle = -MatrixOps.matrixToAxisAngle(currentDelta, N);

		//System.out.println("\nPinRotationConstraint: ");
		//System.out.println("Angle: " + angle);
		//System.out.println("Dir: " + N);

		return angle;
	}

	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		return 0;
	}

	@Override
	protected float computePositionInvMass(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		float inv_mass = 0;
		if (!pointA.isStatic()) {
			Matrix3f.transform(pointA.getBody().getInvIws(), N, temp);
			inv_mass += pointA.getBody().getInvMass() * Vector3f.dot(N, temp);
		}
		if (!pointB.isStatic()) {
			Matrix3f.transform(pointB.getBody().getInvIws(), N, temp);
			inv_mass += pointB.getBody().getInvMass() * Vector3f.dot(N, temp);
		}
		return inv_mass;
	}

	@Override
	protected void applyPseudoImpulse(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp, float applied_impulse) {
		if (!pointA.isStatic()) {
			pointA.getBody().applyPseudoImpulseTorque(N, applied_impulse);
		}
		if (!pointB.isStatic()) {
			pointB.getBody().applyPseudoImpulseTorque(N, -applied_impulse);
		}
	}

	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}

	/**
	 * Calcule la "diff�rence" d'orientation entre le solide A et le solide B.
	 * 
	 * @param dest
	 */
	protected void computeRotationDelta(Matrix3f dest) {
		if (!pointA.isStatic() && pointB.isStatic()) {

			Matrix3f RA = pointA.getBody().getOrientation();
			MatrixOps.matrixTransposeMult(RA0, RA, false, dest);

		} else if (pointA.isStatic() && !pointB.isStatic()) {

			Matrix3f RB = pointB.getBody().getOrientation();
			MatrixOps.matrixTransposeMult(RB, RB0, false, dest);

		} else {

			Matrix3f RA = pointA.getBody().getOrientation();
			MatrixOps.matrixTransposeMult(RA, RA0, false, deltaRA);

			Matrix3f RB = pointB.getBody().getOrientation();
			MatrixOps.matrixTransposeMult(RB, RB0, false, deltaRB);

			MatrixOps.matrixTransposeMult(deltaRB, deltaRA, false, dest);

		}
	}

}
