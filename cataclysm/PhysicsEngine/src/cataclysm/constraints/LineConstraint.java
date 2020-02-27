package cataclysm.constraints;

import cataclysm.Epsilons;
import math.vector.Vector3f;

/**
 * Cette contrainte force les translations des solides selon un axe. Cet axe est
 * d�fini dans le rep�re local (model-space) du solide de r�f�rence. On peut
 * s'en servir pour guider un objet selon une direction fixe.
 * 
 * @author Briac
 *
 */
public class LineConstraint extends LinearConstraint {

	private final Vector3f axis = new Vector3f();

	/**
	 * Cette contrainte force les translations des solides selon un axe reliant
	 * initialement les deux points d'ancrage. Cet axe est ensuite converti dans le
	 * rep�re local (model-space) du solide de r�ference. Une rotation ult�rieure du
	 * solide de r�ference entrainera donc une rotation de l'axe. <br>
	 * Ce type de contrainte peut �tre utilis� pour mod�liser une grue (solide de
	 * r�ference) et sa charge (solide contraint). La charge se d�place le long de
	 * la grue, sans restriction particuli�re sur sa rotation. La grue peut se
	 * d�placer et tourner sur elle-m�me, entra�nant la charge dans son mouvement.
	 * <br>
	 * 
	 * La direction de la ligne est d�duite de la position des points d'ancrage.
	 * 
	 * @param referencePoint
	 * @param constrainedPoint
	 * @throws IllegalArgumentException si les points d'ancrage sont confondus.
	 */
	public LineConstraint(AnchorPoint referencePoint, AnchorPoint constrainedPoint) throws IllegalArgumentException {
		super(referencePoint, constrainedPoint);
		Vector3f pA = referencePoint.getWorldSpacePosition();
		Vector3f pB = constrainedPoint.getWorldSpacePosition();

		Vector3f.sub(pB, pA, axis);
		if (!referencePoint.isStatic()) {
			referencePoint.getBody().normalToBodySpace(axis, axis);
		}
		float d = axis.length();
		if (d > Epsilons.MIN_LENGTH) {
			axis.scale(1.0f / d);
		} else {
			throw new IllegalArgumentException("Erreur, les deux points d'ancrage sont confondus.");
		}
	}

	/**
	 * Cette contrainte force les translations des solides selon un axe reliant
	 * initialement les deux points d'ancrage. Cet axe est ensuite converti dans le
	 * rep�re local (model-space) du solide de r�ference. Une rotation ult�rieure du
	 * solide de r�ference entrainera donc une rotation de l'axe. <br>
	 * Ce type de contrainte peut �tre utilis� pour mod�liser une grue (solide de
	 * r�ference) et sa charge (solide contraint). La charge se d�place le long de
	 * la grue, sans restriction particuli�re sur sa rotation. La grue peut se
	 * d�placer et tourner sur elle-m�me, entra�nant la charge dans son mouvement.
	 * 
	 * @param referencePoint
	 * @param constrainedPoint
	 * @param axis             La direction de l'axe en world-space. Le vecteur doit
	 *                         �tre unitaire.
	 */
	public LineConstraint(AnchorPoint referencePoint, AnchorPoint constrainedPoint, Vector3f axis) {
		super(referencePoint, constrainedPoint);

		if (!referencePoint.isStatic()) {
			referencePoint.getBody().normalToBodySpace(axis, this.axis);
		}
	}

	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {

		Ra.set(pointA.getWorldSpacePosition());
		Rb.set(pointB.getWorldSpacePosition());
		if (!pointA.isStatic()) {
			super.pointA.getBody().normalToWorldSpace(axis, N);
		} else {
			N.set(axis);
		}

		closestToLine(Ra, N, Rb, temp);

		Vector3f.sub(temp, Rb, N);
		float distance = N.length();
		if (distance > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / distance;
			N.set(N.x * one_over_dist, N.y * one_over_dist, N.z * one_over_dist);
		} else {
			N.set(0, 0, 0);
		}

		if (!pointA.isStatic()) {
			Vector3f.sub(temp, pointA.getBody().getPosition(), Ra);
			Vector3f.cross(Ra, N, RaxN);
		}
		if (!pointB.isStatic()) {
			Vector3f.sub(temp, pointB.getBody().getPosition(), Rb);
			Vector3f.cross(Rb, N, RbxN);
		}

		return computeVelocityError(N, RaxN, RbxN, temp);
	}

	@Override
	protected float computeVelocityError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
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
		if (!pointA.isStatic()) {
			super.pointA.getBody().normalToWorldSpace(axis, N);
		} else {
			N.set(axis);
		}

		closestToLine(Ra, N, Rb, temp);

		Vector3f.sub(temp, Rb, N);
		float distance = N.length();
		if (distance > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / distance;
			N.set(N.x * one_over_dist, N.y * one_over_dist, N.z * one_over_dist);
		} else {
			N.set(0, 0, 0);
		}

		if (!pointA.isStatic()) {
			Vector3f.sub(temp, pointA.getBody().getPosition(), Ra);
			Vector3f.cross(Ra, N, RaxN);
		}
		if (!pointB.isStatic()) {
			Vector3f.sub(temp, pointB.getBody().getPosition(), Rb);
			Vector3f.cross(Rb, N, RbxN);
		}

		return distance;
	}

	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		Vector3f.sub(pointA.getWorldSpacePosition(), pointB.getWorldSpacePosition(), temp);
		return Vector3f.dot(temp, N);
	}

	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}

	/**
	 * Calcule la position du projet� orthogonal de point sur le segment form� par
	 * base et axis. La position est ramen�e dans l'intervalle [minDistance,
	 * maxDistance] sur le segment.
	 * 
	 * @param base  La base du segment.
	 * @param axis  Le vecteur directeur, unitaire, du segment.
	 * @param point Le point dont on cherche le projet� orthogonal.
	 * @param dest  Le vecteur de destination
	 */
	private void closestToLine(Vector3f base, Vector3f axis, Vector3f point, Vector3f dest) {
		float x = point.x - base.x;
		float y = point.y - base.y;
		float z = point.z - base.z;
		float d = axis.x * x + axis.y * y + axis.z * z;
		dest.set(base.x + d * axis.x, base.y + d * axis.y, base.z + d * axis.z);
	}

}
