package cataclysm.constraints;

import org.lwjgl.util.vector.Vector3f;

/**
 * Cette contrainte restreint la position relative des solides à un plan. Ce
 * plan est défini dans le repère local (model-space) du solide de référence.
 * 
 * @author Briac
 *
 */
public class PlaneConstraint extends LinearConstraint {

	private final Vector3f planeNormal = new Vector3f();

	/**
	 * Cette contrainte restreint la position relative des solides à un plan. Ce
	 * plan est défini dans le repère local (model-space) du solide de référence.
	 * 
	 * @param referencePoint
	 * @param constrainedPoint
	 * @param planeNormal
	 */
	public PlaneConstraint(AnchorPoint referencePoint, AnchorPoint constrainedPoint, Vector3f planeNormal) {
		super(referencePoint, constrainedPoint);
		if (!referencePoint.isStatic()) {
			referencePoint.getBody().normalToBodySpace(planeNormal, this.planeNormal);
		} else {
			this.planeNormal.set(planeNormal);
		}
	}
	
	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {

		Ra.set(pointA.getWorldSpacePosition());
		Rb.set(pointB.getWorldSpacePosition());
		planeNormal.negate(N);
		if (!pointA.isStatic()) {
			super.pointA.getBody().normalToWorldSpace(N, N);
		}
		closestToPlane(Ra, N, Rb, temp);


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
		planeNormal.negate(N);
		if (!pointA.isStatic()) {
			super.pointA.getBody().normalToWorldSpace(N, N);
		}
		float distance = closestToPlane(Ra, N, Rb, temp);

		if (!pointA.isStatic()) {
			Vector3f.sub(temp, pointA.getBody().getPosition(), Ra);
			Vector3f.cross(Ra, N, RaxN);
		}
		if (!pointB.isStatic()) {
			Vector3f.sub(temp, pointB.getBody().getPosition(), Rb);
			Vector3f.cross(Rb, N, RbxN);
		}

		return -distance;
	}

	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		Vector3f.sub(pointA.getWorldSpacePosition(), pointB.getWorldSpacePosition(), temp);
		return Vector3f.dot(temp, N);
	}


	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}

	/**
	 * Calcule le projeté orthogonal de point sur le plan défini par base et normal.
	 * Le résultat est stocké dans dest.
	 * 
	 * @param base
	 * @param normal
	 * @param point
	 * @param dest
	 * @return La distance signée entre point et le plan.
	 */
	private float closestToPlane(Vector3f base, Vector3f normal, Vector3f point, Vector3f dest) {
		float x = point.x - base.x;
		float y = point.y - base.y;
		float z = point.z - base.z;
		float d = normal.x * x + normal.y * y + normal.z * z;
		dest.set(point.x - d * normal.x, point.y - d * normal.y, point.z - d * normal.z);
		return d;
	}

}
