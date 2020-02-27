package cataclysm.constraints;

import cataclysm.Epsilons;
import math.Clamp;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Cette contrainte permet de forcer deux solides � tourner autour d'un axe. La
 * translation n'est pas influenc�e.
 * 
 * @author Briac
 *
 */
public class AxisConstraint extends PinRotationConstraint {

	protected final Vector3f axis = new Vector3f();

	/**
	 * 
	 * @param referencePoint
	 * @param constrainedPoint
	 * @param axis
	 */
	public AxisConstraint(AnchorPoint referencePoint, AnchorPoint constrainedPoint, Vector3f axis) {
		super(referencePoint, constrainedPoint);

		if (!referencePoint.isStatic()) {
			referencePoint.getBody().normalToBodySpace(axis, this.axis);
		} else {
			this.axis.set(axis);
		}
	}

	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		N.set(0, 0, 0);
		if(!pointA.isStatic()) {
			Vector3f.add(N, pointA.getBody().getAngularVelocity(), N);
			pointA.getBody().normalToWorldSpace(axis, temp);
		}else {
			temp.set(axis);
		}
		if(!pointB.isStatic()) {
			Vector3f.sub(N, pointB.getBody().getAngularVelocity(), N);
		}
		
		Vector3f.cross(temp, N, N);
		Vector3f.cross(temp, N, N);
		
		float deltaV = N.length();
		if (deltaV > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / deltaV;
			N.x *= one_over_dist;
			N.y *= one_over_dist;
			N.z *= one_over_dist;
		} else {
			N.set(0, 1, 0);
		}
		

		//System.out.println("DeltaV: " + -deltaV);
		
		return -deltaV;
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
		
		//System.out.println("DeltaV: " + deltaV);
		
		return deltaV;
	}

	@Override
	protected float buildPositionJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		computeRotationDelta(currentDelta);

		if (!pointA.isStatic()) {
			pointA.getBody().normalToWorldSpace(axis, N);
		}else {
			N.set(axis);
		}
		Matrix3f.transform(currentDelta, N, temp);
		
		//System.out.println("AxisConstraint: ");
		//System.out.println("delta x axis = " + temp);
		
		float angle = 0;
		float dot = Vector3f.dot(temp, N);
		Vector3f.cross(temp, N, N);

		float distance = N.length();
		if (distance > Epsilons.MIN_LENGTH) {
			float one_over_dist = 1.0f / distance;
			N.set(N.x * one_over_dist, N.y * one_over_dist, N.z * one_over_dist);
			angle = (float) Math.acos(Clamp.symetricClamp(dot));
		} else {
			N.set(0, 1, 0);
		}
		
		
		//System.out.println("angle: " + angle);
		//System.out.println("dot: " + dot);
		//System.out.println("Dir: " + N);

		return angle;
	}
	
	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		return 0;
	}
	
	public Vector3f getAxis() {
		return axis;
	}
	
	public void setAxis(float x, float y, float z) {
		axis.set(x, y, z);
	}

}
