package cataclysm.constraints;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import math.MatrixOps;

/**
 * Cette contrainte permet d'appliquer un couple entre deux solides selon un axe.
 * @author Briac
 *
 */
public class AngularMotorConstraint extends AxisConstraint{
	
	/**
	 * Définit le comportement du moteur.
	 * 
	 * @author Briac
	 *
	 */
	public enum MotionMode {
		/**
		 * Le moteur essaie de tourner pour atteindre un angle spécifié par {@link AngularMotorConstraint#setTargetAngle(float)}. <br>
		 * Sa vitesse est égale à
		 * {@code targetSpeed = gain * (targetAngle - currentAngle)}
		 */
		TrackAngle,

		/**
		 * Le moteur essaie d'atteindre la vitesse spécifiée par
		 * {@link AngularMotorConstraint#setTargetSpeed(float)}
		 */
		TrackSpeed;
	}

	/**
	 * La vitesse actuelle de rotation des objets autour de l'axe.
	 */
	private float currentSpeed = 0;

	/**
	 * La vitesse de rotation souhaitée.
	 */
	private float targetSpeed = 0;

	/**
	 * L'angle actuel du moteur.
	 */
	private float currentAngle;

	/**
	 * L'angle souhaité du moteur.
	 */
	private float targetAngle;

	/**
	 * La vitesse à laquelle le moteur cherche à modifier sa vitesse de rotation pour atteindre
	 * l'angle voulu. Plus précisément:
	 * {@code targetSpeed = gain * (targetAngle - currentAngle)}
	 */
	private float gain = 30.0f;

	/**
	 * Le mode de fonctionnement du moteur.
	 */
	private MotionMode mode;

	
	public AngularMotorConstraint(AnchorPoint pointA, AnchorPoint pointB, Vector3f axis) {
		super(pointA, pointB, axis);
		this.mode = MotionMode.TrackSpeed;
	}
	
	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		
		if(!pointA.isStatic()) {
			pointA.getBody().normalToWorldSpace(axis, N);
		}else {
			N.set(axis);
		}
		
		return computeVelocityError(N, RaxN, RbxN, temp);
	}
	
	@Override
	protected float computeVelocityError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		float deltaV = 0;
		if(!pointA.isStatic()) {
			deltaV += Vector3f.dot(N, pointA.getBody().getAngularVelocity());
		}
		if(!pointB.isStatic()) {
			deltaV -= Vector3f.dot(N, pointB.getBody().getAngularVelocity());
		}

		this.currentAngle = computeAngle(N);
		
		if (mode == MotionMode.TrackAngle) {
			targetSpeed = computeTargetSpeed();
		} else {
			targetAngle = currentAngle;
		}

		currentSpeed = deltaV;
		return deltaV - targetSpeed;
	}

	@Override
	protected boolean hasPositionCorrection() {
		return false;
	}
	
	private float computeAngle(Vector3f axis) {
		super.computeRotationDelta(currentDelta);
		
		Vector3f axisOffset = Matrix3f.transform(currentDelta, axis, null);
		Matrix3f toAxis = MatrixOps.createRotationMatrix(axisOffset, axis, (Matrix3f)null);
		Matrix3f.mul(toAxis, currentDelta, currentDelta);
		
		float angle = MatrixOps.matrixToAxisAngle(currentDelta, axisOffset);
		
		if(Vector3f.dot(axis, axisOffset) < 0) {
			angle = -angle;
		}
		
		return angle;
	}
	
	private float computeTargetSpeed() {
		
		float targetSpeed = -gain * (targetAngle - currentAngle);
		
		return targetSpeed;
	}
	

	public float getTargetSpeed() {
		return targetSpeed;
	}

	public void setTargetSpeed(float targetSpeed) {
		this.targetSpeed = targetSpeed;
	}

	public float getTargetAngle() {
		return targetAngle;
	}

	public void setTargetAngle(float targetAngle) {
		this.targetAngle = targetAngle;
	}

	public float getGain() {
		return gain;
	}

	public void setGain(float gain) {
		this.gain = gain;
	}

	public MotionMode getMode() {
		return mode;
	}

	public void setMode(MotionMode mode) {
		this.mode = mode;
	}

	public float getCurrentSpeed() {
		return currentSpeed;
	}

	public float getCurrentAngle() {
		return currentAngle;
	}

}
