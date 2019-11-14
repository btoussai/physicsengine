package cataclysm.constraints;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;

/**
 * Cette contrainte permet de simuler une barre dont la longueur peut varier.
 * Cette barre peut tourner, on peut s'en servir pour modéliser un treuil.
 * 
 * @see PistonConstraint <br>
 *      pour modéliser un piston d'axe fixe.
 * @author Briac
 *
 */
public class LinearMotorConstraint extends LinearConstraint {

	/**
	 * Définit le comportement du piston.
	 * 
	 * @author Briac
	 *
	 */
	public enum MotionMode {
		/**
		 * Le piston essaie de s'étendre et de se rétracter pour atteindre une longueur
		 * spécifiée par {@link LinearMotorConstraint#setTargetLength(float)}. <br>
		 * Sa vitesse est égale à
		 * {@code targetSpeed = gain * (targetLength - currentLength)}
		 */
		TrackLength,

		/**
		 * Le piston essaie de modifier sa longueur à la vitesse spécifiée par
		 * {@link LinearMotorConstraint#setTargetSpeed(float)}
		 */
		TrackSpeed;
	}

	/**
	 * La vitesse actuelle d'extension du piston.
	 */
	private float currentSpeed = 0;

	/**
	 * La vitesse souhaitée d'extension du piston
	 */
	private float targetSpeed = 0;

	/**
	 * La longueur actuelle du piston.
	 */
	private float currentLength;

	/**
	 * La longueur souhaitée du piston.
	 */
	private float targetLength;

	/**
	 * La vitesse à laquelle le piston cherche à modifier sa longueur pour atteindre
	 * la longueur voulue. Plus précisément:
	 * {@code targetSpeed = gain * (targetLength - currentLength)}
	 */
	private float gain = 1.0f;

	/**
	 * Le mode d'extension du piston.
	 */
	private MotionMode mode;

	/**
	 * Cette contrainte permet de simuler un piston dont la longueur peut varier. La
	 * longueur du piston ne doit pas être nulle sinon le piston ne peut pas
	 * déterminer une direction d'extension.
	 * 
	 * @param pointA
	 * @param pointB
	 * @param mode   Le comportement du piston.
	 */
	public LinearMotorConstraint(AnchorPoint pointA, AnchorPoint pointB, MotionMode mode) {
		super(pointA, pointB);
		this.mode = mode;
		this.targetLength = this.currentLength = AnchorPoint.computeDistance(pointA, pointB);
	}

	@Override
	protected float buildVelocityJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		Ra.set(pointA.getWorldSpacePosition());
		Rb.set(pointB.getWorldSpacePosition());
		Vector3f.sub(Ra, Rb, N);

		currentLength = N.length();

		if (currentLength > Epsilons.MIN_LENGTH) {
			N.scale(1.0f / currentLength);
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

		if (mode == MotionMode.TrackLength) {
			targetSpeed = gain * (targetLength - currentLength);
		} else {
			targetLength = currentLength;
		}

		currentSpeed = deltaV;
		return deltaV - targetSpeed;
	}

	@Override
	protected float buildPositionJacobian(Vector3f N, Vector3f Ra, Vector3f Rb, Vector3f RaxN, Vector3f RbxN,
			Vector3f temp) {
		return 0;
	}

	@Override
	protected float computePositionError(Vector3f N, Vector3f RaxN, Vector3f RbxN, Vector3f temp) {
		return 0;
	}

	@Override
	protected boolean hasPositionCorrection() {
		return false;
	}

	/**
	 * @return La vitesse actuelle du piston.
	 */
	public float getCurrentSpeed() {
		return currentSpeed;
	}

	/**
	 * @return La vitesse voulue du pison.
	 */
	public float getTargetSpeed() {
		return targetSpeed;
	}

	/**
	 * Change la vitesse cible d'extension du piston. Ceci n'a un effet que si le
	 * piston est en mode {@link MotionMode#TrackSpeed}.
	 * 
	 * @param targetSpeed
	 */
	public void setTargetSpeed(float targetSpeed) {
		this.targetSpeed = targetSpeed;
	}

	/**
	 * @return La longueur actuelle du piston.
	 */
	public float getCurrentLength() {
		return currentLength;
	}

	/**
	 * @return La longueur voulue du pison.
	 */
	public float getTargetLength() {
		return targetLength;
	}

	/**
	 * Change la longueur cible d'extension du pison. Ceci n'a un effet que si le
	 * piston est en mode {@link MotionMode#TrackLength}.
	 * 
	 * @param targetLength
	 */
	public void setTargetLength(float targetLength) {
		this.targetLength = targetLength;
	}

	/**
	 * @return La vitesse à laquelle le piston cherche à modifier sa longueur pour
	 *         atteindre la longueur voulue. Plus précisément:
	 *         {@code targetSpeed = gain * (targetLength - currentLength)}
	 */
	public float getGain() {
		return gain;
	}

	/**
	 * Modifie la vitesse à laquelle le piston cherche à modifier sa longueur pour
	 * atteindre la longueur voulue. Plus précisément:
	 * {@code targetSpeed = gain * (targetLength - currentLength)} Ceci n'a un effet
	 * que si le piston est en mode vaut {@link MotionMode#TrackLength}.
	 * 
	 * @param gain
	 */
	public void setGain(float gain) {
		this.gain = gain;
	}

	/**
	 * @return Le mode d'extension du piston.
	 */
	public MotionMode getMode() {
		return mode;
	}

	/**
	 * Change le mode d'extension du piston.
	 * 
	 * @param mode
	 */
	public void setMode(MotionMode mode) {
		this.mode = mode;
	}

}
