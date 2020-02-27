package cataclysm.constraints;

import cataclysm.constraints.LinearMotorConstraint.MotionMode;
import math.vector.Vector3f;

/**
 * Cette contrainte repr�sente un piston par l'association d'un
 * {@link LinearMotorConstraint} et d'un {@link LineConstraint}. La direction du
 * piston est fixe dans le rep�re du solide A, sa longueur peut varier.
 * 
 * 
 * @author Briac
 *
 */
public class PistonConstraint extends CompoundConstraint {

	private final LinearMotorConstraint motor;
	private final LineConstraint line;

	/**
	 * Cette contrainte repr�sente un piston par l'association d'un
	 * {@link LinearMotorConstraint} et d'un {@link LineConstraint}. La direction du
	 * piston est fixe dans le rep�re du solide A, sa longueur peut varier.
	 * La direction du piston est d�duite de la position des points d'ancrage.
	 * 
	 * @param pointA
	 * @param pointB
	 * @throws IllegalArgumentException  si les points d'ancrage sont confondus.
	 */
	public PistonConstraint(AnchorPoint pointA, AnchorPoint pointB) throws IllegalArgumentException {
		super(pointA, pointB);
		motor = new LinearMotorConstraint(pointA, pointB, MotionMode.TrackLength);
		line = new LineConstraint(pointA, pointB);
	}

	/**
	 *  Cette contrainte repr�sente un piston par l'association d'un
	 * {@link LinearMotorConstraint} et d'un {@link LineConstraint}. La direction du
	 * piston est fixe dans le rep�re du solide A, sa longueur peut varier.
	 * 
	 * @param pointA
	 * @param pointB
	 * @param axis La direction d'extension du piston.
	 */
	public PistonConstraint(AnchorPoint pointA, AnchorPoint pointB, Vector3f axis) {
		super(pointA, pointB);
		motor = new LinearMotorConstraint(pointA, pointB, MotionMode.TrackLength);
		line = new LineConstraint(pointA, pointB, axis);
	}

	@Override
	protected void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp) {
		motor.solveVelocity(firstIteration, timeStep, temp);
		line.solveVelocity(firstIteration, timeStep, temp);
	}

	@Override
	protected void solvePosition(boolean firstIteration, float timeStep, Vector3f temp) {
		motor.solvePosition(firstIteration, timeStep, temp);
		line.solvePosition(firstIteration, timeStep, temp);
	}

	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}

	public LinearMotorConstraint getMotor() {
		return motor;
	}

	public LineConstraint getLine() {
		return line;
	}

}
