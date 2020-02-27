package cataclysm.constraints;

import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Repr�sente une contrainte pour simuler une roue.
 * 
 * @author Briac
 *
 */
public class WheelConstraint extends CompoundConstraint {

	private final LineConstraint line;
	private final SpringConstraint spring;
	private final AxisConstraint axis;
	private final AngularMotorConstraint motor;

	public WheelConstraint(AnchorPoint pointA, AnchorPoint pointB, Vector3f up, Vector3f axis) {
		super(pointA, pointB);
		line = new LineConstraint(pointA, pointB, up);
		spring = new SpringConstraint(pointA, pointB);
		this.axis = new AxisConstraint(pointA, pointB, axis);
		motor = new AngularMotorConstraint(pointA, pointB, axis);
	}

	@Override
	protected void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp) {
		line.solveVelocity(firstIteration, timeStep, temp);
		spring.solveVelocity(firstIteration, timeStep, temp);
		axis.solveVelocity(firstIteration, timeStep, temp);
		motor.solveVelocity(firstIteration, timeStep, temp);
	}

	@Override
	protected void solvePosition(boolean firstIteration, float timeStep, Vector3f temp) {
		line.solvePosition(firstIteration, timeStep, temp);
		spring.solvePosition(firstIteration, timeStep, temp);
		axis.solvePosition(firstIteration, timeStep, temp);
		motor.solvePosition(firstIteration, timeStep, temp);
	}

	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}

	public LineConstraint getLine() {
		return line;
	}

	public SpringConstraint getSpring() {
		return spring;
	}

	public AxisConstraint getAxis() {
		return axis;
	}

	public AngularMotorConstraint getMotor() {
		return motor;
	}

	/**
	 * Applique une rotation sur la roue ainsi que sur l'axe de rotation de la roue.
	 * 
	 * @param rotation
	 */
	public void steer(Matrix3f rotation) {
		Vector3f axis = this.axis.getAxis();

		Matrix3f.transform(rotation, axis, axis);

		MatrixOps.matrixTransposeMult(rotation, this.axis.RA0, true, this.axis.RA0);

		this.motor.getAxis().set(axis);

	}

}
