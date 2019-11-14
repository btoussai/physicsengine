package cataclysm.constraints;

import org.lwjgl.util.vector.Vector3f;

/**
 * Représente une soudure entre deux rigidbody. Leur translation et leur
 * rotation deviennent liées.
 * 
 * @author Briac
 *
 */
public class WeldConstraint extends CompoundConstraint {
	
	private final PinLocationConstraint pinLocation;
	private final PinRotationConstraint pinRotation;

	/**
	 * Représente une soudure entre deux rigidbody. Leur translation et leur
	 * rotation deviennent liées.
	 * 
	 * @param pointA
	 * @param pointB
	 */
	public WeldConstraint(AnchorPoint pointA, AnchorPoint pointB) {
		super(pointA, pointB);
		pinLocation = new PinLocationConstraint(pointA, pointB);
		pinRotation = new PinRotationConstraint(pointA, pointB);
	}

	@Override
	protected void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp) {
		pinLocation.solveVelocity(firstIteration, timeStep, temp);
		pinRotation.solveVelocity(firstIteration, timeStep, temp);
	}

	@Override
	protected void solvePosition(boolean firstIteration, float timeStep, Vector3f temp) {
		pinLocation.solvePosition(firstIteration, timeStep, temp);
		pinRotation.solvePosition(firstIteration, timeStep, temp);
	}

	@Override
	protected boolean hasPositionCorrection() {
		return true;
	}

}
