package cataclysm;

import cataclysm.constraints.AnchorPoint;
import cataclysm.wrappers.RigidBody;

/**
 * Determines if two rigid bodies can collide by applying the following rule: <br>
 * {@code boolean canCollide = (maskA & categoryB) != 0 && (maskB & categoryA) != 0;}
 * 
 * @author Briac
 *
 */
public class DefaultCollisionFilter implements CollisionFilter {

	@Override
	public boolean canCollide(RigidBody A, RigidBody B) {

		// Not the same body
		if (A.getID() != B.getID()) {
			// can collide according to their mask and category
			if ((A.getMask() & B.getCategory()) != 0 && (B.getMask() & A.getCategory()) != 0) {
				// they are not both kinematic bodies
				if (A.getInvMass() != 0 || B.getInvMass() != 0) {

					for (AnchorPoint point : A.getAnchorPoints()) {
						if (point.getConstraint().checkConnected(A, B)) {
							if (!point.getConstraint().shouldCollide()) {
								return false;
							}
						}
					}

					return true;
				}
			}
		}
		return false;
	}

}
