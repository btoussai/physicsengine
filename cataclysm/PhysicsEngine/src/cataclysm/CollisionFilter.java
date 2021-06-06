package cataclysm;

import cataclysm.wrappers.RigidBody;

/**
 * 
 * 
 * Determines whether two rigid bodies can collide.
 * 
 * @see DefaultCollisionFilter
 * 
 * @author Briac
 *
 */
@FunctionalInterface
public interface CollisionFilter {

	public boolean canCollide(RigidBody A, RigidBody B);

}
