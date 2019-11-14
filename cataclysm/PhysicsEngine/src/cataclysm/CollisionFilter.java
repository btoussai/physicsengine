package cataclysm;

import cataclysm.wrappers.RigidBody;

/**
 * 
 * 
 * Permet de déterminer si deux rigidbody doivent ou non enter en collision.
 * 
 * @author Briac
 *
 */
@FunctionalInterface
public interface CollisionFilter {

	public boolean canCollide(RigidBody A, RigidBody B);

}
