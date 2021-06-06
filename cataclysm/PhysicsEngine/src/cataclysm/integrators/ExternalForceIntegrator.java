package cataclysm.integrators;

import cataclysm.annotations.Parallelizable;
import cataclysm.wrappers.RigidBody;

/**
 * Applies external forces on objects such as gravity
 * 
 * @author Briac
 *
 */
public interface ExternalForceIntegrator {

	/**
	 * This function is called as a setup operation before actually updating the
	 * bodies.
	 */
	public void prepare();

	public void setGravityStrength(float gravityStrength);

	/**
	 * Integrates the external forces to derive the new linear and angular speeds
	 * for the frame being prepared. The position and rotation shouldn't be modified
	 * as the collision step occurs after the forces are applied. <br>
	 * This function is called sequentially or concurrently by multiple threads if
	 * multithreading is enabled. Rigid bodies ignoring external forces
	 * ({@link RigidBody#setExternalForces(boolean)}) are skipped.
	 * 
	 * @param body     The body on which forces are to be applied
	 * @param timeStep The time step
	 */
	@Parallelizable
	public void applyExternalForces(RigidBody body, float timeStep);

}
