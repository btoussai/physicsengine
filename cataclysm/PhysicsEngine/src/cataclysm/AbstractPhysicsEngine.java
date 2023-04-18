package cataclysm;

import java.util.List;

import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.ConstraintSolver;
import cataclysm.integrators.ExternalForceIntegrator;
import cataclysm.integrators.GyroscopicIntegrator;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Base class for a physics engine.
 * 
 * @author Briac
 *
 */
public abstract class AbstractPhysicsEngine {

	/**
	 * The world in which the physics engine updates objects
	 */
	protected final PhysicsWorld world;

	/**
	 * The default parameters of the simulation
	 */
	protected final DefaultParameters params;

	/**
	 * Applies forces such as gravity on bodies
	 */
	protected final ExternalForceIntegrator forceInegrator;

	/**
	 * Solves the constraints system
	 */
	protected final ConstraintSolver solver;

	/**
	 * Integrates the gyroscopic member in the differential equation of the rotation
	 */
	protected final GyroscopicIntegrator gyroscopicIntegrator = new GyroscopicIntegrator();

	/**
	 * Build a new physics engine
	 * 
	 * @param params
	 */
	AbstractPhysicsEngine(PhysicsWorld world, ConstraintSolver solver) {
		this.world = world;
		this.params = world.getParameters();
		this.forceInegrator = params.getForceIntegrator();
		this.solver = solver;
	}

	/**
	 * Updates the whole simulation in the following order: <br>
	 * 
	 * add/remove new/old static meshes <br>
	 * 
	 * apply forces on all bodies <br>
	 * 
	 * add/remove new/old bodies <br>
	 * 
	 * update all bodies <br>
	 * 
	 * @param bodies
	 * @param meshes
	 * @param constraints
	 * @param stats
	 */
	protected abstract void update(RigidBodyManager bodies, StaticMeshManager meshes,
			List<AbstractConstraint> constraints, PhysicsStats stats);

	/**
	 * Applies external forces on all bodies subject to them.
	 * 
	 * @param bodies
	 * @param timeStep
	 */
	protected void applyForces(List<RigidBody> bodies, float timeStep) {
		for (RigidBody body : bodies) {
			if (!body.isExternalForces() || body.isSleeping()) {
				continue;
			}

			forceInegrator.applyExternalForces(body, timeStep);
		}
	}

	/**
	 * Computes the new position and velocity based on the time step.
	 * 
	 * @param timeStep
	 * @param gyroscopicIntegration true if the gyroscopic term should be taken into
	 *                              account while solving the differential equation
	 *                              on the angular velocity
	 */
	protected void integrateVelocity(List<RigidBody> bodies, float timeStep, boolean gyroscopicIntegration) {

		// some temporary variables
		final Matrix3f rotation = new Matrix3f();
		final Vector3f axis = new Vector3f();
		
		for (RigidBody body : bodies) {
			if (body.isSkipIntegration()) {
				body.updateTransforms();
				continue;
			}

			Vector3f velocity = body.getVelocity();
			Vector3f pseudoVel = body.getPseudoVelocity();

			float dx = (velocity.x + pseudoVel.x);
			float dy = (velocity.y + pseudoVel.y);
			float dz = (velocity.z + pseudoVel.z);

			Vector3f angularVelocity = body.getAngularVelocity();

			if (gyroscopicIntegration) {
				gyroscopicIntegrator.integrateGyroscopicTerm(timeStep, body);
			}

			Vector3f pseudoAngVel = body.getPseudoAngularVelocity();
			Vector3f.add(angularVelocity, pseudoAngVel, axis);
			float omega2 = axis.lengthSquared();

			float movement_squared = dx * dx + dy * dy + dz * dz;

			boolean translation_negligible = movement_squared < Epsilons.Sleep.MIN_TRANSLATION_SPEED
					* Epsilons.Sleep.MIN_TRANSLATION_SPEED;
			boolean rotation_negligible = omega2 < Epsilons.Sleep.MIN_ROTATION_SPEED
					* Epsilons.Sleep.MIN_ROTATION_SPEED;

			if (translation_negligible && rotation_negligible) {
				int sleepCounter = body.getSleepCounter();
				if (sleepCounter >= Epsilons.Sleep.FRAMES_SPENT_AT_REST) {
					body.setSleeping(true);
					velocity.set(0, 0, 0);
					angularVelocity.set(0, 0, 0);
					pseudoVel.set(0, 0, 0);
					pseudoAngVel.set(0, 0, 0);
				} else {
					pseudoVel.set(0, 0, 0);
					pseudoAngVel.set(0, 0, 0);
					if (Epsilons.Sleep.SLEEPING_ALLOWED) {
						body.setSleepCounter(sleepCounter + 1);
					}
				}
				continue;
			} else {
				body.setSleepCounter(0);
				body.setSleeping(false);
			}

			if (!translation_negligible) {
				body.translate(dx * timeStep, dy * timeStep, dz * timeStep);
			}

			if (!rotation_negligible) {
				float omega = (float) Math.sqrt(omega2);
				float one_over_omega = 1.0f / omega;
				axis.set(axis.x * one_over_omega, axis.y * one_over_omega, axis.z * one_over_omega);
				MatrixOps.createRotationMatrix3f(omega * timeStep, axis, rotation);
				body.rotateAboutCenterOfMass(rotation);
			}

			body.updateTransforms();

			pseudoVel.set(0, 0, 0);
			pseudoAngVel.set(0, 0, 0);

		}
	}

}
