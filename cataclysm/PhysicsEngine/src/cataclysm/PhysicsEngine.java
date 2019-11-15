package cataclysm;

import java.util.List;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.SequentialImpulseSolver;
import cataclysm.integrators.ExternalForceIntegrator;
import cataclysm.integrators.GyroscopicIntegrator;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;
import cataclysm.wrappers.Transform;
import math.MatrixOps;

/**
 * Permet de simuler les intéractions entre les objets.
 * 
 * @author Briac
 *
 */
final class PhysicsEngine {

	/**
	 * Les paramètres globaux pour la simulation.
	 */
	private final DefaultParameters params;

	/**
	 * Permet d'appliquer les forces ext�rieurs aux solides.
	 */
	private final ExternalForceIntegrator forceInegrator;

	/**
	 * Résout le système de contraintes entre les objets.
	 */
	private final SequentialImpulseSolver solver = new SequentialImpulseSolver();

	/**
	 * Permet d'intégrer le terme gyroscopique dans l'équation différentielle sur la
	 * vitesse angulaire.
	 */
	private final GyroscopicIntegrator gyroscopicIntegrator = new GyroscopicIntegrator();

	// quelques variables de travail
	private final Matrix3f rotation = new Matrix3f();
	private final Vector3f axis = new Vector3f();

	/**
	 * Instancie un moteur physique. Il s'occupe de la mise � jour des objets.
	 * 
	 * @param params
	 */
	PhysicsEngine(DefaultParameters params) {
		this.params = params;
		this.forceInegrator = params.getForceIntegrator();
	}

	/**
	 * Met à jour l'ensemble de la simulation.
	 * 
	 * @param bodies
	 * @param meshes
	 * @param constraints
	 * @param stats
	 */
	void update(RigidBodyManager bodies, StaticMeshManager meshes, List<AbstractConstraint> constraints,
			PhysicsStats stats) {
		meshes.update();

		float timeStep = params.getTimeStep();
		boolean gyroscopicIntegration = params.useGyroscopicIntegration();

		applyForces(bodies, timeStep);

		stats.broadAndNarrowphase.start();
		bodies.update();
		stats.broadAndNarrowphase.stop();
		

		stats.reset(bodies.size(), meshes.size(), constraints.size());
		
		stats.constraintSolver.start();
		solver.solve(bodies.getMeshContacts(), bodies.getBodyContacts(), constraints, timeStep, params.getMaxIterationsPosition(),
				params.getMaxIterationVelocity());
		stats.constraintSolver.stop();

		stats.velocityIntegration.start();
		integrateVelocity(bodies, timeStep, gyroscopicIntegration);
		stats.velocityIntegration.stop();

	}

	/**
	 * Int�gre l'acc�l�ration des objets sur la dur�e du pas pour obtenir leur
	 * nouvelle vitesse.
	 * 
	 * @param timeStep
	 */
	private void applyForces(RigidBodyManager bodies, float timeStep) {
		forceInegrator.prepare();

		for (RigidBody body : bodies) {
			if (!body.isGravity() || body.isSleeping()) {
				continue;
			}

			forceInegrator.applyExternalForces(body, timeStep);
		}

	}

	/**
	 * Int�gre la vitesse des objets sur la dur�e du pas de temps pour obtenir leur
	 * nouvelle position. Met � jour la matrice de transformation et l'enveloppe des
	 * objets.
	 * 
	 * @param timeStep
	 * @param gyroscopicIntegration
	 */
	private void integrateVelocity(RigidBodyManager bodies, float timeStep, boolean gyroscopicIntegration) {

		Transform temp = new Transform();

		for (RigidBody body : bodies) {
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
			float omega = axis.length();

			float movement_squared = dx * dx + dy * dy + dz * dz;

			boolean translation_negligible = movement_squared < Epsilons.Sleep.MIN_TRANSLATION_SPEED
					* Epsilons.Sleep.MIN_TRANSLATION_SPEED;
			boolean rotation_negligible = omega < Epsilons.Sleep.MIN_ROTATION_SPEED;

			if (Epsilons.Sleep.SLEEPING_ALLOWED) {

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
						body.setSleepCounter(sleepCounter + 1);
					}
					continue;
				} else {
					body.setSleepCounter(0);
					body.setSleeping(false);
				}
			}

			if (!translation_negligible) {
				body.translate(dx * timeStep, dy * timeStep, dz * timeStep);
			}

			if (!rotation_negligible) {
				float one_over_omega = 1.0f / omega;
				axis.set(axis.x * one_over_omega, axis.y * one_over_omega, axis.z * one_over_omega);
				MatrixOps.createRotationMatrix3f(omega * timeStep, axis, rotation);
				body.rotateAboutCenterOfMass(rotation);
			}

			body.updateTransform(temp);

			pseudoVel.set(0, 0, 0);
			pseudoAngVel.set(0, 0, 0);
		}
	}

}
