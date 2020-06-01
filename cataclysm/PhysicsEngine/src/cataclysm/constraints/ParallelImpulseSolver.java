package cataclysm.constraints;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import cataclysm.Epsilons;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.parallel.PhysicsWork;
import cataclysm.parallel.PhysicsWorkerPool;
import cataclysm.parallel.PhysicsWorkerThread;
import math.vector.Vector3f;

/**
 * Applique des impulsions sur les corps jusqu'à satisfaction des contraintes et
 * des contacts.
 * 
 * @author Briac
 *
 */
public class ParallelImpulseSolver implements ConstraintSolver {
	
	private static final boolean DEBUG = false;
	
	private final PhysicsWorkerPool workers;
	
	public ParallelImpulseSolver(PhysicsWorkerPool workers) {
		this.workers = workers;
	}

	@Override
	public void solve(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int MAX_ITERATIONS_POSITION, int MAX_ITERATIONS_VELOCITY) {

		List<PhysicsWork> tasks = new ArrayList<>();
		for (int i = 0; i < workers.getThreadCount(); i++) {
			final List<AbstractSingleBodyContact> meshes = workers.buildSubList(activeMeshContacts, i);
			final List<AbstractDoubleBodyContact> bodies = workers.buildSubList(activeBodyContacts, i);
			final List<AbstractConstraint> constraintsSubList;
			
			if(i == 0) {
				constraintsSubList = constraints;
			}else {
				constraintsSubList = Collections.emptyList();
			}
			
			tasks.add(new PhysicsWork() {

				@Override
				public void run(PhysicsWorkerThread physicsWorkerThread) {
					solve(meshes, bodies, constraintsSubList, timeStep,
							MAX_ITERATIONS_POSITION, MAX_ITERATIONS_VELOCITY, physicsWorkerThread);
				}
				
			});

		}

		workers.scheduleWork(tasks);
	}

	private void solve(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int MAX_ITERATIONS_POSITION, int MAX_ITERATIONS_VELOCITY, PhysicsWorkerThread worker) {

		// the velocity must be solved first, since the position correction needs data
		// computed during the velocity step.
		for (int iteration = 0; iteration < MAX_ITERATIONS_VELOCITY; iteration++) {
			solveVelocity(activeMeshContacts, activeBodyContacts, constraints, timeStep, iteration);
			worker.waitForGroup();
		}

		for (int iteration = 0; iteration < MAX_ITERATIONS_POSITION; iteration++) {
			solvePosition(activeMeshContacts, activeBodyContacts, constraints, timeStep, iteration == 0);
			worker.waitForGroup();
		}


		if (DEBUG)
			System.err.println(" ID: " + worker.getThreadIndex() + " DoubleBodies: " + activeBodyContacts.size()
					+ " SingleBodies: " + activeMeshContacts.size() + " Constraints: " + constraints.size());
	}

	/**
	 * Applique des impulsions pour corriger les erreurs de vitesse.
	 * 
	 * @param activeMeshContacts
	 * @param activeBodyContacts
	 * @param constraints
	 * @param timeStep
	 */
	private void solveVelocity(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int iteration) {

		final Vector3f temp = new Vector3f();

		if (iteration == 0) {

			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.velocityStart();
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.velocityStart();
			}

			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				if (Epsilons.WARM_START) {
					contact.warmStart();
				} else {
					contact.resetImpulses();
				}
				contact.solveVelocity();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solveVelocity(true, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				if (Epsilons.WARM_START) {
					contact.warmStart();
				} else {
					contact.resetImpulses();
				}
				contact.solveVelocity();
			}

		} else {
			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.solveVelocity();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solveVelocity(false, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.solveVelocity();
			}
		}

	}

	/**
	 * Applique des impulsions pour corriger les erreurs de position. Ces impulsions
	 * ne modifient pas la vitesse des solides.
	 * 
	 * @param activeMeshContacts
	 * @param activeBodyContacts
	 * @param constraints
	 * @param timeStep
	 */
	private void solvePosition(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			boolean firstIteration) {

		final Vector3f temp = new Vector3f();

		if (firstIteration) {
			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.positionStart(timeStep);
				contact.solvePosition();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solvePosition(true, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.positionStart(timeStep);
				contact.solvePosition();
			}
		} else {
			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.solvePosition();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solvePosition(false, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.solvePosition();
			}
		}

	}
	
	public PhysicsWorkerPool getWorkers() {
		return workers;
	}

}
