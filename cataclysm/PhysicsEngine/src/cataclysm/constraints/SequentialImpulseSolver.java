package cataclysm.constraints;

import java.util.List;

import cataclysm.Epsilons;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import math.vector.Vector3f;

/**
 * Applique des impulsions sur les corps jusqu'� satisfaction des contraintes et
 * des contacts.
 * 
 * @author Briac
 *
 */
public class SequentialImpulseSolver implements ConstraintSolver{

	private final Vector3f temp = new Vector3f();

	public SequentialImpulseSolver() {

	}

	@Override
	public void solve(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int MAX_ITERATIONS_POSITION, int MAX_ITERATIONS_VELOCITY) {
		// the velocity must be solved first, since the position correction needs data
		// computed during the velocity step.
		for (int i = 0; i < MAX_ITERATIONS_VELOCITY; i++) {
			// System.out.println("Iteration " + i);
			solveVelocity(activeMeshContacts, activeBodyContacts, constraints, timeStep, i);
		}

		for (int i = 0; i < MAX_ITERATIONS_POSITION; i++) {
			solvePosition(activeMeshContacts, activeBodyContacts, constraints, timeStep, i == 0);
		}

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
			int i) {
		if (i == 0) {
			
				for (AbstractDoubleBodyContact contact : activeBodyContacts) {
					contact.velocityStart();
				}

				for (AbstractSingleBodyContact contact : activeMeshContacts) {
					contact.velocityStart();
				}
				
				for (AbstractDoubleBodyContact contact : activeBodyContacts) {
					if (Epsilons.WARM_START) {
						contact.warmStart();
					}else {
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
					}else {
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

}
