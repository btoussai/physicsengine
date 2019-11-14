package cataclysm.constraints;

import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.contact_creation.DoubleBodyContact;
import cataclysm.contact_creation.SingleBodyContact;

/**
 * Applique des impulsions sur les corps jusqu'à satisfaction des contraintes et
 * des contacts.
 * 
 * @author Briac
 *
 */
public class SequentialImpulseSolver {
	
	private final Vector3f temp = new Vector3f();

	public SequentialImpulseSolver() {

	}

	/**
	 * Résout le système des contraintes.
	 * @param activeMeshContacts 
	 * @param activeBodyContacts 
	 * @param constraints
	 * @param timeStep
	 * @param MAX_ITERATIONS_VELOCITY 
	 * @param MAX_ITERATIONS_POSITION 
	 */
	public void solve(List<SingleBodyContact> activeMeshContacts, List<DoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints,
			float timeStep, int MAX_ITERATIONS_VELOCITY, int MAX_ITERATIONS_POSITION) {
		// the velocity must be solved first, since the position correction needs data
		// computed during the velocity step.
		for (int i = 0; i < MAX_ITERATIONS_VELOCITY; i++) {
			solveVelocity(activeMeshContacts, activeBodyContacts, constraints, timeStep, i == 0);
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
	private void solveVelocity(List<SingleBodyContact> activeMeshContacts, List<DoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints,
			float timeStep, boolean firstIteration) {
		
		for(DoubleBodyContact contact : activeBodyContacts) {
			contact.solveVelocity(firstIteration, timeStep, temp);
		}
		
		for (AbstractConstraint constraint : constraints) {
			constraint.solveVelocity(firstIteration, timeStep, temp);
		}
		
		for(SingleBodyContact contact : activeMeshContacts) {
			contact.solveVelocity(firstIteration, timeStep, temp);
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
	private void solvePosition(List<SingleBodyContact> activeMeshContacts, List<DoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints,
			float timeStep, boolean firstIteration) {
		
		for(DoubleBodyContact contact : activeBodyContacts) {
			contact.solvePosition(firstIteration, timeStep, temp);
		}

		for (AbstractConstraint constraint : constraints) {
			constraint.solvePosition(firstIteration, timeStep, temp);
		}
			
		for(SingleBodyContact contact : activeMeshContacts) {
			contact.solvePosition(firstIteration, timeStep, temp);
		}
	}

}
