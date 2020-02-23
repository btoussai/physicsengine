package cataclysm.constraints;

import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;

/**
 * Applique des impulsions sur les corps jusqu'� satisfaction des contraintes et
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
	 * R�sout le syst�me des contraintes.
	 * 
	 * @param activeMeshContacts
	 * @param activeBodyContacts
	 * @param constraints
	 * @param timeStep
	 * @param MAX_ITERATIONS_VELOCITY
	 * @param MAX_ITERATIONS_POSITION
	 */
	public void solve(List<AbstractSingleBodyContact> activeMeshContacts, List<AbstractDoubleBodyContact> activeBodyContacts,
			List<AbstractConstraint> constraints, float timeStep, int MAX_ITERATIONS_POSITION,
			int MAX_ITERATIONS_VELOCITY) {
		// the velocity must be solved first, since the position correction needs data
		// computed during the velocity step.
		for (int i = 0; i < MAX_ITERATIONS_VELOCITY; i++) {
			// System.out.println("Iteration " + i);
			solveVelocity(activeMeshContacts, activeBodyContacts, constraints, timeStep, i == 0);
		}

		/*
		for (DoubleBodyContact contact : activeBodyContacts) {
			System.out.println("Total impulse: " + contact.impulses_N[0] + " y = "
					+ 0.5f * (contact.getWrapperA().getCentroid().y + contact.getWrapperB().getCentroid().y));
		}
		*/

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
	private void solveVelocity(List<AbstractSingleBodyContact> activeMeshContacts, List<AbstractDoubleBodyContact> activeBodyContacts,
			List<AbstractConstraint> constraints, float timeStep, boolean firstIteration) {

		for (AbstractDoubleBodyContact contact : activeBodyContacts) {
			contact.solveVelocity(firstIteration, timeStep, temp);
		}

		for (AbstractConstraint constraint : constraints) {
			constraint.solveVelocity(firstIteration, timeStep, temp);
		}

		for (AbstractSingleBodyContact contact : activeMeshContacts) {
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
	private void solvePosition(List<AbstractSingleBodyContact> activeMeshContacts, List<AbstractDoubleBodyContact> activeBodyContacts,
			List<AbstractConstraint> constraints, float timeStep, boolean firstIteration) {

		for (AbstractDoubleBodyContact contact : activeBodyContacts) {
			contact.solvePosition(firstIteration, timeStep, temp);
		}

		for (AbstractConstraint constraint : constraints) {
			constraint.solvePosition(firstIteration, timeStep, temp);
		}

		for (AbstractSingleBodyContact contact : activeMeshContacts) {
			contact.solvePosition(firstIteration, timeStep, temp);
		}
	}

}
