package cataclysm.constraints;

import java.util.List;

import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;

public interface ConstraintSolver {

	/**
	 * Résout le système des contraintes.
	 * 
	 * @param activeMeshContacts
	 * @param activeBodyContacts
	 * @param constraints
	 * @param timeStep
	 * @param MAX_ITERATIONS_VELOCITY
	 * @param MAX_ITERATIONS_POSITION
	 */
	public void solve(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int MAX_ITERATIONS_POSITION, int MAX_ITERATIONS_VELOCITY);

}
