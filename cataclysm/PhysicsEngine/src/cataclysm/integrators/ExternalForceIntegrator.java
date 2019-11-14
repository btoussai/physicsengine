package cataclysm.integrators;

import cataclysm.wrappers.RigidBody;

/**
 * Cet objet permet d'appliquer des forces externes aux objets, telles que la
 * gravité.
 * 
 * @author Briac
 *
 */
public interface ExternalForceIntegrator {

	/**
	 * Cette fonction est appelée avant l'application des forces.
	 */
	public void prepare();

	/**
	 * Intègre les forces externes sur la vitesse (linéaire et angulaire)
	 * uniquement. La position (translation et rotation) ne doit pas être modifiée
	 * car la résolution des collisions intervient après l'application des forces.
	 * <br>
	 * Cette fonction est appelée sequentiellement sur tous les rigidbody de la
	 * simulation soumis à la gravité.
	 * 
	 * @param body     L'objet sur lequel des forces seront appliquées.
	 * @param timeStep Le pas d'intégration.
	 */
	public void applyExternalForces(RigidBody body, float timeStep);

}
