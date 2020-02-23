package cataclysm.integrators;

import cataclysm.wrappers.RigidBody;

/**
 * Cet objet permet d'appliquer des forces externes aux objets, telles que la
 * gravit�.
 * 
 * @author Briac
 *
 */
public interface ExternalForceIntegrator {

	/**
	 * Cette fonction est appel�e avant l'application des forces.
	 */
	public void prepare();
	
	public void setGravityStrength(float gravityStrength);

	/**
	 * Int�gre les forces externes sur la vitesse (lin�aire et angulaire)
	 * uniquement. La position (translation et rotation) ne doit pas �tre modifi�e
	 * car la r�solution des collisions intervient apr�s l'application des forces.
	 * <br>
	 * Cette fonction est appel�e sequentiellement sur tous les rigidbody de la
	 * simulation soumis � la gravit�.
	 * 
	 * @param body     L'objet sur lequel des forces seront appliqu�es.
	 * @param timeStep Le pas d'int�gration.
	 */
	public void applyExternalForces(RigidBody body, float timeStep);

}
