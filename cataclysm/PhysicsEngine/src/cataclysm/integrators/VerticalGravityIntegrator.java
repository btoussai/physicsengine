package cataclysm.integrators;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.wrappers.RigidBody;

/**
 * Cet objet permet de simuler une gravité constante selon une direction.
 * 
 * @author Briac
 *
 */
public class VerticalGravityIntegrator implements ExternalForceIntegrator {

	private final Vector3f GRAVITY;

	/**
	 * Représente une accélération contante dans la direction [-Y] de 9.81 m/s².
	 * Comme sur Terre, quoi.
	 */
	public VerticalGravityIntegrator() {
		this.GRAVITY = new Vector3f(0, -9.81f, 0);
	}

	/**
	 * Permet d'appliquer une force de gravité constante dans une direction fixe.
	 * 
	 * @param gravity L'accélération appliquée aux objets. Le vecteur est recopié à
	 *                l'instanciation.
	 */
	public VerticalGravityIntegrator(Vector3f gravity) {
		this.GRAVITY = new Vector3f(gravity);
	}

	@Override
	public void applyExternalForces(RigidBody body, float timeStep) {
		Vector3f velocity = body.getVelocity();
		velocity.x += timeStep * GRAVITY.x;
		velocity.y += timeStep * GRAVITY.y;
		velocity.z += timeStep * GRAVITY.z;
	}

	@Override
	public void prepare() {
		// TODO Auto-generated method stub
		
	}

}
