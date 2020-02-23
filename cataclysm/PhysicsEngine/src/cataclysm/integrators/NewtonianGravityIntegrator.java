package cataclysm.integrators;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import cataclysm.wrappers.RigidBody;

/**
 * Cet objet permet d'appliquer une force d'attraction d�croissant comme
 * l'inverse du carr� de la distance entre les solides et un point fixe de
 * l'espace.
 * 
 * @author Briac
 *
 */
public class NewtonianGravityIntegrator implements ExternalForceIntegrator {

	private float K = 200;
	private final Vector3f center = new Vector3f();

	private final Vector3f toBody = new Vector3f();
	private final Vector3f force = new Vector3f();
	
	public NewtonianGravityIntegrator() {
		
	}
	
	public NewtonianGravityIntegrator(float strength, Vector3f center) {
		this.K = strength;
		this.center.set(center);
	}

	@Override
	public void applyExternalForces(RigidBody body,  float timeStep) {
		Vector3f position = body.getPosition();
		Vector3f velocity = body.getVelocity();
		Vector3f.sub(position, center, toBody);

		float distance = toBody.length();


		float f = 0;
		if (distance > Epsilons.MIN_LENGTH) {
			float d3 = distance * distance * distance;
			f = K / d3;
		}
		
		force.set(-toBody.x * f, -toBody.y * f, -toBody.z * f);

		velocity.x += force.x * timeStep;
		velocity.y += force.y * timeStep;
		velocity.z += force.z * timeStep;
	}

	@Override
	public void prepare() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setGravityStrength(float gravityStrength) {
		
	}

}
