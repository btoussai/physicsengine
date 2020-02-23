package cataclysm.contact_creation;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.Wrapper;
import math.Clamp;

/**
 * Représente un contact entre deux wrappers. Les wrappers ne se touchent pas
 * forcément, le contact n'est alors pas actif.
 * 
 * @author Briac
 *
 */
public class DoubleBodyContact extends AbstractDoubleBodyContact {

	private final Vector3f N = new Vector3f();
	private final Vector3f finalImpulse = new Vector3f();
	private final Vector3f[] Ra;
	private final Vector3f[] Rb;
	private final Vector3f[] RaxN;
	private final Vector3f[] RbxN;
	private final Vector3f[] T;
	private final Vector3f[] RaxT;
	private final Vector3f[] RbxT;

	private final Vector3f[] deltaV;
	private final float[] deltaV_N;
	private final float[] deltaV_T;
	private final float[] pseudo_deltaV;

	private final float[] bias;
	private final float[] pseudo_bias;

	private final float[] inv_mass_N;
	private final float[] inv_mass_T;

	public final float[] impulses_N;
	private final float[] impulses_T;
	private final float[] pseudo_impulses;

	public DoubleBodyContact(int maxContacts, Wrapper wrapperA, Wrapper wrapperB) {
		super(maxContacts, wrapperA, wrapperB);

		Ra = super.initArray(maxContacts);
		Rb = super.initArray(maxContacts);
		RaxN = super.initArray(maxContacts);
		RbxN = super.initArray(maxContacts);
		T = super.initArray(maxContacts);
		RaxT = super.initArray(maxContacts);
		RbxT = super.initArray(maxContacts);

		deltaV = super.initArray(maxContacts);
		deltaV_N = new float[maxContacts];
		deltaV_T = new float[maxContacts];
		pseudo_deltaV = new float[maxContacts];

		bias = new float[maxContacts];
		pseudo_bias = new float[maxContacts];

		inv_mass_N = new float[maxContacts];
		inv_mass_T = new float[maxContacts];

		impulses_N = new float[maxContacts];
		impulses_T = new float[maxContacts];
		pseudo_impulses = new float[maxContacts];
	}

	@Override
	public void resetImpulses() {
		for(int i=0; i<super.getMaxContacts(); i++) {
			this.impulses_N[i] = 0;
			this.impulses_T[i] = 0;
		}
	}

	@Override
	public void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp) {
		if (firstIteration) {
			area.getNormal().negate(N);
			super.mixContactProperties(wrapperA.getBody().getContactProperties(),
					wrapperB.getBody().getContactProperties());
		}

		for (int i = 0; i < super.area.getContactCount(); i++) {
			if (firstIteration) {
				buildVelocityJacobian(i, temp);
				computeVelocityInvMass(i, temp);
				if (deltaV_N[i] < -Epsilons.VELOCITY_ELASTICITY_LIMIT) {
					bias[i] = elasticity * deltaV_N[i];
				} else {
					bias[i] = 0;
				}
				
				if(Epsilons.WARM_START) {
					float applied_impulse_N = impulses_N[i];
					float applied_impulse_T = impulses_T[i];
					finalImpulse.x = N.x * applied_impulse_N + T[i].x * applied_impulse_T;
					finalImpulse.y = N.y * applied_impulse_N + T[i].y * applied_impulse_T;
					finalImpulse.z = N.z * applied_impulse_N + T[i].z * applied_impulse_T;

					applyImpulse(i, temp, finalImpulse);
				}else {
					impulses_N[i] = 0;
					impulses_T[i] = 0;
				}
				
			} else {
				// the velocity may have changed due to other constraints, we need to recompute
				// it.
				computeVelocityError(i, temp);
			}

			// Normal impulse
			float prev_impulse_N = impulses_N[i];
			float impulse_N = prev_impulse_N - (deltaV_N[i] + bias[i]) / inv_mass_N[i];
			impulse_N = Math.max(0, impulse_N);
			impulses_N[i] = impulse_N;
			float applied_impulse_N = impulse_N - prev_impulse_N;

			// Tangent impulse
			float prev_impulse_T = impulses_T[i];
			float impulse_T = prev_impulse_T - deltaV_T[i] / inv_mass_T[i];
			impulse_T = Clamp.clamp(impulse_T, -friction * impulse_N, friction * impulse_N);
			impulses_T[i] = impulse_T;
			float applied_impulse_T = impulse_T - prev_impulse_T;

			finalImpulse.x = N.x * applied_impulse_N + T[i].x * applied_impulse_T;
			finalImpulse.y = N.y * applied_impulse_N + T[i].y * applied_impulse_T;
			finalImpulse.z = N.z * applied_impulse_N + T[i].z * applied_impulse_T;

			applyImpulse(i, temp, finalImpulse);
		}
	}

	@Override
	public void solvePosition(boolean firstIteration, float timeStep, Vector3f temp) {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			if (firstIteration) {
				pseudo_impulses[i] = 0;
				pseudo_bias[i] = (Epsilons.PENETRATION_RECOVERY / timeStep)
						* Math.min(0, (area.penetrations[i] + Epsilons.ALLOWED_PENETRATION));
			}

			computePositionError(i, temp);

			float prev_impulse = pseudo_impulses[i];
			float impulse = prev_impulse - (pseudo_deltaV[i] + pseudo_bias[i]) / inv_mass_N[i];
			impulse = Math.max(0, impulse);
			pseudo_impulses[i] = impulse;
			float applied_impulse = impulse - prev_impulse;

			applyPseudoImpulse(i, temp, applied_impulse);
		}
	}

	@Override
	protected void buildVelocityJacobian(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		area.getNormal().negate(N);

		Vector3f[] contacts = area.getContactPoints();
		Vector3f.sub(contacts[i], bodyA.getPosition(), Ra[i]);
		Vector3f.cross(Ra[i], N, RaxN[i]);
		Vector3f.sub(contacts[i], bodyB.getPosition(), Rb[i]);
		Vector3f.cross(Rb[i], N, RbxN[i]);

		Vector3f dv = deltaV[i];

		dv.set(bodyA.getVelocity());
		Vector3f.cross(bodyA.getAngularVelocity(), Ra[i], temp);
		Vector3f.add(dv, temp, dv);

		Vector3f.sub(dv, bodyB.getVelocity(), dv);
		Vector3f.cross(bodyB.getAngularVelocity(), Rb[i], temp);
		Vector3f.sub(dv, temp, dv);

		float dvN = Vector3f.dot(dv, N);
		float dvT = 0;

		temp.set(N.x * dvN, N.y * dvN, N.z * dvN);
		Vector3f.sub(dv, temp, temp);

		dvT = temp.length();
		if (dvT > Epsilons.MIN_TANGENT_SPEED) {
			float inv = 1.0f / dvT;
			T[i].set(temp.x * inv, temp.y * inv, temp.z * inv);
			Vector3f.cross(Ra[i], T[i], RaxT[i]);
		} else {
			T[i].set(0, 0, 0);
			RaxT[i].set(0, 0, 0);
		}

		deltaV_N[i] = dvN;
		deltaV_T[i] = dvT;
	}

	@Override
	protected void computeVelocityInvMass(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Matrix3f.transform(bodyA.getInvIws(), RaxN[i], temp);
		inv_mass_N[i] = bodyA.getInvMass() * (1.0f + Vector3f.dot(RaxN[i], temp));
		Matrix3f.transform(bodyB.getInvIws(), RbxN[i], temp);
		inv_mass_N[i] += bodyB.getInvMass() * (1.0f + Vector3f.dot(RbxN[i], temp));

		Matrix3f.transform(bodyA.getInvIws(), RaxT[i], temp);
		inv_mass_T[i] = bodyA.getInvMass() * (1.0f + Vector3f.dot(RaxT[i], temp));
		Matrix3f.transform(bodyB.getInvIws(), RbxT[i], temp);
		inv_mass_T[i] += bodyB.getInvMass() * (1.0f + Vector3f.dot(RbxT[i], temp));
	}

	@Override
	protected void computeVelocityError(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Vector3f dv = deltaV[i];

		dv.set(bodyA.getVelocity());
		Vector3f.cross(bodyA.getAngularVelocity(), Ra[i], temp);
		Vector3f.add(dv, temp, dv);

		Vector3f.sub(dv, bodyB.getVelocity(), dv);
		Vector3f.cross(bodyB.getAngularVelocity(), Rb[i], temp);
		Vector3f.sub(dv, temp, dv);

		deltaV_N[i] = Vector3f.dot(dv, N);
		deltaV_T[i] = Vector3f.dot(dv, T[i]);
	}

	@Override
	protected void applyImpulse(int i, Vector3f temp, Vector3f finalImpulse) {
		//System.out.println("Total impulse: " + this.impulses_N[0] + " Impulse: " + finalImpulse.y);
		wrapperA.getBody().applyImpulse(finalImpulse, Ra[i], temp);
		finalImpulse.negate();
		wrapperB.getBody().applyImpulse(finalImpulse, Rb[i], temp);
	}

	@Override
	protected void computePositionError(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		float deltaV = 0;
		deltaV += Vector3f.dot(bodyA.getPseudoVelocity(), N);
		deltaV += Vector3f.dot(bodyA.getPseudoAngularVelocity(), RaxN[i]);
		deltaV -= Vector3f.dot(bodyB.getPseudoVelocity(), N);
		deltaV -= Vector3f.dot(bodyB.getPseudoAngularVelocity(), RbxN[i]);
		pseudo_deltaV[i] = deltaV;
	}

	@Override
	protected void applyPseudoImpulse(int i, Vector3f temp, float applied_impulse) {
		wrapperA.getBody().applyPseudoImpulse(N, RaxN[i], applied_impulse, temp);
		wrapperB.getBody().applyPseudoImpulse(N, RbxN[i], -applied_impulse, temp);
	}

}
