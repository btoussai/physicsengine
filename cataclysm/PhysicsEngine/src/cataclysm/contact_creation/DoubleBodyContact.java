package cataclysm.contact_creation;

import cataclysm.Epsilons;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.Wrapper;
import math.Clamp;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Représente un contact entre deux wrappers. Les wrappers ne se touchent pas
 * forcément, le contact n'est alors pas actif.
 * 
 * @author Briac
 *
 */
public class DoubleBodyContact extends AbstractDoubleBodyContact {

	private final Vector3f N = new Vector3f();// normal vector
	private final Vector3f T = new Vector3f();// tangent vector
	private final Vector3f B = new Vector3f();// bitangent vector
	private final Vector3f finalImpulse = new Vector3f();
	private final Vector3f[] Ra;
	private final Vector3f[] Rb;

	private final Vector3f[] RaxN;
	private final Vector3f[] RbxN;
	private final Vector3f[] RaxT;
	private final Vector3f[] RbxT;
	private final Vector3f[] RaxB;
	private final Vector3f[] RbxB;

	private final float[] deltaV_N;
	private final float[] deltaV_T;
	private final float[] deltaV_B;
	private final float[] pseudo_deltaV;

	private final float[] bias;
	private final float[] pseudo_bias;

	private final float[] inv_mass_N;
	private final float[] inv_mass_T;
	private final float[] inv_mass_B;

	public final float[] impulses_N;
	private final float[] impulses_T;
	private final float[] impulses_B;
	private final float[] pseudo_impulses;

	public DoubleBodyContact(int maxContacts, Wrapper wrapperA, Wrapper wrapperB) {
		super(maxContacts, wrapperA, wrapperB);

		Ra = super.initArray(maxContacts);
		Rb = super.initArray(maxContacts);

		RaxN = super.initArray(maxContacts);
		RbxN = super.initArray(maxContacts);
		RaxT = super.initArray(maxContacts);
		RbxT = super.initArray(maxContacts);
		RaxB = super.initArray(maxContacts);
		RbxB = super.initArray(maxContacts);

		deltaV_N = new float[maxContacts];
		deltaV_T = new float[maxContacts];
		deltaV_B = new float[maxContacts];
		pseudo_deltaV = new float[maxContacts];

		bias = new float[maxContacts];
		pseudo_bias = new float[maxContacts];

		inv_mass_N = new float[maxContacts];
		inv_mass_T = new float[maxContacts];
		inv_mass_B = new float[maxContacts];

		impulses_N = new float[maxContacts];
		impulses_T = new float[maxContacts];
		impulses_B = new float[maxContacts];
		pseudo_impulses = new float[maxContacts];
	}

	

	@Override
	public void velocityStart() {
		super.mixContactProperties(wrapperA.getBody().getContactProperties(),
				wrapperB.getBody().getContactProperties());
		Vector3f.negate(area.getNormal(), N);
		MatrixOps.computeOrthogonalComplement(N, T, B);

		for (int i = 0; i < super.area.getContactCount(); i++) {
			buildJacobian(i);
			computeInvMass(i);
			computeVelocityError(i);// in order to compute deltaV_N
			if (deltaV_N[i] < -Epsilons.VELOCITY_ELASTICITY_LIMIT) {
				bias[i] = elasticity * deltaV_N[i];
			} else {
				bias[i] = 0;
			}

		}
	}
	
	@Override
	public void resetImpulses() {
		for (int i = 0; i < super.getMaxContacts(); i++) {
			this.impulses_N[i] = 0;
			this.impulses_T[i] = 0;
			this.impulses_B[i] = 0;
		}
	}
	
	@Override
	public void warmStart() {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			float applied_impulse_N = impulses_N[i];
			float applied_impulse_T = impulses_T[i];
			float applied_impulse_B = impulses_B[i];
			finalImpulse.x = N.x * applied_impulse_N + T.x * applied_impulse_T + B.x * applied_impulse_B;
			finalImpulse.y = N.y * applied_impulse_N + T.y * applied_impulse_T + B.y * applied_impulse_B;
			finalImpulse.z = N.z * applied_impulse_N + T.z * applied_impulse_T + B.z * applied_impulse_B;
	
			wrapperA.getBody().applyImpulse(finalImpulse, Ra[i]);
			finalImpulse.negate();
			wrapperB.getBody().applyImpulse(finalImpulse, Rb[i]);
			finalImpulse.negate();
		}
	}

	@Override
	public void solveVelocity() {

		for (int i = 0; i < super.area.getContactCount(); i++) {

			// the velocity may have changed due to other constraints, we need to recompute
			// it.
			computeVelocityError(i);

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

			// Bitangent impulse
			float prev_impulse_B = impulses_B[i];
			float impulse_B = prev_impulse_B - deltaV_B[i] / inv_mass_B[i];
			impulse_B = Clamp.clamp(impulse_B, -friction * impulse_N, friction * impulse_N);
			impulses_B[i] = impulse_B;
			float applied_impulse_B = impulse_B - prev_impulse_B;

			finalImpulse.x = N.x * applied_impulse_N + T.x * applied_impulse_T + B.x * applied_impulse_B;
			finalImpulse.y = N.y * applied_impulse_N + T.y * applied_impulse_T + B.y * applied_impulse_B;
			finalImpulse.z = N.z * applied_impulse_N + T.z * applied_impulse_T + B.z * applied_impulse_B;

			wrapperA.getBody().applyImpulse(finalImpulse, Ra[i]);
			finalImpulse.negate();
			wrapperB.getBody().applyImpulse(finalImpulse, Rb[i]);
			finalImpulse.negate();
			
		}
	}

	@Override
	public void positionStart(float timeStep) {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			pseudo_impulses[i] = 0;
			pseudo_bias[i] = (Epsilons.PENETRATION_RECOVERY / timeStep)
					* Math.min(0, (area.penetrations[i] + Epsilons.ALLOWED_PENETRATION));
		}
	}

	@Override
	public void solvePosition() {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			computePositionError(i);

			float prev_impulse = pseudo_impulses[i];
			float impulse = prev_impulse - (pseudo_deltaV[i] + pseudo_bias[i]) / inv_mass_N[i];
			impulse = Math.max(0, impulse);
			pseudo_impulses[i] = impulse;
			float applied_impulse = impulse - prev_impulse;

			wrapperA.getBody().applyPseudoImpulse(N, RaxN[i], applied_impulse);
			wrapperB.getBody().applyPseudoImpulse(N, RbxN[i], -applied_impulse);
		}
	}

	private final void buildJacobian(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Vector3f Ra = this.Ra[i];
		Vector3f Rb = this.Rb[i];

		Vector3f[] contacts = area.getContactPoints();
		Vector3f.sub(contacts[i], bodyA.getPosition(), Ra);
		Vector3f.sub(contacts[i], bodyB.getPosition(), Rb);

		Vector3f.cross(Ra, N, RaxN[i]);
		Vector3f.cross(Rb, N, RbxN[i]);
		Vector3f.cross(Ra, T, RaxT[i]);
		Vector3f.cross(Rb, T, RbxT[i]);
		Vector3f.cross(Ra, B, RaxB[i]);
		Vector3f.cross(Rb, B, RbxB[i]);
	}

	private final void computeInvMass(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Matrix3f IA_inv = bodyA.getInvIws();
		Matrix3f IB_inv = bodyB.getInvIws();

		inv_mass_N[i] = bodyA.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(IA_inv, RaxN[i]));
		inv_mass_N[i] += bodyB.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(IB_inv, RbxN[i]));

		inv_mass_T[i] = bodyA.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(IA_inv, RaxT[i]));
		inv_mass_T[i] += bodyB.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(IB_inv, RbxT[i]));

		inv_mass_B[i] = bodyA.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(IA_inv, RaxB[i]));
		inv_mass_B[i] += bodyB.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(IB_inv, RbxB[i]));

	}

	private final void computeVelocityError(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Vector3f Va = bodyA.getVelocity();
		Vector3f Wa = bodyA.getAngularVelocity();
		Vector3f Ra = this.Ra[i];

		// dv = Va + Wa x Ra
		float dvx = Va.x + Wa.y * Ra.z - Wa.z * Ra.y;
		float dvy = Va.y + Wa.z * Ra.x - Wa.x * Ra.z;
		float dvz = Va.z + Wa.x * Ra.y - Wa.y * Ra.x;

		Vector3f Vb = bodyB.getVelocity();
		Vector3f Wb = bodyB.getAngularVelocity();
		Vector3f Rb = this.Rb[i];

		// dv -= Vb + Wb x Rb
		dvx -= Vb.x + Wb.y * Rb.z - Wb.z * Rb.y;
		dvy -= Vb.y + Wb.z * Rb.x - Wb.x * Rb.z;
		dvz -= Vb.z + Wb.x * Rb.y - Wb.y * Rb.x;

		deltaV_N[i] = dvx * N.x + dvy * N.y + dvz * N.z;// dv . N
		deltaV_T[i] = dvx * T.x + dvy * T.y + dvz * T.z;// dv . T
		deltaV_B[i] = dvx * B.x + dvy * B.y + dvz * B.z;// dv . B
	}

	private final void computePositionError(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		float deltaV = 0;
		deltaV += Vector3f.dot(bodyA.getPseudoVelocity(), N);
		deltaV += Vector3f.dot(bodyA.getPseudoAngularVelocity(), RaxN[i]);
		deltaV -= Vector3f.dot(bodyB.getPseudoVelocity(), N);
		deltaV -= Vector3f.dot(bodyB.getPseudoAngularVelocity(), RbxN[i]);
		pseudo_deltaV[i] = deltaV;
	}

}
