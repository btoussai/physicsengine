package cataclysm.contact_creation;

import cataclysm.Epsilons;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.Wrapper;
import math.Clamp;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Représente un contact entre un solide et un triangle. Les solides ne se
 * touchent pas forcément, le contact n'est alors pas actif.
 * 
 * @author Briac
 *
 */
public class SingleBodyContact extends AbstractSingleBodyContact {

	private final Vector3f N = new Vector3f();// normal vector
	private final Vector3f T = new Vector3f();// tangent vector
	private final Vector3f B = new Vector3f();// bitangent vector
	private final Vector3f finalImpulse = new Vector3f();
	private final Vector3f[] R;

	private final Vector3f[] RxN;
	private final Vector3f[] RxT;
	private final Vector3f[] RxB;

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

	public SingleBodyContact(int maxContacts, Wrapper wrapper, Triangle triangle) {
		super(maxContacts, wrapper, triangle);

		R = super.initArray(maxContacts);

		RxN = super.initArray(maxContacts);
		RxT = super.initArray(maxContacts);
		RxB = super.initArray(maxContacts);

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
		super.mixContactProperties(wrapper.getBody().getContactProperties(),
				triangle.mesh.getContactProperties());
		Vector3f.negate(area.getNormal(), N);
		MatrixOps.computeOrthogonalComplement(N, T, B);

		for (int i = 0; i < super.area.getContactCount(); i++) {
			buildJacobian(i);
			computeInvMass(i);
			computeVelocityError(i);//in order to compute deltaV_N
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
	
			wrapper.getBody().applyImpulse(finalImpulse, R[i]);
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

			wrapper.getBody().applyImpulse(finalImpulse, R[i]);
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

			wrapper.getBody().applyPseudoImpulse(N, RxN[i], applied_impulse);
		}
	}

	private final void buildJacobian(int i) {
		RigidBody body = wrapper.getBody();

		Vector3f R = this.R[i];

		Vector3f[] contacts = area.getContactPoints();
		Vector3f.sub(contacts[i], body.getPosition(), R);

		Vector3f.cross(R, N, RxN[i]);
		Vector3f.cross(R, T, RxT[i]);
		Vector3f.cross(R, B, RxB[i]);
	}

	private final void computeInvMass(int i) {
		RigidBody body = wrapper.getBody();

		Matrix3f I_inv = body.getInvIws();
		inv_mass_N[i] = body.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(I_inv, RxN[i]));
		inv_mass_T[i] = body.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(I_inv, RxT[i]));
		inv_mass_B[i] = body.getInvMass() * (1.0f + MatrixOps.sandwichDotProduct(I_inv, RxB[i]));

	}

	private final void computeVelocityError(int i) {
		RigidBody body = wrapper.getBody();

		Vector3f V = body.getVelocity();
		Vector3f W = body.getAngularVelocity();
		Vector3f R = this.R[i];

		// dv = V + W x R
		float dvx = V.x + W.y * R.z - W.z * R.y;
		float dvy = V.y + W.z * R.x - W.x * R.z;
		float dvz = V.z + W.x * R.y - W.y * R.x;

		deltaV_N[i] = dvx * N.x + dvy * N.y + dvz * N.z;// dv . N
		deltaV_T[i] = dvx * T.x + dvy * T.y + dvz * T.z;// dv . T
		deltaV_B[i] = dvx * B.x + dvy * B.y + dvz * B.z;// dv . B
	}

	private final void computePositionError(int i) {
		RigidBody body = wrapper.getBody();

		float deltaV = 0;
		deltaV += Vector3f.dot(body.getPseudoVelocity(), N);
		deltaV += Vector3f.dot(body.getPseudoAngularVelocity(), RxN[i]);
		pseudo_deltaV[i] = deltaV;
	}

}
