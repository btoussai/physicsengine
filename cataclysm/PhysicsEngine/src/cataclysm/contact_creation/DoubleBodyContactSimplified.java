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
public class DoubleBodyContactSimplified extends AbstractDoubleBodyContact {

	private enum VecData {
		Ra, Rb, RaxN, RbxN, T, RaxT, RbxT, deltaV, END;
	}

	private enum FloatData {
		deltaV_N, deltaV_T, pseudo_deltaV, bias, pseudo_bias,
				inv_mass_N, inv_mass_T, impulses_N, impulses_T, pseudo_impulses, END;
	}

	private final Vector3f N = new Vector3f();
	private final Vector3f finalImpulse = new Vector3f();
	private final float vecData[];
	private final float floatData[];

	public DoubleBodyContactSimplified(int maxContacts, Wrapper wrapperA, Wrapper wrapperB) {
		super(maxContacts, wrapperA, wrapperB);
		this.wrapperA = wrapperA;
		this.wrapperB = wrapperB;

		vecData = new float[VecData.END.ordinal() * 3 * maxContacts];
		floatData = new float[FloatData.END.ordinal() * maxContacts];
	}

	@Override
	public void resetImpulses() {
		for (int i = 0; i < super.getMaxContacts(); i++) {
			setFloat(FloatData.impulses_N, 0, i);
			setFloat(FloatData.impulses_T, 0, i);
		}
	}

	@Override
	public void solveVelocity(boolean firstIteration, float timeStep, Vector3f temp) {
		if (firstIteration) {
			area.getNormal().negate(N);
			super.mixContactProperties(wrapperA.getBody().getContactProperties(),
					wrapperB.getBody().getContactProperties());
		}

		if (firstIteration && Epsilons.WARM_START) {
			for (int i = 0; i < super.area.getContactCount(); i++) {
				buildVelocityJacobian(i, temp);
				computeVelocityInvMass(i, temp);

				float deltaV_N = getFloat(FloatData.deltaV_N, i);
				if (deltaV_N < -Epsilons.VELOCITY_ELASTICITY_LIMIT) {
					setFloat(FloatData.bias, elasticity * deltaV_N, i);
				} else {
					setFloat(FloatData.bias, 0, i);
				}

				float applied_impulse_N = getFloat(FloatData.impulses_N, i);
				float applied_impulse_T = getFloat(FloatData.impulses_T, i);

				getVec(VecData.T, temp, i);
				finalImpulse.x = N.x * applied_impulse_N + temp.x * applied_impulse_T;
				finalImpulse.y = N.y * applied_impulse_N + temp.y * applied_impulse_T;
				finalImpulse.z = N.z * applied_impulse_N + temp.z * applied_impulse_T;

				applyImpulse(i, temp, finalImpulse);
			}
			return;
		}

		for (int i = 0; i < super.area.getContactCount(); i++) {
			if (firstIteration) {
				setFloat(FloatData.impulses_N, 0, i);
				setFloat(FloatData.impulses_T, 0, i);
				buildVelocityJacobian(i, temp);
				computeVelocityInvMass(i, temp);

				float deltaV_N = getFloat(FloatData.deltaV_N, i);
				if (deltaV_N < -Epsilons.VELOCITY_ELASTICITY_LIMIT) {
					setFloat(FloatData.bias, elasticity * deltaV_N, i);
				} else {
					setFloat(FloatData.bias, 0, i);
				}
			} else {
				// the velocity may have changed due to other constraints, we need to recompute
				// it.
				computeVelocityError(i, temp);
			}

			// Normal impulse
			float prev_impulse_N = getFloat(FloatData.impulses_N, i);
			float impulse_N = prev_impulse_N - (getFloat(FloatData.deltaV_N, i) + getFloat(FloatData.bias, i))
					/ getFloat(FloatData.inv_mass_N, i);
			impulse_N = Math.max(0, impulse_N);
			setFloat(FloatData.impulses_N, impulse_N, i);
			float applied_impulse_N = impulse_N - prev_impulse_N;

			// Tangent impulse
			float prev_impulse_T = getFloat(FloatData.impulses_T, i);
			float impulse_T = prev_impulse_T - getFloat(FloatData.deltaV_T, i) / getFloat(FloatData.inv_mass_T, i);
			impulse_T = Clamp.clamp(impulse_T, -friction * impulse_N, friction * impulse_N);
			setFloat(FloatData.impulses_T, impulse_T, i);
			float applied_impulse_T = impulse_T - prev_impulse_T;

			getVec(VecData.T, temp, i);
			finalImpulse.x = N.x * applied_impulse_N + temp.x * applied_impulse_T;
			finalImpulse.y = N.y * applied_impulse_N + temp.y * applied_impulse_T;
			finalImpulse.z = N.z * applied_impulse_N + temp.z * applied_impulse_T;

			applyImpulse(i, temp, finalImpulse);
		}
	}

	@Override
	public void solvePosition(boolean firstIteration, float timeStep, Vector3f temp) {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			if (firstIteration) {
				setFloat(FloatData.pseudo_impulses, 0, i);
				float pseudo_bias = (Epsilons.PENETRATION_RECOVERY / timeStep)
						* Math.min(0, (area.penetrations[i] + Epsilons.ALLOWED_PENETRATION));
				setFloat(FloatData.pseudo_bias, pseudo_bias, i);
			}

			computePositionError(i, temp);

			float prev_impulse = getFloat(FloatData.pseudo_impulses, i);
			float impulse = prev_impulse - (getFloat(FloatData.pseudo_deltaV, i) + getFloat(FloatData.pseudo_bias, i))
					/ getFloat(FloatData.inv_mass_N, i);
			impulse = Math.max(0, impulse);
			setFloat(FloatData.pseudo_impulses, impulse, i);
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
		sub(contacts[i], bodyA.getPosition(), VecData.Ra, i);
		cross(VecData.Ra, N, VecData.RaxN, i);
		sub(contacts[i], bodyB.getPosition(), VecData.Rb, i);
		cross(VecData.Rb, N, VecData.RbxN, i);

		cross(bodyA.getAngularVelocity(), VecData.Ra, temp, i);
		add(bodyA.getVelocity(), temp, VecData.deltaV, i);

		cross(bodyB.getAngularVelocity(), VecData.Rb, temp, i);
		Vector3f.add(bodyB.getVelocity(), temp, temp);
		sub(VecData.deltaV, temp, VecData.deltaV, i);

		float dvN = dot(VecData.deltaV, N, i);
		float dvT = 0;

		temp.set(N.x * dvN, N.y * dvN, N.z * dvN);
		sub(VecData.deltaV, temp, temp, i);

		dvT = temp.length();
		if (dvT > Epsilons.MIN_TANGENT_SPEED) {
			float inv = 1.0f / dvT;
			store(temp.x * inv, temp.y * inv, temp.z * inv, VecData.T, i);
			cross(VecData.Ra, VecData.T, VecData.RaxT, i);
		} else {
			store(0, 0, 0, VecData.T, i);
			store(0, 0, 0, VecData.RaxT, i);
		}

		setFloat(FloatData.deltaV_N, dvN, i);
		setFloat(FloatData.deltaV_T, dvT, i);
	}

	@Override
	protected void computeVelocityInvMass(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		float inv_mass_N = bodyA.getInvMass() * (1.0f + sandwichProduct(bodyA.getInvIws(), VecData.RaxN, i));
		inv_mass_N += bodyB.getInvMass() * (1.0f + sandwichProduct(bodyB.getInvIws(), VecData.RbxN, i));
		setFloat(FloatData.inv_mass_N, inv_mass_N, i);

		float inv_mass_T = bodyA.getInvMass() * (1.0f + sandwichProduct(bodyA.getInvIws(), VecData.RaxT, i));
		inv_mass_T += bodyB.getInvMass() * (1.0f + sandwichProduct(bodyB.getInvIws(), VecData.RbxT, i));
		setFloat(FloatData.inv_mass_T, inv_mass_T, i);
	}

	@Override
	protected void computeVelocityError(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		cross(bodyA.getAngularVelocity(), VecData.Ra, temp, i);
		add(bodyA.getVelocity(), temp, VecData.deltaV, i);

		cross(bodyB.getAngularVelocity(), VecData.Rb, temp, i);
		Vector3f.add(bodyB.getVelocity(), temp, temp);
		sub(VecData.deltaV, temp, VecData.deltaV, i);

		setFloat(FloatData.deltaV_N, dot(VecData.deltaV, N, i), i);
		setFloat(FloatData.deltaV_T, dot(VecData.deltaV, VecData.T, i), i);
	}

	@Override
	protected void applyImpulse(int i, Vector3f temp, Vector3f finalImpulse) {
		RigidBody bodyA = wrapperA.getBody();
		float inv_mass = bodyA.getInvMass();
		temp.set(finalImpulse.x * inv_mass, finalImpulse.y * inv_mass, finalImpulse.z * inv_mass);
		bodyA.getVelocity().translate(temp);

		cross(VecData.Ra, temp, temp, i);
		Matrix3f.transform(bodyA.getInvIws(), temp, temp);
		bodyA.getAngularVelocity().translate(temp);

		RigidBody bodyB = wrapperB.getBody();
		inv_mass = bodyB.getInvMass();
		temp.set(finalImpulse.x * -inv_mass, finalImpulse.y * -inv_mass, finalImpulse.z * -inv_mass);
		bodyB.getVelocity().translate(temp);

		cross(VecData.Rb, temp, temp, i);
		Matrix3f.transform(bodyB.getInvIws(), temp, temp);
		bodyB.getAngularVelocity().translate(temp);
	}

	@Override
	protected void computePositionError(int i, Vector3f temp) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		float deltaV = 0;
		deltaV += Vector3f.dot(bodyA.getPseudoVelocity(), N);
		deltaV += dot(bodyA.getPseudoAngularVelocity(), VecData.RaxN, i);
		deltaV -= Vector3f.dot(bodyB.getPseudoVelocity(), N);
		deltaV -= dot(bodyB.getPseudoAngularVelocity(), VecData.RbxN, i);
		setFloat(FloatData.pseudo_deltaV, deltaV, i);
	}

	@Override
	protected void applyPseudoImpulse(int i, Vector3f temp, float applied_impulse) {
		RigidBody bodyA = wrapperA.getBody();
		float inv_mass = bodyA.getInvMass();
		float effect = applied_impulse * inv_mass;
		bodyA.getPseudoVelocity().translate(N, effect);

		transform(bodyA.getInvIws(), VecData.RaxN, temp, i);
		bodyA.getPseudoAngularVelocity().translate(temp, effect);

		RigidBody bodyB = wrapperB.getBody();
		inv_mass = bodyB.getInvMass();
		effect = -applied_impulse * inv_mass;
		bodyB.getPseudoVelocity().translate(N, effect);

		transform(bodyB.getInvIws(), VecData.RbxN, temp, i);
		bodyB.getPseudoAngularVelocity().translate(temp, effect);
	}

	private final void add(Vector3f left, Vector3f right, VecData dest, int i) {
		int iDest = getVecDataIndex(dest, i);

		this.vecData[3 * iDest] = left.x + right.x;
		this.vecData[3 * iDest + 1] = left.y + right.y;
		this.vecData[3 * iDest + 2] = left.z + right.z;
	}

	private final void sub(Vector3f left, Vector3f right, VecData dest, int i) {
		int iDest = getVecDataIndex(dest, i);

		this.vecData[3 * iDest] = left.x - right.x;
		this.vecData[3 * iDest + 1] = left.y - right.y;
		this.vecData[3 * iDest + 2] = left.z - right.z;
	}

	private final void sub(VecData left, Vector3f right, Vector3f dest, int i) {
		int iLeft = getVecDataIndex(left, i);

		dest.x = this.vecData[3 * iLeft] - right.x;
		dest.y = this.vecData[3 * iLeft + 1] - right.y;
		dest.z = this.vecData[3 * iLeft + 2] - right.z;
	}

	private void sub(VecData left, Vector3f right, VecData dest, int i) {
		int iLeft = getVecDataIndex(left, i);
		int iDest = getVecDataIndex(dest, i);

		this.vecData[3 * iDest] = this.vecData[3 * iLeft] - right.x;
		this.vecData[3 * iDest + 1] = this.vecData[3 * iLeft + 1] - right.y;
		this.vecData[3 * iDest + 2] = this.vecData[3 * iLeft + 2] - right.z;
	}

	public final void cross(VecData left, VecData right, VecData dest, int i) {
		int iLeft = getVecDataIndex(left, i);
		int iRight = getVecDataIndex(right, i);
		int iDest = getVecDataIndex(dest, i);

		float x1 = this.vecData[3 * iLeft];
		float y1 = this.vecData[3 * iLeft + 1];
		float z1 = this.vecData[3 * iLeft + 2];

		float x2 = this.vecData[3 * iRight];
		float y2 = this.vecData[3 * iRight + 1];
		float z2 = this.vecData[3 * iRight + 2];

		this.vecData[3 * iDest] = y1 * z2 - z1 * y2;
		this.vecData[3 * iDest + 1] = z1 * x2 - x1 * z2;
		this.vecData[3 * iDest + 2] = x1 * y2 - y1 * x2;
	}

	private final void cross(VecData left, Vector3f right, VecData dest, int i) {
		int iLeft = getVecDataIndex(left, i);
		int iDest = getVecDataIndex(dest, i);

		float x1 = this.vecData[3 * iLeft];
		float y1 = this.vecData[3 * iLeft + 1];
		float z1 = this.vecData[3 * iLeft + 2];

		float x2 = right.x;
		float y2 = right.y;
		float z2 = right.z;

		this.vecData[3 * iDest] = y1 * z2 - z1 * y2;
		this.vecData[3 * iDest + 1] = z1 * x2 - x1 * z2;
		this.vecData[3 * iDest + 2] = x1 * y2 - y1 * x2;
	}

	private final void cross(Vector3f left, VecData right, Vector3f dest, int i) {
		int iRight = getVecDataIndex(right, i);

		float x1 = left.x;
		float y1 = left.y;
		float z1 = left.z;

		float x2 = this.vecData[3 * iRight];
		float y2 = this.vecData[3 * iRight + 1];
		float z2 = this.vecData[3 * iRight + 2];

		dest.x = y1 * z2 - z1 * y2;
		dest.y = z1 * x2 - x1 * z2;
		dest.z = x1 * y2 - y1 * x2;
	}

	private final void cross(VecData left, Vector3f right, Vector3f dest, int i) {
		int iLeft = getVecDataIndex(left, i);

		float x1 = this.vecData[3 * iLeft];
		float y1 = this.vecData[3 * iLeft + 1];
		float z1 = this.vecData[3 * iLeft + 2];

		float x2 = right.x;
		float y2 = right.y;
		float z2 = right.z;

		dest.x = y1 * z2 - z1 * y2;
		dest.y = z1 * x2 - x1 * z2;
		dest.z = x1 * y2 - y1 * x2;
	}

	public final float dot(VecData left, VecData right, int i) {
		int iLeft = getVecDataIndex(left, i);
		int iRight = getVecDataIndex(right, i);

		float x1 = this.vecData[3 * iLeft];
		float y1 = this.vecData[3 * iLeft + 1];
		float z1 = this.vecData[3 * iLeft + 2];

		float x2 = this.vecData[3 * iRight];
		float y2 = this.vecData[3 * iRight + 1];
		float z2 = this.vecData[3 * iRight + 2];

		return x1 * x2 + y1 * y2 + z1 * z2;
	}

	private final float dot(VecData left, Vector3f right, int i) {
		int iLeft = getVecDataIndex(left, i);

		float x1 = this.vecData[3 * iLeft];
		float y1 = this.vecData[3 * iLeft + 1];
		float z1 = this.vecData[3 * iLeft + 2];

		float x2 = right.x;
		float y2 = right.y;
		float z2 = right.z;

		return x1 * x2 + y1 * y2 + z1 * z2;
	}

	private final float dot(Vector3f left, VecData right, int i) {
		int iRight = getVecDataIndex(right, i);

		float x1 = left.x;
		float y1 = left.y;
		float z1 = left.z;

		float x2 = this.vecData[3 * iRight];
		float y2 = this.vecData[3 * iRight + 1];
		float z2 = this.vecData[3 * iRight + 2];

		return x1 * x2 + y1 * y2 + z1 * z2;
	}

	private final void transform(Matrix3f mat, VecData vec, Vector3f dest, int i) {
		int iVec = getVecDataIndex(vec, i);

		float x = this.vecData[3 * iVec];
		float y = this.vecData[3 * iVec + 1];
		float z = this.vecData[3 * iVec + 2];

		dest.x = mat.m00 * x + mat.m10 * y + mat.m20 * z;
		dest.y = mat.m01 * x + mat.m11 * y + mat.m21 * z;
		dest.z = mat.m02 * x + mat.m12 * y + mat.m22 * z;
	}

	private final float sandwichProduct(Matrix3f mat, VecData vec, int i) {
		int iVec = getVecDataIndex(vec, i);

		float x = this.vecData[3 * iVec];
		float y = this.vecData[3 * iVec + 1];
		float z = this.vecData[3 * iVec + 2];

		return x * (mat.m00 * x + mat.m10 * y + mat.m20 * z) + y * (mat.m01 * x + mat.m11 * y + mat.m21 * z)
				+ z * (mat.m02 * x + mat.m12 * y + mat.m22 * z);
	}

	private final void store(float x, float y, float z, VecData dest, int i) {
		int iDest = getVecDataIndex(dest, i);

		this.vecData[3 * iDest] = x;
		this.vecData[3 * iDest + 1] = y;
		this.vecData[3 * iDest + 2] = z;
	}

	public final float getFloat(FloatData data, int i) {
		return this.floatData[getFloatDataIndex(data, i)];
	}

	public final void setFloat(FloatData data, float value, int i) {
		this.floatData[getFloatDataIndex(data, i)] = value;
	}

	public final void getVec(VecData data, Vector3f dest, int i) {
		int index = getVecDataIndex(data, i);
		dest.x = this.vecData[3 * index];
		dest.y = this.vecData[3 * index + 1];
		dest.z = this.vecData[3 * index + 2];
	}

	public final void setVec(Vector3f src, VecData dest, int i) {
		int index = getVecDataIndex(dest, i);
		this.vecData[3 * index] = src.x;
		this.vecData[3 * index + 1] = src.y;
		this.vecData[3 * index + 2] = src.z;
	}

	private static final int getFloatDataIndex(FloatData data, int i) {
		return data.ordinal() + FloatData.END.ordinal() * i;
	}

	private static final int getVecDataIndex(VecData data, int i) {
		return data.ordinal() + VecData.END.ordinal() * i;
	}

}
