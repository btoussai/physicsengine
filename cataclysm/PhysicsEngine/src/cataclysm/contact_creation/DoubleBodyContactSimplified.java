package cataclysm.contact_creation;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.Wrapper;
import math.Clamp;
import math.MatrixOps;

/**
 * Représente un contact entre deux wrappers. Les wrappers ne se touchent pas
 * forcément, le contact n'est alors pas actif.
 * 
 * @author Briac
 *
 */
public class DoubleBodyContactSimplified extends AbstractDoubleBodyContact {

	private enum VecData {
		Ra, Rb, RaxN, RbxN, RaxT, RbxT, RaxB, RbxB, END;
	}

	private enum FloatData {
		deltaV_N, deltaV_T, deltaV_B, pseudo_deltaV, bias, pseudo_bias, inv_mass_N, inv_mass_T, inv_mass_B, impulses_N,
		impulses_T, impulses_B, pseudo_impulses, END;
	}

	private final Vector3f N = new Vector3f();// normal vector
	private final Vector3f T = new Vector3f();// tangent vector
	private final Vector3f B = new Vector3f();// bitangent vector
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
	public void velocityStart() {
		super.mixContactProperties(wrapperA.getBody().getContactProperties(),
				wrapperB.getBody().getContactProperties());
		area.getNormal().negate(N);
		MatrixOps.computeOrthogonalComplement(N, T, B);

		for (int i = 0; i < super.area.getContactCount(); i++) {
			buildJacobian(i);
			computeInvMass(i);
			computeVelocityError(i);
			float deltaV_N = getFloat(FloatData.deltaV_N, i);
			if (deltaV_N < -Epsilons.VELOCITY_ELASTICITY_LIMIT) {
				setFloat(FloatData.bias, elasticity * deltaV_N, i);
			} else {
				setFloat(FloatData.bias, 0, i);
			}
		}
	}
	
	@Override
	public void resetImpulses() {
		for (int i = 0; i < super.getMaxContacts(); i++) {
			setFloat(FloatData.impulses_N, 0, i);
			setFloat(FloatData.impulses_T, 0, i);
			setFloat(FloatData.impulses_B, 0, i);
		}
	}
	
	@Override
	public void warmStart() {
		for (int i = 0; i < super.getMaxContacts(); i++) {
			float applied_impulse_N = getFloat(FloatData.impulses_N, i);
			float applied_impulse_T = getFloat(FloatData.impulses_T, i);
			float applied_impulse_B = getFloat(FloatData.impulses_B, i);
			float Jx = N.x * applied_impulse_N + T.x * applied_impulse_T + B.x * applied_impulse_B;
			float Jy = N.y * applied_impulse_N + T.y * applied_impulse_T + B.y * applied_impulse_B;
			float Jz = N.z * applied_impulse_N + T.z * applied_impulse_T + B.z * applied_impulse_B;

			applyImpulse(i, Jx, Jy, Jz);
		}
	}

	@Override
	public void solveVelocity() {

		for (int i = 0; i < super.area.getContactCount(); i++) {

			// the velocity may have changed due to other constraints, we need to recompute
			// it.
			computeVelocityError(i);
			
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

			// Bitangent impulse
			float prev_impulse_B = getFloat(FloatData.impulses_B, i);
			float impulse_B = prev_impulse_B - getFloat(FloatData.deltaV_B, i) / getFloat(FloatData.inv_mass_B, i);
			impulse_B = Clamp.clamp(impulse_B, -friction * impulse_N, friction * impulse_N);
			setFloat(FloatData.impulses_B, impulse_B, i);
			float applied_impulse_B = impulse_B - prev_impulse_B;

			float Jx = N.x * applied_impulse_N + T.x * applied_impulse_T + B.x * applied_impulse_B;
			float Jy = N.y * applied_impulse_N + T.y * applied_impulse_T + B.y * applied_impulse_B;
			float Jz = N.z * applied_impulse_N + T.z * applied_impulse_T + B.z * applied_impulse_B;

			applyImpulse(i, Jx, Jy, Jz);
		}
	}

	@Override
	public void positionStart(float timeStep) {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			setFloat(FloatData.pseudo_impulses, 0, i);
			float pseudo_bias = (Epsilons.PENETRATION_RECOVERY / timeStep)
					* Math.min(0, (area.penetrations[i] + Epsilons.ALLOWED_PENETRATION));
			setFloat(FloatData.pseudo_bias, pseudo_bias, i);
		}
	}

	@Override
	public void solvePosition() {
		for (int i = 0; i < super.area.getContactCount(); i++) {
			computePositionError(i);

			float prev_impulse = getFloat(FloatData.pseudo_impulses, i);
			float impulse = prev_impulse - (getFloat(FloatData.pseudo_deltaV, i) + getFloat(FloatData.pseudo_bias, i))
					/ getFloat(FloatData.inv_mass_N, i);
			impulse = Math.max(0, impulse);
			setFloat(FloatData.pseudo_impulses, impulse, i);
			float applied_impulse = impulse - prev_impulse;

			applyPseudoImpulse(i, applied_impulse);
		}
	}

	private final void buildJacobian(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Vector3f[] contacts = area.getContactPoints();
		sub(contacts[i], bodyA.getPosition(), VecData.Ra, i);
		sub(contacts[i], bodyB.getPosition(), VecData.Rb, i);

		cross(VecData.Ra, N, VecData.RaxN, i);
		cross(VecData.Rb, N, VecData.RbxN, i);
		cross(VecData.Ra, T, VecData.RaxT, i);
		cross(VecData.Rb, T, VecData.RbxT, i);
		cross(VecData.Ra, B, VecData.RaxB, i);
		cross(VecData.Rb, B, VecData.RbxB, i);
	}

	private final void computeInvMass(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Matrix3f IA_inv = bodyA.getInvIws();
		Matrix3f IB_inv = bodyB.getInvIws();

		float inv_mass_N = bodyA.getInvMass() * (1.0f + sandwichDotProduct(IA_inv, VecData.RaxN, i));
		inv_mass_N += bodyB.getInvMass() * (1.0f + sandwichDotProduct(IB_inv, VecData.RbxN, i));

		float inv_mass_T = bodyA.getInvMass() * (1.0f + sandwichDotProduct(IA_inv, VecData.RaxT, i));
		inv_mass_T += bodyB.getInvMass() * (1.0f + sandwichDotProduct(IB_inv, VecData.RbxT, i));

		float inv_mass_B = bodyA.getInvMass() * (1.0f + sandwichDotProduct(IA_inv, VecData.RaxB, i));
		inv_mass_B += bodyB.getInvMass() * (1.0f + sandwichDotProduct(IB_inv, VecData.RbxB, i));

		setFloat(FloatData.inv_mass_N, inv_mass_N, i);
		setFloat(FloatData.inv_mass_T, inv_mass_T, i);
		setFloat(FloatData.inv_mass_B, inv_mass_B, i);
	}

	private final void computeVelocityError(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		Vector3f Va = bodyA.getVelocity();
		Vector3f Wa = bodyA.getAngularVelocity();

		int indexRa = getVecDataIndex(VecData.Ra, i);
		float Rax = this.vecData[3 * indexRa];
		float Ray = this.vecData[3 * indexRa + 1];
		float Raz = this.vecData[3 * indexRa + 2];

		// dv = Va + Wa x Ra
		float dvx = Va.x + Wa.y * Raz - Wa.z * Ray;
		float dvy = Va.y + Wa.z * Rax - Wa.x * Raz;
		float dvz = Va.z + Wa.x * Ray - Wa.y * Rax;

		Vector3f Vb = bodyB.getVelocity();
		Vector3f Wb = bodyB.getAngularVelocity();

		int indexRb = getVecDataIndex(VecData.Rb, i);
		float Rbx = this.vecData[3 * indexRb];
		float Rby = this.vecData[3 * indexRb + 1];
		float Rbz = this.vecData[3 * indexRb + 2];

		// dv -= Vb + Wb x Rb
		dvx -= Vb.x + Wb.y * Rbz - Wb.z * Rby;
		dvy -= Vb.y + Wb.z * Rbx - Wb.x * Rbz;
		dvz -= Vb.z + Wb.x * Rby - Wb.y * Rbx;

		setFloat(FloatData.deltaV_N, dvx * N.x + dvy * N.y + dvz * N.z, i);// dv . N
		setFloat(FloatData.deltaV_T, dvx * T.x + dvy * T.y + dvz * T.z, i);// dv . T
		setFloat(FloatData.deltaV_B, dvx * B.x + dvy * B.y + dvz * B.z, i);// dv . B

	}

	private final void applyImpulse(int i, float Jx, float Jy, float Jz) {
		RigidBody bodyA = wrapperA.getBody();
		float inv_mass = bodyA.getInvMass();

		float dvx = Jx * inv_mass;
		float dvy = Jy * inv_mass;
		float dvz = Jz * inv_mass;
		bodyA.getVelocity().translate(dvx, dvy, dvz);

		int indexR = getVecDataIndex(VecData.Ra, i);
		float Rx = this.vecData[3 * indexR];
		float Ry = this.vecData[3 * indexR + 1];
		float Rz = this.vecData[3 * indexR + 2];
		float Tx = Ry * dvz - Rz * dvy;
		float Ty = Rz * dvx - Rx * dvz;
		float Tz = Rx * dvy - Ry * dvx;
		Matrix3f inv_Iws = bodyA.getInvIws();
		float dwx = inv_Iws.m00 * Tx + inv_Iws.m10 * Ty + inv_Iws.m20 * Tz;
		float dwy = inv_Iws.m01 * Tx + inv_Iws.m11 * Ty + inv_Iws.m21 * Tz;
		float dwz = inv_Iws.m02 * Tx + inv_Iws.m12 * Ty + inv_Iws.m22 * Tz;
		bodyA.getAngularVelocity().translate(dwx, dwy, dwz);

		RigidBody bodyB = wrapperB.getBody();
		inv_mass = -bodyB.getInvMass();

		dvx = Jx * inv_mass;
		dvy = Jy * inv_mass;
		dvz = Jz * inv_mass;
		bodyB.getVelocity().translate(dvx, dvy, dvz);

		indexR = getVecDataIndex(VecData.Rb, i);
		Rx = this.vecData[3 * indexR];
		Ry = this.vecData[3 * indexR + 1];
		Rz = this.vecData[3 * indexR + 2];
		Tx = Ry * dvz - Rz * dvy;
		Ty = Rz * dvx - Rx * dvz;
		Tz = Rx * dvy - Ry * dvx;
		inv_Iws = bodyB.getInvIws();
		dwx = inv_Iws.m00 * Tx + inv_Iws.m10 * Ty + inv_Iws.m20 * Tz;
		dwy = inv_Iws.m01 * Tx + inv_Iws.m11 * Ty + inv_Iws.m21 * Tz;
		dwz = inv_Iws.m02 * Tx + inv_Iws.m12 * Ty + inv_Iws.m22 * Tz;
		bodyB.getAngularVelocity().translate(dwx, dwy, dwz);

	}

	private final void computePositionError(int i) {
		RigidBody bodyA = wrapperA.getBody();
		RigidBody bodyB = wrapperB.getBody();

		float deltaV = 0;
		deltaV += Vector3f.dot(bodyA.getPseudoVelocity(), N);
		deltaV += dot(bodyA.getPseudoAngularVelocity(), VecData.RaxN, i);
		deltaV -= Vector3f.dot(bodyB.getPseudoVelocity(), N);
		deltaV -= dot(bodyB.getPseudoAngularVelocity(), VecData.RbxN, i);
		setFloat(FloatData.pseudo_deltaV, deltaV, i);
	}

	private final void applyPseudoImpulse(int i, float applied_impulse) {
		RigidBody bodyA = wrapperA.getBody();
		float effect = applied_impulse * bodyA.getInvMass();

		bodyA.getPseudoVelocity().translate(N, effect);
		Matrix3f inv_Iws = bodyA.getInvIws();
		int indexRxN = getVecDataIndex(VecData.RaxN, i);
		float RxNx = this.vecData[3 * indexRxN];
		float RxNy = this.vecData[3 * indexRxN + 1];
		float RxNz = this.vecData[3 * indexRxN + 2];
		float dwx = inv_Iws.m00 * RxNx + inv_Iws.m10 * RxNy + inv_Iws.m20 * RxNz;
		float dwy = inv_Iws.m01 * RxNx + inv_Iws.m11 * RxNy + inv_Iws.m21 * RxNz;
		float dwz = inv_Iws.m02 * RxNx + inv_Iws.m12 * RxNy + inv_Iws.m22 * RxNz;
		bodyA.getPseudoAngularVelocity().translate(dwx * effect, dwy * effect, dwz * effect);

		RigidBody bodyB = wrapperB.getBody();
		effect = -applied_impulse * bodyB.getInvMass();

		bodyB.getPseudoVelocity().translate(N, effect);
		inv_Iws = bodyB.getInvIws();
		indexRxN = getVecDataIndex(VecData.RbxN, i);
		RxNx = this.vecData[3 * indexRxN];
		RxNy = this.vecData[3 * indexRxN + 1];
		RxNz = this.vecData[3 * indexRxN + 2];
		dwx = inv_Iws.m00 * RxNx + inv_Iws.m10 * RxNy + inv_Iws.m20 * RxNz;
		dwy = inv_Iws.m01 * RxNx + inv_Iws.m11 * RxNy + inv_Iws.m21 * RxNz;
		dwz = inv_Iws.m02 * RxNx + inv_Iws.m12 * RxNy + inv_Iws.m22 * RxNz;
		bodyB.getPseudoAngularVelocity().translate(dwx * effect, dwy * effect, dwz * effect);

	}

	private final void sub(Vector3f left, Vector3f right, VecData dest, int i) {
		int iDest = getVecDataIndex(dest, i);

		this.vecData[3 * iDest] = left.x - right.x;
		this.vecData[3 * iDest + 1] = left.y - right.y;
		this.vecData[3 * iDest + 2] = left.z - right.z;
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

	private final float sandwichDotProduct(Matrix3f mat, VecData vec, int i) {
		int iVec = getVecDataIndex(vec, i);

		float x = this.vecData[3 * iVec];
		float y = this.vecData[3 * iVec + 1];
		float z = this.vecData[3 * iVec + 2];

		return x * (mat.m00 * x + mat.m10 * y + mat.m20 * z) + y * (mat.m01 * x + mat.m11 * y + mat.m21 * z)
				+ z * (mat.m02 * x + mat.m12 * y + mat.m22 * z);
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
