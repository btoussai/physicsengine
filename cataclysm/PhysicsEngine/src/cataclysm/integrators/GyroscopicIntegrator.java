package cataclysm.integrators;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import cataclysm.wrappers.RigidBody;

/**
 * Permet d'intégrer le terme gyroscopique (W x Ib * W) dans l'équation
 * différentielle portant sur la rotation d'un objet.
 * 
 * @author Briac
 *
 */
public class GyroscopicIntegrator {

	private Vector3f X = new Vector3f();
	private Vector3f dX = new Vector3f();
	private Vector3f f = new Vector3f();
	private Matrix3f J = new Matrix3f();
	private Matrix3f invJ = new Matrix3f();

	/**
	 * Effectue une itération de la méthode de Newton pour résoudre l'équation
	 * différentielle sur la rotation, incluant le terme gyroscopique.
	 * 
	 * @param timeStep
	 * @param body
	 */
	public void integrateGyroscopicTerm(float timeStep, RigidBody body) {
		solveInBodySpace(timeStep, body.getOrientation(), body.getI0(), body.getAngularVelocity());
	}

	/**
	 * @param timeStep Le pas de temps.
	 * @param rotation La rotation du repère barycentrique.
	 * @param inertia Les moments principaux d'inertie.
	 * @param angularVelocity La vitesse angulaire en world-space.
	 */
	private void solveInBodySpace(float timeStep, Matrix3f rotation, Vector3f inertia, Vector3f angularVelocity) {

		// X = transpose( rotation) * angularVelocity
		X.x = rotation.m00 * angularVelocity.x + rotation.m01 * angularVelocity.y + rotation.m02 * angularVelocity.z;
		X.y = rotation.m10 * angularVelocity.x + rotation.m11 * angularVelocity.y + rotation.m12 * angularVelocity.z;
		X.z = rotation.m20 * angularVelocity.x + rotation.m21 * angularVelocity.y + rotation.m22 * angularVelocity.z;

		float I1 = inertia.x;
		float I2 = inertia.y;
		float I3 = inertia.z;

		float I3_minus_I2 = I3 - I2;
		float I1_minus_I3 = I1 - I3;
		float I2_minus_I1 = I2 - I1;

		if (Math.abs(I1_minus_I3) + Math.abs(I2_minus_I1) + Math.abs(I3_minus_I2) < Epsilons.MIN_LENGTH) {
			return;
		}

		f.x = timeStep * I3_minus_I2 * X.y * X.z;
		f.y = timeStep * I1_minus_I3 * X.x * X.z;
		f.z = timeStep * I2_minus_I1 * X.x * X.y;

		J.m00 = I1;
		J.m01 = timeStep * I1_minus_I3 * X.z;
		J.m02 = timeStep * I2_minus_I1 * X.y;

		J.m10 = timeStep * I3_minus_I2 * X.z;
		J.m11 = I2;
		J.m12 = timeStep * I2_minus_I1 * X.x;

		J.m20 = timeStep * I3_minus_I2 * X.y;
		J.m21 = timeStep * I1_minus_I3 * X.x;
		J.m22 = I3;

		Matrix3f.invert(J, invJ);
		Matrix3f.transform(invJ, f, dX);
		Vector3f.sub(X, dX, X);

		Matrix3f.transform(rotation, X, angularVelocity);
	}

}
