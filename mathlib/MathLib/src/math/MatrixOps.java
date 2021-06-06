package math;

import math.vector.Matrix3f;
import math.vector.Matrix4f;
import math.vector.Vector2f;
import math.vector.Vector3f;

/**
 * Cette classe permet de cr�er diff�rents types de matrices: transformation,
 * projection, etc..
 * 
 * @author Briac
 *
 */
public class MatrixOps {

	private static final float sqrt2_over_2 = 0.5f * (float) Math.sqrt(2.0);

	/**
	 * Convertit une matrice de rotation en angles d'Euler sous la forme d'un
	 * vecteur, chaque composante correspond � une rotation autour de l'axe de cette
	 * composante. La rotation est exprim�e en degr�s.
	 * 
	 * @param mat
	 * @return
	 */
	public static Vector3f matrixToEulerAngle(Matrix4f mat) {
		double rotX, rotY, rotZ;
		rotY = Math.asin(mat.m20);
		double C = Math.cos(rotY);

		if (Math.abs(C) > 0.0005) {
			rotX = Math.atan2(-mat.m21 / C, mat.m22 / C);
			rotZ = Math.atan2(-mat.m10 / C, mat.m00 / C);
		} else {
			rotX = 0;

			rotZ = Math.atan2(mat.m11, mat.m01);

		}

		rotX *= 180 / Math.PI;
		rotY *= 180 / Math.PI;
		rotZ *= 180 / Math.PI;

		return new Vector3f((float) rotX, (float) rotY, (float) rotZ);
	}

	/**
	 * Convertit une matrice de rotation en une repr�sentation (axe, angle). Ce code
	 * est bas� sur
	 * {@link "http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm"}
	 * 
	 * @param rotation La matrice de rotation.
	 * @param dest     Un vecteur dans lequel stocker l'axe de la rotation
	 * @return L'angle de la rotation, en radians.
	 */
	public static float matrixToAxisAngle(Matrix3f rotation, Vector3f dest) {
		float xs = rotation.m12 - rotation.m21;
		float ys = rotation.m20 - rotation.m02;
		float zs = rotation.m01 - rotation.m10;

		float x, y, z;
		float epsilon = 1E-4f; // margin to allow for rounding errors
		float epsilon2 = 1E-3f; // margin to distinguish between 0 and 180 degrees

		if ((Math.abs(zs) < epsilon) && (Math.abs(ys) < epsilon) && (Math.abs(xs) < epsilon)) {
			// singularity found
			// first check for identity matrix which must have +1 for all terms
			// in leading diagonal and zero in other terms
			float xy = (rotation.m01 + rotation.m10) * 0.25f;
			float xz = (rotation.m02 + rotation.m20) * 0.25f;
			float yz = (rotation.m12 + rotation.m21) * 0.25f;

			if ((Math.abs(xy) < epsilon2) && (Math.abs(xz) < epsilon2) && (Math.abs(yz) < epsilon2)
					&& (Math.abs(rotation.m00 + rotation.m11 + rotation.m22 - 3) < epsilon2)) {
				// this singularity is identity matrix so angle = 0
				dest.set(0, 1, 0);
				return 0; // zero angle, arbitrary axis
			}
			// otherwise this singularity is angle = 180
			float xx = (rotation.m00 + 1.0f) * 0.5f;
			float yy = (rotation.m11 + 1.0f) * 0.5f;
			float zz = (rotation.m22 + 1.0f) * 0.5f;
			if ((xx > yy) && (xx > zz)) { // m[0][0] is the largest diagonal term
				if (xx < epsilon) {
					x = 0;
					y = sqrt2_over_2;
					z = sqrt2_over_2;
				} else {
					x = (float) Math.sqrt(xx);
					y = xy / x;
					z = xz / x;
				}
			} else if (yy > zz) { // m[1][1] is the largest diagonal term
				if (yy < epsilon) {
					x = sqrt2_over_2;
					y = 0;
					z = sqrt2_over_2;
				} else {
					y = (float) Math.sqrt(yy);
					x = xy / y;
					z = yz / y;
				}
			} else { // m[2][2] is the largest diagonal term
				if (zz < epsilon) {
					x = sqrt2_over_2;
					y = sqrt2_over_2;
					z = 0;
				} else {
					z = (float) Math.sqrt(zz);
					x = xz / z;
					y = yz / z;
				}
			}
			dest.set(x, y, z);
			return (float) Math.PI;
		}

		float c = Clamp.symetricClamp(0.5f * (rotation.m00 + rotation.m11 + rotation.m22 - 1.0f));
		float angle = (float) Math.acos(c);

		float s = xs * xs + ys * ys + zs * zs;
		s = (float) Math.sqrt(s);
		x = xs / s;
		y = ys / s;
		z = zs / s;
		dest.set(x, y, z);
		return angle;
	}

	/**
	 * Calcule les valeurs propres et vecteurs propres de la matrice mat, symétrique
	 * réelle. Plus précisément: <br>
	 * <br>
	 * mat = eigenVectors * diag(eigenValues) * transpose(eigenVectors) <br>
	 * Cet algorithme est issu de: <br>
	 * <a href=
	 * "https://fr.wikipedia.org/wiki/Algorithme_de_recherche_de_valeur_propre">https://fr.wikipedia.org/wiki/Algorithme_de_recherche_de_valeur_propre</a>
	 * <br>
	 * et <br>
	 * <a href=
	 * "https://www.geometrictools.com/Documentation/RobustEigenSymmetric3x3.pdf">https://www.geometrictools.com/Documentation/RobustEigenSymmetric3x3.pdf</a>
	 * 
	 * @param mat          La matrice dont on veut calculer les valeurs propres.
	 * @param eigenValues  Un vecteur dans lequel les valeurs propres seront
	 *                     rangées.
	 * @param eigenVectors Une matrice dans laquelle les vecteurs propres seront
	 *                     rangés par colonne.
	 */
	public static void eigenValues(Matrix3f mat, Vector3f eigenValues, Matrix3f eigenVectors) {
		float norm = mat.m01 * mat.m01 + mat.m02 * mat.m02 + mat.m12 * mat.m12;
		if (norm == 0) {
			eigenValues.set(mat.m00, mat.m11, mat.m22);
			eigenVectors.setIdentity();
			return;
		}

		float trM = mat.m00 + mat.m11 + mat.m22;
		float q = trM / 3.0f;

		float b00 = mat.m00 - q;
		float b11 = mat.m11 - q;
		float b22 = mat.m22 - q;

		float p = b00 * b00 + b11 * b11 + b22 * b22 + 2.0f * norm;
		p = (float) Math.sqrt(p / 6.0f);// (tr( (M - qI)� ) / 6) ^ (1/2)

		// B = (A - qI) / p
		float detB = b00 * (b11 * b22 - mat.m21 * mat.m21) - mat.m10 * (mat.m10 * b22 - mat.m20 * mat.m21)
				+ mat.m20 * (mat.m10 * mat.m21 - mat.m20 * b11);
		detB /= (p * p * p);// detB in [-2; 2]

		// theta in [0; pi/3]
		double theta = Math.acos(Math.min(Math.max(0.5 * detB, -1.0), 1.0)) / 3.0;

		// beta0 < beta1 < beta2
		float beta2 = 2.0f * (float) Math.cos(theta);
		float beta0 = 2.0f * (float) Math.cos(theta + 2.0 / 3.0 * Math.PI);
		float beta1 = -(beta0 + beta2);// because cos(x) + cos(x + 2*pi/3) + cos(x + 4*pi/3) = 0

		// lambda = p * beta + q
		float lambda0 = p * beta0 + q;
		float lambda1 = p * beta1 + q;
		float lambda2 = p * beta2 + q;

		Vector3f temp1 = new Vector3f();
		Vector3f temp2 = new Vector3f();
		Vector3f temp3 = new Vector3f();
		Vector3f eigenVector0 = new Vector3f();
		Vector3f eigenVector1 = new Vector3f();
		Vector3f eigenVector2 = new Vector3f();

		if (detB >= 0) {
			computeEigenVector0(mat, temp1, temp2, temp3, lambda2, eigenVector2);
			computeEigenVector1(mat, temp1, temp2, temp3, eigenVector2, lambda1, eigenVector1);
			Vector3f.cross(eigenVector1, eigenVector2, eigenVector0);
		} else {
			computeEigenVector0(mat, temp1, temp2, temp3, lambda0, eigenVector0);
			computeEigenVector1(mat, temp1, temp2, temp3, eigenVector0, lambda1, eigenVector1);
			Vector3f.cross(eigenVector0, eigenVector1, eigenVector2);
		}

		eigenVectors.m00 = eigenVector0.x;
		eigenVectors.m01 = eigenVector0.y;
		eigenVectors.m02 = eigenVector0.z;

		eigenVectors.m10 = eigenVector1.x;
		eigenVectors.m11 = eigenVector1.y;
		eigenVectors.m12 = eigenVector1.z;

		eigenVectors.m20 = eigenVector2.x;
		eigenVectors.m21 = eigenVector2.y;
		eigenVectors.m22 = eigenVector2.z;

		eigenValues.set(lambda0, lambda1, lambda2);
	}

	private static void computeEigenVector0(Matrix3f mat, Vector3f temp1, Vector3f temp2, Vector3f temp3, float lambda0,
			Vector3f evec0) {

		evec0.set(mat.m00 - lambda0, mat.m10, mat.m20);
		temp2.set(mat.m01, mat.m11 - lambda0, mat.m21);
		temp3.set(mat.m02, mat.m12, mat.m22 - lambda0);

		Vector3f.cross(temp2, temp3, temp1);
		Vector3f.cross(evec0, temp3, temp3);
		Vector3f.cross(evec0, temp2, temp2);

		float d1 = temp1.lengthSquared();
		float d2 = temp2.lengthSquared();
		float d3 = temp3.lengthSquared();

		float d = 0;
		if (d1 > d2) {
			if (d1 > d3) {// d1 is the largest
				evec0.set(temp1);
				d = d1;
			} else {// d3 is the largest
				evec0.set(temp3);
				d = d3;
			}
		} else {
			if (d2 > d3) {// d2 is the largest
				evec0.set(temp2);
				d = d2;
			} else {// d3 is the largest
				evec0.set(temp3);
				d = d3;
			}
		}

		evec0.scale(1.0f / (float) Math.sqrt(d));
	}

	/**
	 * Computes U and V from W such that [U, V, W] is an orthonormal set. W must be
	 * unit-length
	 * 
	 * @param W
	 * @param U
	 * @param V
	 */
	public static void computeOrthogonalComplement(Vector3f W, Vector3f U, Vector3f V) {
		if (Math.abs(W.x) > Math.abs(W.y)) {
			float inv_length = 1.0f / (float) Math.sqrt(W.x * W.x + W.z * W.z);
			U.set(-W.z * inv_length, 0, W.x * inv_length);

			V.set(W.y * U.z, U.x * W.z - U.z * W.x, -W.y * U.x);// Vector3f.cross(W, U, V);
		} else {
			float inv_length = 1.0f / (float) Math.sqrt(W.y * W.y + W.z * W.z);
			U.set(0, W.z * inv_length, -W.y * inv_length);

			V.set(W.y * U.z - W.z * U.y, -U.z * W.x, W.x * U.y);// Vector3f.cross(W, U, V);
		}

	}

	/**
	 * Same as {@link #computeOrthogonalComplement(Vector3f, Vector3f, Vector3f)}
	 * but the result is placed in a matrix instead, in the order [U | V | W]
	 * 
	 * @param W
	 * @param dest
	 * @return
	 */
	public static Matrix3f computeOrthogonalComplement(Vector3f W, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}
		dest.m20 = W.x;
		dest.m21 = W.y;
		dest.m22 = W.z;
		if (Math.abs(W.x) > Math.abs(W.y)) {
			float inv_length = 1.0f / (float) Math.sqrt(W.x * W.x + W.z * W.z);
			dest.m00 = -W.z * inv_length;
			dest.m01 = 0.0f;
			dest.m02 = W.x * inv_length;

			// Vector3f.cross(W, U, V);
			dest.m10 = W.y * dest.m02;
			dest.m11 = dest.m00 * W.z - dest.m02 * W.x;
			dest.m12 = -W.y * dest.m00;
		} else {
			float inv_length = 1.0f / (float) Math.sqrt(W.y * W.y + W.z * W.z);
			dest.m00 = 0.0f;
			dest.m01 = W.z * inv_length;
			dest.m02 = -W.y * inv_length;

			// Vector3f.cross(W, U, V);
			dest.m10 = W.y * dest.m02 - W.z * dest.m01;
			dest.m11 = -dest.m02 * W.x;
			dest.m12 = W.x * dest.m01;
		}

		return dest;
	}

	/**
	 * Same as {@link #computeOrthogonalComplement(Vector3f, Vector3f, Vector3f)}
	 * but the result is placed in a matrix instead, in the order [U | V | W]
	 * 
	 * @param W
	 * @param dest
	 * @return
	 */
	public static Matrix4f computeOrthogonalComplement(Vector3f W, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}
		dest.m20 = W.x;
		dest.m21 = W.y;
		dest.m22 = W.z;
		if (Math.abs(W.x) > Math.abs(W.y)) {
			float inv_length = 1.0f / (float) Math.sqrt(W.x * W.x + W.z * W.z);
			dest.m00 = -W.z * inv_length;
			dest.m01 = 0.0f;
			dest.m02 = W.x * inv_length;

			// Vector3f.cross(W, U, V);
			dest.m10 = W.y * dest.m02;
			dest.m11 = dest.m00 * W.z - dest.m02 * W.x;
			dest.m12 = -W.y * dest.m00;
		} else {
			float inv_length = 1.0f / (float) Math.sqrt(W.y * W.y + W.z * W.z);
			dest.m00 = 0.0f;
			dest.m01 = W.z * inv_length;
			dest.m02 = -W.y * inv_length;

			// Vector3f.cross(W, U, V);
			dest.m10 = W.y * dest.m02 - W.z * dest.m01;
			dest.m11 = -dest.m02 * W.x;
			dest.m12 = W.x * dest.m01;
		}

		dest.m03 = dest.m13 = dest.m23 = dest.m30 = dest.m31 = dest.m32 = 0.0f;
		dest.m33 = 1.0f;

		return dest;
	}

	private static void computeEigenVector1(Matrix3f mat, Vector3f temp1, Vector3f temp2, Vector3f temp3,
			Vector3f evec0, float eval1, Vector3f evec1) {

		computeOrthogonalComplement(evec0, temp1, temp2);// W, U, V

		Matrix3f.transform(mat, temp1, temp3);// A * U
		float m00 = Vector3f.dot(temp1, temp3) - eval1;// U^T * A * U

		Matrix3f.transform(mat, temp2, temp3);// A * V
		float m01 = Vector3f.dot(temp1, temp3);// U^T * A * V
		float m11 = Vector3f.dot(temp2, temp3) - eval1;// V^T * A * V

		float absM00 = Math.abs(m00);
		float absM01 = Math.abs(m01);
		float absM11 = Math.abs(m11);

		if (absM00 >= absM11) {
			if (Math.max(absM00, absM01) > 0) {
				if (absM00 >= absM01) {
					m01 /= m00;
					m00 = 1.0f / (float) Math.sqrt(1.0f + m01 * m01);
					m01 *= m00;
				} else {
					m00 /= m01;
					m01 = 1.0f / (float) Math.sqrt(1.0f + m00 * m00);
					m00 *= m01;
				}
				evec1.x = m01 * temp1.x - m00 * temp2.x;
				evec1.y = m01 * temp1.y - m00 * temp2.y;
				evec1.z = m01 * temp1.z - m00 * temp2.z;
			} else {
				evec1.set(temp1);
			}
		} else {
			if (Math.max(absM11, absM01) > 0) {
				if (absM11 >= absM01) {
					m01 /= m11;
					m11 = 1.0f / (float) Math.sqrt(1.0f + m01 * m01);
					m01 *= m11;
				} else {
					m11 /= m01;
					m01 = 1.0f / (float) Math.sqrt(1.0f + m11 * m11);
					m11 *= m01;
				}
				evec1.x = m11 * temp1.x - m01 * temp2.x;
				evec1.y = m11 * temp1.y - m01 * temp2.y;
				evec1.z = m11 * temp1.z - m01 * temp2.z;
			} else {
				evec1.set(temp1);
			}
		}
	}

	/**
	 * Construit une matrice de transformation pour un mod�le 3D.
	 * 
	 * @param rotationMatrix La matrice de rotation repr�sentant la rotation de
	 *                       l'objet.
	 * @param translation    La translation � appliquer appr�s la rotation.
	 * @param scale          Le facteur d'�chelle � appliquer � l'objet.
	 * @param dest           La matrice de destination ou null si une nouvelle
	 *                       matrice doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createTransformationMatrix(Matrix4f rotationMatrix, Vector3f translation, float scale,
			Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		dest.m00 = rotationMatrix.m00 * scale;
		dest.m10 = rotationMatrix.m10 * scale;
		dest.m20 = rotationMatrix.m20 * scale;
		dest.m30 = translation.x;

		dest.m01 = rotationMatrix.m01 * scale;
		dest.m11 = rotationMatrix.m11 * scale;
		dest.m21 = rotationMatrix.m21 * scale;
		dest.m31 = translation.y;

		dest.m02 = rotationMatrix.m02 * scale;
		dest.m12 = rotationMatrix.m12 * scale;
		dest.m22 = rotationMatrix.m22 * scale;
		dest.m32 = translation.z;

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1;

		return dest;
	}

	/**
	 * Construit une matrice de transformation pour un mod�le 3D.
	 * 
	 * @param rotationMatrix La matrice de rotation repr�sentant la rotation de
	 *                       l'objet.
	 * @param translation    La translation � appliquer appr�s la rotation.
	 * @param scale          Le facteur d'�chelle � appliquer � l'objet.
	 * @param dest           La matrice de destination ou null si une nouvelle
	 *                       matrice doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createTransformationMatrix(Matrix3f rotationMatrix, Vector3f translation, float scale,
			Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		dest.m00 = rotationMatrix.m00 * scale;
		dest.m10 = rotationMatrix.m10 * scale;
		dest.m20 = rotationMatrix.m20 * scale;
		dest.m30 = translation.x;

		dest.m01 = rotationMatrix.m01 * scale;
		dest.m11 = rotationMatrix.m11 * scale;
		dest.m21 = rotationMatrix.m21 * scale;
		dest.m31 = translation.y;

		dest.m02 = rotationMatrix.m02 * scale;
		dest.m12 = rotationMatrix.m12 * scale;
		dest.m22 = rotationMatrix.m22 * scale;
		dest.m32 = translation.z;

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1;

		return dest;
	}

	/**
	 * Construit une matrice de transformation pour un mod�le 3D.
	 * 
	 * @param rotationMatrix La matrice de rotation repr�sentant la rotation de
	 *                       l'objet.
	 * @param translation    La translation � appliquer appr�s la rotation.
	 * @param scale          Le facteur d'�chelle � appliquer � l'objet selon chaque
	 *                       axe.
	 * @param dest           La matrice de destination ou null si une nouvelle
	 *                       matrice doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createTransformationMatrix(Matrix4f rotationMatrix, Vector3f translation, Vector3f scale,
			Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		dest.m00 = rotationMatrix.m00 * scale.x;
		dest.m10 = rotationMatrix.m10 * scale.x;
		dest.m20 = rotationMatrix.m20 * scale.x;
		dest.m30 = translation.x;

		dest.m01 = rotationMatrix.m01 * scale.y;
		dest.m11 = rotationMatrix.m11 * scale.y;
		dest.m21 = rotationMatrix.m21 * scale.y;
		dest.m31 = translation.y;

		dest.m02 = rotationMatrix.m02 * scale.z;
		dest.m12 = rotationMatrix.m12 * scale.z;
		dest.m22 = rotationMatrix.m22 * scale.z;
		dest.m32 = translation.z;

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1;

		return dest;
	}

	/**
	 * Construit une matrice de transformation pour un mod�le 3D.
	 * 
	 * @param translation La translation � appliquer � l'objet.
	 * @param rotX        La rotation autour de l'axe X, en degr�s.
	 * @param rotY        La rotation autour de l'axe Y, en degr�s.
	 * @param rotZ        La rotation autour de l'axe Z, en degr�s.
	 * @param scale       Le facteur d'�chelle � appliquer � l'objet.
	 * @param dest        La matrice de destination ou null si une nouvelle matrice
	 *                    doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createTransformationMatrix(Vector3f translation, float rotX, float rotY, float rotZ,
			float scale, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}
		float cosX = (float) Math.cos(Math.toRadians(rotX));
		float sinX = (float) Math.sin(Math.toRadians(rotX));
		float cosY = (float) Math.cos(Math.toRadians(rotY));
		float sinY = (float) Math.sin(Math.toRadians(rotY));
		float cosZ = (float) Math.cos(Math.toRadians(rotZ));
		float sinZ = (float) Math.sin(Math.toRadians(rotZ));

		dest.m00 = scale * cosZ * cosY;
		dest.m10 = scale * -sinZ * cosY;
		dest.m20 = scale * sinY;
		dest.m30 = translation.x;

		dest.m01 = scale * (cosZ * sinY * sinX + sinZ * cosX);
		dest.m11 = scale * (-sinZ * sinY * sinX + cosZ * cosX);
		dest.m21 = scale * -cosY * sinX;
		dest.m31 = translation.y;

		dest.m02 = scale * (-cosZ * sinY * cosX + sinZ * sinX);
		dest.m12 = scale * (sinZ * sinY * cosX + cosZ * sinX);
		dest.m22 = scale * cosY * cosX;
		dest.m32 = translation.z;

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1;

		return dest;
	}

	/**
	 * Construit une matrice de transformation pour un mod�le 3D.
	 * 
	 * @param translation La translation � appliquer � l'objet.
	 * @param rotX        La rotation autour de l'axe X, en degr�s.
	 * @param rotY        La rotation autour de l'axe Y, en degr�s.
	 * @param rotZ        La rotation autour de l'axe Z, en degr�s.
	 * @param scale       Le facteur d'�chelle � appliquer � l'objet selon chaque
	 *                    axe.
	 * @param dest        La matrice de destination ou null si une nouvelle matrice
	 *                    doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createTransformationMatrix(Vector3f translation, float rotX, float rotY, float rotZ,
			Vector3f scale, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}
		float cosX = (float) Math.cos(Math.toRadians(rotX));
		float sinX = (float) Math.sin(Math.toRadians(rotX));
		float cosY = (float) Math.cos(Math.toRadians(rotY));
		float sinY = (float) Math.sin(Math.toRadians(rotY));
		float cosZ = (float) Math.cos(Math.toRadians(rotZ));
		float sinZ = (float) Math.sin(Math.toRadians(rotZ));

		dest.m00 = scale.x * cosZ * cosY;
		dest.m10 = scale.x * -sinZ * cosY;
		dest.m20 = scale.x * sinY;
		dest.m30 = translation.x;

		dest.m01 = scale.y * (cosZ * sinY * sinX + sinZ * cosX);
		dest.m11 = scale.y * (-sinZ * sinY * sinX + cosZ * cosX);
		dest.m21 = scale.y * -cosY * sinX;
		dest.m31 = translation.y;

		dest.m02 = scale.z * (-cosZ * sinY * cosX + sinZ * sinX);
		dest.m12 = scale.z * (sinZ * sinY * cosX + cosZ * sinX);
		dest.m22 = scale.z * cosY * cosX;
		dest.m32 = translation.z;

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1;

		return dest;
	}

	/**
	 * Construit une matrice de transformation pour une AABB.
	 * 
	 * @param translation Le vecteur de translation � appliquer � la bo�te.
	 * @param scale       Le facteur d'�chelle � appliquer sur chaque axe de la
	 *                    bo�te.
	 * @param dest        La matrice de destination ou null si une nouvelle matrice
	 *                    doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createAABBTransformationMatrix(Vector3f translation, Vector3f scale, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		dest.m00 = scale.x;
		dest.m30 = translation.x;

		dest.m11 = scale.y;
		dest.m31 = translation.y;

		dest.m22 = scale.z;
		dest.m32 = translation.z;

		return dest;
	}

	/**
	 * Construit une matrice de transformation pour un rectangle en 2D comme un
	 * bouton affichable � l'�cran.
	 * 
	 * @param translation La translation en NDC.
	 * @param scale       Le facteur d'�chelle � appliquer sur les axes X et Y.
	 * @param mirrorX     True pour inverser verticalement la position des sommets
	 *                    du rectangle par rapport � son centre.
	 * @param mirrorY     True pour inverser horizontalement la position des sommets
	 *                    du rectangle par rapport � son centre.
	 * @param depth       La profondeur du rectangle, utile pour masquer les
	 *                    rectangles dsssin�s � une profondeur sup�rieure.
	 * @param dest        La matrice de destination ou null si une nouvelle matrice
	 *                    doit �tre cr��e.
	 * @return la matrice de transformation.
	 */
	public static Matrix4f createGuiTransformationMatrix(Vector2f translation, Vector2f scale, boolean mirrorX,
			boolean mirrorY, float depth, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		dest.m00 = mirrorX ? -scale.x : scale.x;
		dest.m30 = translation.x;

		dest.m11 = mirrorY ? -scale.y : scale.y;
		dest.m31 = translation.y;

		dest.m22 = depth;

		return dest;
	}

	/**
	 * Construit une matrice de rotation pour un mod�le 3D. Permet d'orienter un
	 * objet suivant une direction pr�cise.
	 * 
	 * @param direction      Un vecteur unitaire repr�sentant l'orientation initiale
	 *                       de l'objet, Vec3(0, 1, 0) g�n�ralement.
	 * @param finalDirection Un vecteur unitaire repr�sentant l'orientation finale
	 *                       de l'objet.
	 * @param dest           La matrice de destination ou null si une nouvelle
	 *                       matrice doit �tre cr��e.
	 * @return la matrice de rotation.
	 */
	public static Matrix4f createRotationMatrix(Vector3f direction, Vector3f finalDirection, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		Vector3f normal = Vector3f.cross(direction, finalDirection);
		float length = normal.length();
		float angle = 0;
		if (length == 0.0f) {
			if (Vector3f.dot(direction, finalDirection) < 0.0f) {
				dest.m00 = -1.0f;
				dest.m11 = -1.0f;
				dest.m22 = -1.0f;
			}
		} else {
			angle = (float) Math.acos(Clamp.symetricClamp(Vector3f.dot(direction, finalDirection)));
			normal.scale(1.0f / length);
			Matrix4f.rotate(angle, normal, dest, dest);
		}

		return dest;

	}

	/**
	 * Construit une matrice de rotation pour un mod�le 3D. Permet d'orienter un
	 * objet suivant une direction pr�cise.
	 * 
	 * @param direction      Un vecteur unitaire repr�sentant l'orientation initiale
	 *                       de l'objet, Vec3(0, 1, 0) g�n�ralement.
	 * @param finalDirection Un vecteur unitaire repr�sentant l'orientation finale
	 *                       de l'objet.
	 * @param dest           La matrice de destination ou null si une nouvelle
	 *                       matrice doit �tre cr��e.
	 * @return la matrice de rotation.
	 */
	public static Matrix3f createRotationMatrix(Vector3f direction, Vector3f finalDirection, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		} else {
			dest.setIdentity();
		}

		Vector3f normal = Vector3f.cross(direction, finalDirection);
		float length = normal.length();
		float angle = 0;
		if (length == 0.0f) {
			if (Vector3f.dot(direction, finalDirection) < 0.0f) {
				dest.m00 = -1.0f;
				dest.m11 = -1.0f;
				dest.m22 = -1.0f;
			}
		} else {
			angle = (float) Math.acos(Clamp.symetricClamp(Vector3f.dot(direction, finalDirection)));
			normal.scale(1.0f / length);
			MatrixOps.createRotationMatrix3f(angle, normal, dest);
		}

		return dest;
	}

	/**
	 * Construit une matrice de rotation � partir des 3 angles d'Euler.
	 * 
	 * @param rotX La rotation autour de l'axe X, en degr�s.
	 * @param rotY La rotation autour de l'axe Y, en degr�s.
	 * @param rotZ La rotation autour de l'axe Z, en degr�s.
	 * @param dest La matrice de destination ou null si une nouvelle matrice doit
	 *             �tre cr��e.
	 * @return la matrice de rotation.
	 */
	public static Matrix4f createRotationMatrix(float rotX, float rotY, float rotZ, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		float cosX = (float) Math.cos(Math.toRadians(rotX));
		float sinX = (float) Math.sin(Math.toRadians(rotX));
		float cosY = (float) Math.cos(Math.toRadians(rotY));
		float sinY = (float) Math.sin(Math.toRadians(rotY));
		float cosZ = (float) Math.cos(Math.toRadians(rotZ));
		float sinZ = (float) Math.sin(Math.toRadians(rotZ));

		dest.m00 = cosZ * cosY;
		dest.m10 = -sinZ * cosY;
		dest.m20 = sinY;

		dest.m01 = cosZ * sinY * sinX + sinZ * cosX;
		dest.m11 = -sinZ * sinY * sinX + cosZ * cosX;
		dest.m21 = -cosY * sinX;

		dest.m02 = -cosZ * sinY * cosX + sinZ * sinX;
		dest.m12 = sinZ * sinY * cosX + cosZ * sinX;
		dest.m22 = cosY * cosX;

		return dest;

	}

	/**
	 * Construit une matrice de rotation � partir des 3 angles d'Euler.
	 * 
	 * @param rotX La rotation autour de l'axe X, en degr�s.
	 * @param rotY La rotation autour de l'axe Y, en degr�s.
	 * @param rotZ La rotation autour de l'axe Z, en degr�s.
	 * @param dest La matrice de destination ou null si une nouvelle matrice doit
	 *             �tre cr��e.
	 * @return la matrice de rotation.
	 */
	public static Matrix3f createRotationMatrix3f(float rotX, float rotY, float rotZ, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		float cosX = (float) Math.cos(Math.toRadians(rotX));
		float sinX = (float) Math.sin(Math.toRadians(rotX));
		float cosY = (float) Math.cos(Math.toRadians(rotY));
		float sinY = (float) Math.sin(Math.toRadians(rotY));
		float cosZ = (float) Math.cos(Math.toRadians(rotZ));
		float sinZ = (float) Math.sin(Math.toRadians(rotZ));

		dest.m00 = cosZ * cosY;
		dest.m10 = -sinZ * cosY;
		dest.m20 = sinY;

		dest.m01 = cosZ * sinY * sinX + sinZ * cosX;
		dest.m11 = -sinZ * sinY * sinX + cosZ * cosX;
		dest.m21 = -cosY * sinX;

		dest.m02 = -cosZ * sinY * cosX + sinZ * sinX;
		dest.m12 = sinZ * sinY * cosX + cosZ * sinX;
		dest.m22 = cosY * cosX;

		return dest;

	}

	/**
	 * Construit une matrice de rotation � partir d'unaxe et d'un angle.
	 * 
	 * @param angle La rotation autour de l'axe, en radians.
	 * @param axis  L'axe de rotation, unitaire.
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return la matrice de rotation.
	 */
	public static Matrix3f createRotationMatrix3f(float angle, Vector3f axis, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		float c = (float) Math.cos(angle);
		float s = (float) Math.sin(angle);

		float one_minus_c = 1.0f - c;

		float xy = axis.x * axis.y * one_minus_c;
		float xz = axis.x * axis.z * one_minus_c;
		float yz = axis.y * axis.z * one_minus_c;

		float xs = axis.x * s;
		float ys = axis.y * s;
		float zs = axis.z * s;

		dest.m00 = axis.x * axis.x * one_minus_c + c;
		dest.m01 = xy + zs;
		dest.m02 = xz - ys;

		dest.m10 = xy - zs;
		dest.m11 = axis.y * axis.y * one_minus_c + c;
		dest.m12 = yz + xs;

		dest.m20 = xz + ys;
		dest.m21 = yz - xs;
		dest.m22 = axis.z * axis.z * one_minus_c + c;

		return dest;

	}

	public static Matrix4f rotateAbout(Vector3f origin, Vector3f offset, Matrix4f rotationMatrix, Vector3f scale,
			Matrix4f dest) {
		Matrix4f matrix = new Matrix4f(rotationMatrix);

		matrix.m30 = origin.x;
		matrix.m31 = origin.y;
		matrix.m32 = origin.z;

		matrix.translate(new Vector3f(offset.x, offset.y, offset.z));

		matrix.scale(scale);

		return matrix;
	}

	public static Matrix4f rotateAbout(Vector3f origin, Vector3f offset, float rx, float ry, float rz, Vector3f scale) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();

		float cosX = (float) Math.cos(Math.toRadians(rx));
		float sinX = (float) Math.sin(Math.toRadians(rx));
		float cosY = (float) Math.cos(Math.toRadians(ry));
		float sinY = (float) Math.sin(Math.toRadians(ry));
		float cosZ = (float) Math.cos(Math.toRadians(rz));
		float sinZ = (float) Math.sin(Math.toRadians(rz));

		matrix.m00 = cosZ * cosY;
		matrix.m10 = -sinZ * cosY;
		matrix.m20 = sinY;
		matrix.m30 = origin.x;

		matrix.m01 = cosZ * sinY * sinX + sinZ * cosX;
		matrix.m11 = -sinZ * sinY * sinX + cosZ * cosX;
		matrix.m21 = -cosY * sinX;
		matrix.m31 = origin.y;

		matrix.m02 = -cosZ * sinY * cosX + sinZ * sinX;
		matrix.m12 = sinZ * sinY * cosX + cosZ * sinX;
		matrix.m22 = cosY * cosX;
		matrix.m32 = origin.z;

		matrix.translate(new Vector3f(offset.x, offset.y, offset.z));

		matrix.scale(scale);

		return matrix;
	}

	/**
	 * Construit une matrice de vue pour une cam�ra.
	 * 
	 * @param position La position de l'observateur.
	 * @param pitch    L'angle de vue par rapport � l'axe X en degr�s.
	 * @param yaw      L'angle de vue par rapport � l'axe Y en degr�s.
	 * @param roll     L'angle de vue par rapport � l'axe Z en degr�s.
	 * @param dest     La matrice de destination ou null si une nouvelle matrice
	 *                 doit �tre cr��e.
	 * @return la matrice de vue.
	 */
	public static Matrix4f createViewMatrix(Vector3f position, float pitch, float yaw, float roll, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		float cosX = (float) Math.cos(Math.toRadians(pitch));
		float sinX = (float) Math.sin(Math.toRadians(pitch));
		float cosY = (float) Math.cos(Math.toRadians(yaw));
		float sinY = (float) Math.sin(Math.toRadians(yaw));
		float cosZ = (float) Math.cos(Math.toRadians(roll));
		float sinZ = (float) Math.sin(Math.toRadians(roll));

		dest.m00 = cosZ * cosY;
		dest.m10 = -sinZ * cosY;
		dest.m20 = sinY;

		dest.m01 = cosZ * sinY * sinX + sinZ * cosX;
		dest.m11 = -sinZ * sinY * sinX + cosZ * cosX;
		dest.m21 = -cosY * sinX;

		dest.m02 = -cosZ * sinY * cosX + sinZ * sinX;
		dest.m12 = sinZ * sinY * cosX + cosZ * sinX;
		dest.m22 = cosY * cosX;

		Vector3f translation = position;
		Vector3f negativeCameraPos = new Vector3f(-translation.x, -translation.y, -translation.z);
		dest.translate(negativeCameraPos);

		return dest;
	}

	/**
	 * Construit une matrice de vue pour une cam�ra.
	 * 
	 * @param position La position de l'observateur.
	 * @param forward  Le vecteur unitaire pointant dans la direction du regard.
	 * @param up       Le vecteur unitaire pointant vers le haut, perpendiculaire �
	 *                 la direction du regard.
	 * @param dest     La matrice de destination ou null si une nouvelle matrice
	 *                 doit �tre cr��e.
	 * @return la matrice de vue.
	 */
	public static Matrix4f createViewMatrix(Vector3f position, Vector3f forward, Vector3f up, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		Vector3f X, Y, Z;

		Z = Vector3f.negate(forward);
		Y = up;
		X = Vector3f.cross(Y, Z);

		dest.m00 = X.x;
		dest.m10 = X.y;
		dest.m20 = X.z;
		dest.m30 = -Vector3f.dot(X, position);

		dest.m01 = Y.x;
		dest.m11 = Y.y;
		dest.m21 = Y.z;
		dest.m31 = -Vector3f.dot(Y, position);

		dest.m02 = Z.x;
		dest.m12 = Z.y;
		dest.m22 = Z.z;
		dest.m32 = -Vector3f.dot(Z, position);

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1.0f;

		return dest;
	}

	/**
	 * Creates a perspective projection matrix.
	 * 
	 * @param width  The width of the screen, in pixels.
	 * @param height The height of the screen, in pixels.
	 * @param fov    The field of view angle, in degrees.
	 * @param near   The near plane distance.
	 * @param far    The far plane distance.
	 * @param dest   The destination matrix or null if a new matrix is to be
	 *               created.
	 * @return a new perspective projection matrix.
	 */
	public static Matrix4f createPerspectiveProjectionMatrix(float width, float height, float fov, float near,
			float far, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		float aspectRatio = width / height;
		float x_scale = (float) (1f / Math.tan(Math.toRadians(fov / 2f)));
		float y_scale = x_scale * aspectRatio;

		float frustum_length = far - near;

		dest.m00 = x_scale;
		dest.m11 = y_scale;
		dest.m22 = -((far + near) / frustum_length);
		dest.m23 = -1f;
		dest.m32 = -((2 * near * far) / frustum_length);
		dest.m33 = 0;

		return dest;
	}

	/**
	 * Creates a new orthographic projection matrix. The 3 axis are the same as in
	 * opengl: The x axis is left to right. The y axis is bottom to top. The z axis
	 * is near to far.
	 * 
	 * @param boxMinCorner The corner of the cube which has the minimum coordinates.
	 * @param boxMaxCorner The corner of the cube which has the maximum coordinates.
	 * @param dest         The destination matrix or null if a new matrix is to be
	 *                     created.
	 * @return a new orthographic projection matrix.
	 */
	public static Matrix4f createOrthographicProjectionMatrix(Vector3f boxMinCorner, Vector3f boxMaxCorner,
			Matrix4f dest) {

		Vector3f center = Vector3f.add(boxMinCorner, boxMaxCorner);
		center.x /= 2.0f;
		center.y /= 2.0f;
		center.z /= 2.0f;

		return createOrthographicProjectionMatrix(boxMaxCorner.x - boxMinCorner.x, boxMaxCorner.y - boxMinCorner.y,
				boxMaxCorner.z - boxMinCorner.z, center, dest);
	}

	/**
	 * Creates a new orthographic projection matrix.
	 * 
	 * @param width     The width of the cube, along the x axis.
	 * @param height    The height of the cube, along the y axis.
	 * @param depth     The depth of the cube, along the z axis.
	 * @param boxCenter The center of the cube. Vec3(0, 0, -(near+far)/2) most of
	 *                  the time.
	 * @param dest      The destination matrix or null if a new matrix is to be
	 *                  created.
	 * @return a new orthographic projection matrix.
	 */
	public static Matrix4f createOrthographicProjectionMatrix(float width, float height, float depth,
			Vector3f boxCenter, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		} else {
			dest.setIdentity();
		}

		dest.m00 = 2.0f / width;
		dest.m11 = 2.0f / height;
		dest.m22 = -2.0f / depth;

		dest.m30 = -boxCenter.x * dest.m00;
		dest.m31 = -boxCenter.y * dest.m11;
		dest.m32 = -boxCenter.z * dest.m22;

		return dest;
	}

	/**
	 * Construit une nouvelle matrice de transformation pour un billboard. Un
	 * billboard est un objet plan qui fait toujours face � la cam�ra.
	 * 
	 * @param viewMatrix La matrice de vue utilis�e lors du rendu.
	 * @param position   La position du billboard dans le monde 3D.
	 * @param scale      Le facteur d'�chelle � appliquer sur le mod�le du
	 *                   billboard.
	 * @param dest       La matrice de destination ou null si une nouvelle matrice
	 *                   doit �tre cr��e.
	 * @return La nouvelle matrice de transformation.
	 */
	public static Matrix4f createBillboardTransformationMatrix(Matrix4f viewMatrix, Vector3f position, float scale,
			Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		dest.m00 = viewMatrix.m00 * scale;
		dest.m10 = viewMatrix.m01 * scale;
		dest.m20 = viewMatrix.m02 * scale;

		dest.m01 = viewMatrix.m10 * scale;
		dest.m11 = viewMatrix.m11 * scale;
		dest.m21 = viewMatrix.m12 * scale;

		dest.m02 = viewMatrix.m20 * scale;
		dest.m12 = viewMatrix.m21 * scale;
		dest.m22 = viewMatrix.m22 * scale;

		dest.m30 = position.x;
		dest.m31 = position.y;
		dest.m32 = position.z;

		dest.m03 = 0;
		dest.m13 = 0;
		dest.m23 = 0;
		dest.m33 = 1;

		return dest;
	}

	/**
	 * Applique un changement de base. <br>
	 * Plus exactement: <br>
	 * dest = rotation * mat * transpose(rotation) <br>
	 * 
	 * @param mat      La matrice pour laquelle on applique le changement de base.
	 * @param rotation Une matrice de rotation.
	 * @param dest     La matrice de destination ou null si une nouvelle matrice
	 *                 doit �tre cr��e.
	 * 
	 * @return La matrice dans la nouvelle base.
	 */
	public static Matrix3f changeOfBasis(Matrix3f mat, Matrix3f rotation, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		// mXX = rotation * mat
		float m00 = rotation.m00 * mat.m00 + rotation.m10 * mat.m01 + rotation.m20 * mat.m02;
		float m01 = rotation.m01 * mat.m00 + rotation.m11 * mat.m01 + rotation.m21 * mat.m02;
		float m02 = rotation.m02 * mat.m00 + rotation.m12 * mat.m01 + rotation.m22 * mat.m02;
		float m10 = rotation.m00 * mat.m10 + rotation.m10 * mat.m11 + rotation.m20 * mat.m12;
		float m11 = rotation.m01 * mat.m10 + rotation.m11 * mat.m11 + rotation.m21 * mat.m12;
		float m12 = rotation.m02 * mat.m10 + rotation.m12 * mat.m11 + rotation.m22 * mat.m12;
		float m20 = rotation.m00 * mat.m20 + rotation.m10 * mat.m21 + rotation.m20 * mat.m22;
		float m21 = rotation.m01 * mat.m20 + rotation.m11 * mat.m21 + rotation.m21 * mat.m22;
		float m22 = rotation.m02 * mat.m20 + rotation.m12 * mat.m21 + rotation.m22 * mat.m22;

		// destXX = mXX * transpose(rotation)
		dest.m00 = m00 * rotation.m00 + m10 * rotation.m10 + m20 * rotation.m20;
		dest.m01 = m01 * rotation.m00 + m11 * rotation.m10 + m21 * rotation.m20;
		dest.m02 = m02 * rotation.m00 + m12 * rotation.m10 + m22 * rotation.m20;
		dest.m10 = m00 * rotation.m01 + m10 * rotation.m11 + m20 * rotation.m21;
		dest.m11 = m01 * rotation.m01 + m11 * rotation.m11 + m21 * rotation.m21;
		dest.m12 = m02 * rotation.m01 + m12 * rotation.m11 + m22 * rotation.m21;
		dest.m20 = m00 * rotation.m02 + m10 * rotation.m12 + m20 * rotation.m22;
		dest.m21 = m01 * rotation.m02 + m11 * rotation.m12 + m21 * rotation.m22;
		dest.m22 = m02 * rotation.m02 + m12 * rotation.m12 + m22 * rotation.m22;

		return dest;
	}

	/**
	 * Applique un changement de base. <br>
	 * Plus exactement: <br>
	 * dest = rotation * diag(diag0, diag1, diag2) * transpose(rotation) <br>
	 * 
	 * @param diag0    le premier terme sur la diagonale
	 * @param diag1    le second terme sur la diagonale
	 * @param diag2    le troisi�me terme sur la diagonale
	 * 
	 * @param rotation Une matrice de rotation.
	 * @param dest     La matrice de destination ou null si une nouvelle matrice
	 *                 doit �tre cr��e.
	 * 
	 * @return La matrice dans la nouvelle base.
	 */
	public static Matrix3f changeOfBasis(float diag0, float diag1, float diag2, Matrix3f rotation, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		// mXX = rotation * diag(diag0, diag1, diag2)
		float m00 = rotation.m00 * diag0;
		float m01 = rotation.m01 * diag0;
		float m02 = rotation.m02 * diag0;
		float m10 = rotation.m10 * diag1;
		float m11 = rotation.m11 * diag1;
		float m12 = rotation.m12 * diag1;
		float m20 = rotation.m20 * diag2;
		float m21 = rotation.m21 * diag2;
		float m22 = rotation.m22 * diag2;

		// destXX = mXX * transpose(rotation)
		dest.m00 = m00 * rotation.m00 + m10 * rotation.m10 + m20 * rotation.m20;
		dest.m01 = m01 * rotation.m00 + m11 * rotation.m10 + m21 * rotation.m20;
		dest.m02 = m02 * rotation.m00 + m12 * rotation.m10 + m22 * rotation.m20;
		dest.m10 = m00 * rotation.m01 + m10 * rotation.m11 + m20 * rotation.m21;
		dest.m11 = m01 * rotation.m01 + m11 * rotation.m11 + m21 * rotation.m21;
		dest.m12 = m02 * rotation.m01 + m12 * rotation.m11 + m22 * rotation.m21;
		dest.m20 = m00 * rotation.m02 + m10 * rotation.m12 + m20 * rotation.m22;
		dest.m21 = m01 * rotation.m02 + m11 * rotation.m12 + m21 * rotation.m22;
		dest.m22 = m02 * rotation.m02 + m12 * rotation.m12 + m22 * rotation.m22;

		return dest;
	}

	/**
	 * Applique un changement de base. <br>
	 * Plus exactement: <br>
	 * Si invert vaut false: dest = rotation * mat * transpose(rotation) <br>
	 * Si invert vaut true: dest = transpose(rotation) * mat * rotation
	 * 
	 * @param mat       La matrice pour laquelle on applique le changement de base.
	 * @param transform La matrice de transformation. Le changement de base est
	 *                  effectu� avec la sous-matrice 3x3 de transform.
	 * @param invert    Si true: consid�re mat comme l'inverse d'une matrice.
	 * @param dest      La matrice de destination ou null si une nouvelle matrice
	 *                  doit �tre cr��e.
	 * 
	 * @return La matrice dans la nouvelle base.
	 */
	public static Matrix3f changeOfBasis(Matrix3f mat, Matrix4f transform, boolean invert, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		if (!invert) {
			// mXX = transform[3x3] * mat
			float m00 = transform.m00 * mat.m00 + transform.m10 * mat.m01 + transform.m20 * mat.m02;
			float m01 = transform.m01 * mat.m00 + transform.m11 * mat.m01 + transform.m21 * mat.m02;
			float m02 = transform.m02 * mat.m00 + transform.m12 * mat.m01 + transform.m22 * mat.m02;
			float m10 = transform.m00 * mat.m10 + transform.m10 * mat.m11 + transform.m20 * mat.m12;
			float m11 = transform.m01 * mat.m10 + transform.m11 * mat.m11 + transform.m21 * mat.m12;
			float m12 = transform.m02 * mat.m10 + transform.m12 * mat.m11 + transform.m22 * mat.m12;
			float m20 = transform.m00 * mat.m20 + transform.m10 * mat.m21 + transform.m20 * mat.m22;
			float m21 = transform.m01 * mat.m20 + transform.m11 * mat.m21 + transform.m21 * mat.m22;
			float m22 = transform.m02 * mat.m20 + transform.m12 * mat.m21 + transform.m22 * mat.m22;

			// destXX = mXX * transpose(transform[3x3])
			dest.m00 = m00 * transform.m00 + m10 * transform.m10 + m20 * transform.m20;
			dest.m01 = m01 * transform.m00 + m11 * transform.m10 + m21 * transform.m20;
			dest.m02 = m02 * transform.m00 + m12 * transform.m10 + m22 * transform.m20;
			dest.m10 = m00 * transform.m01 + m10 * transform.m11 + m20 * transform.m21;
			dest.m11 = m01 * transform.m01 + m11 * transform.m11 + m21 * transform.m21;
			dest.m12 = m02 * transform.m01 + m12 * transform.m11 + m22 * transform.m21;
			dest.m20 = m00 * transform.m02 + m10 * transform.m12 + m20 * transform.m22;
			dest.m21 = m01 * transform.m02 + m11 * transform.m12 + m21 * transform.m22;
			dest.m22 = m02 * transform.m02 + m12 * transform.m12 + m22 * transform.m22;
		} else {
			// mXX = transpose(transform[3x3]) * mat
			float m00 = transform.m00 * mat.m00 + transform.m01 * mat.m01 + transform.m02 * mat.m02;
			float m01 = transform.m10 * mat.m00 + transform.m11 * mat.m01 + transform.m12 * mat.m02;
			float m02 = transform.m20 * mat.m00 + transform.m21 * mat.m01 + transform.m22 * mat.m02;
			float m10 = transform.m00 * mat.m10 + transform.m01 * mat.m11 + transform.m02 * mat.m12;
			float m11 = transform.m10 * mat.m10 + transform.m11 * mat.m11 + transform.m12 * mat.m12;
			float m12 = transform.m20 * mat.m10 + transform.m21 * mat.m11 + transform.m22 * mat.m12;
			float m20 = transform.m00 * mat.m20 + transform.m01 * mat.m21 + transform.m02 * mat.m22;
			float m21 = transform.m10 * mat.m20 + transform.m11 * mat.m21 + transform.m12 * mat.m22;
			float m22 = transform.m20 * mat.m20 + transform.m21 * mat.m21 + transform.m22 * mat.m22;

			// destXX = mXX * transform[3x3]
			dest.m00 = m00 * transform.m00 + m10 * transform.m01 + m20 * transform.m02;
			dest.m01 = m01 * transform.m00 + m11 * transform.m01 + m21 * transform.m02;
			dest.m02 = m02 * transform.m00 + m12 * transform.m01 + m22 * transform.m02;
			dest.m10 = m00 * transform.m10 + m10 * transform.m11 + m20 * transform.m12;
			dest.m11 = m01 * transform.m10 + m11 * transform.m11 + m21 * transform.m12;
			dest.m12 = m02 * transform.m10 + m12 * transform.m11 + m22 * transform.m12;
			dest.m20 = m00 * transform.m20 + m10 * transform.m21 + m20 * transform.m22;
			dest.m21 = m01 * transform.m20 + m11 * transform.m21 + m21 * transform.m22;
			dest.m22 = m02 * transform.m20 + m12 * transform.m21 + m22 * transform.m22;
		}

		return dest;
	}

	/**
	 * Multiplie src par scalar et range le r�sultat dans dest.
	 * 
	 * @param src    La matrice � multiplier par un scalaire.
	 * @param scalar Le scalaire.
	 * @param dest   La matrice de destination ou null si une nouvelle matrice doit
	 *               �tre cr��e.
	 * 
	 * @return dest
	 */
	public static Matrix3f scalarMult(Matrix3f src, float scalar, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		dest.m00 = src.m00 * scalar;
		dest.m01 = src.m01 * scalar;
		dest.m02 = src.m02 * scalar;

		dest.m10 = src.m10 * scalar;
		dest.m11 = src.m11 * scalar;
		dest.m12 = src.m12 * scalar;

		dest.m20 = src.m20 * scalar;
		dest.m21 = src.m21 * scalar;
		dest.m22 = src.m22 * scalar;

		return dest;
	}

	/**
	 * Multiplie src par scalar et range le r�sultat dans dest. Seule la
	 * sous-matrice 3x3 (coin haut gauche) de src est conern�e.
	 * 
	 * @param src    La matrice � multiplier par un scalaire.
	 * @param scalar Le scalaire.
	 * @param dest   La matrice de destination ou null si une nouvelle matrice doit
	 *               �tre cr��e.
	 * 
	 * @return dest
	 */
	public static Matrix4f scalarMult(Matrix4f src, float scalar, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		dest.m00 = src.m00 * scalar;
		dest.m01 = src.m01 * scalar;
		dest.m02 = src.m02 * scalar;

		dest.m10 = src.m10 * scalar;
		dest.m11 = src.m11 * scalar;
		dest.m12 = src.m12 * scalar;

		dest.m20 = src.m20 * scalar;
		dest.m21 = src.m21 * scalar;
		dest.m22 = src.m22 * scalar;

		dest.m03 = src.m03;
		dest.m13 = src.m13;
		dest.m23 = src.m23;
		dest.m33 = src.m33;

		dest.m30 = src.m30;
		dest.m31 = src.m31;
		dest.m32 = src.m32;

		return dest;
	}

	/**
	 * Short-hand for dest = (matrix * vec4(v, 0.0)).xyz;
	 * 
	 * @param matrix
	 * @param v
	 * @param dest
	 */
	public static void vectorMult(Matrix4f matrix, Vector3f v, Vector3f dest) {
		float x = matrix.m00 * v.x + matrix.m10 * v.y + matrix.m20 * v.z;
		float y = matrix.m01 * v.x + matrix.m11 * v.y + matrix.m21 * v.z;
		float z = matrix.m02 * v.x + matrix.m12 * v.y + matrix.m22 * v.z;

		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	/**
	 * Short-hand for dest = (matrix * vec4(v, 1.0)).xyz;
	 * 
	 * @param matrix
	 * @param v
	 * @param dest
	 */
	public static void vertexMult(Matrix4f matrix, Vector3f v, Vector3f dest) {
		float x = matrix.m00 * v.x + matrix.m10 * v.y + matrix.m20 * v.z + matrix.m30;
		float y = matrix.m01 * v.x + matrix.m11 * v.y + matrix.m21 * v.z + matrix.m31;
		float z = matrix.m02 * v.x + matrix.m12 * v.y + matrix.m22 * v.z + matrix.m32;

		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	/**
	 * Loads the 3x3 upper left block of src into dest.
	 * 
	 * @param src       The src matrix.
	 * @param dest      The dest matrix or null if a new matrix should be created.
	 * @param transpose true if the transpose of src should be loaded into dest.
	 * @return dest The destination matrix.
	 */
	public static Matrix3f loadMatrix(Matrix4f src, Matrix3f dest, boolean transpose) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		if (!transpose) {
			dest.m00 = src.m00;
			dest.m01 = src.m01;
			dest.m02 = src.m02;

			dest.m10 = src.m10;
			dest.m11 = src.m11;
			dest.m12 = src.m12;

			dest.m20 = src.m20;
			dest.m21 = src.m21;
			dest.m22 = src.m22;
		} else {
			dest.m00 = src.m00;
			dest.m01 = src.m10;
			dest.m02 = src.m20;

			dest.m10 = src.m01;
			dest.m11 = src.m11;
			dest.m12 = src.m21;

			dest.m20 = src.m02;
			dest.m21 = src.m12;
			dest.m22 = src.m22;
		}

		return dest;
	}

	/**
	 * Permet de charger src dans la sous-matrice 3x3 (coin haut gauche) de dest.
	 * Les autres composantes de dest ne sont pas modifi�es.
	 * 
	 * @param src       La matrice source.
	 * @param dest      La matrice de destination ou null si une nouvelle matrice
	 *                  doit �tre cr��e.
	 * @param transpose Indique s'il faut transposer dest.
	 * @return dest La matrice de destination.
	 */
	public static Matrix4f loadMatrix(Matrix3f src, Matrix4f dest, boolean transpose) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		if (!transpose) {
			dest.m00 = src.m00;
			dest.m01 = src.m01;
			dest.m02 = src.m02;

			dest.m10 = src.m10;
			dest.m11 = src.m11;
			dest.m12 = src.m12;

			dest.m20 = src.m20;
			dest.m21 = src.m21;
			dest.m22 = src.m22;
		} else {
			dest.m00 = src.m00;
			dest.m01 = src.m10;
			dest.m02 = src.m20;

			dest.m10 = src.m01;
			dest.m11 = src.m11;
			dest.m12 = src.m21;

			dest.m20 = src.m02;
			dest.m21 = src.m12;
			dest.m22 = src.m22;
		}

		return dest;
	}

	/**
	 * Calcule dest = left * right[3x3]
	 * 
	 * @param left
	 * @param right
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix3f matrixMult(Matrix3f left, Matrix4f right, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		float m00 = left.m00 * right.m00 + left.m10 * right.m01 + left.m20 * right.m02;
		float m01 = left.m01 * right.m00 + left.m11 * right.m01 + left.m21 * right.m02;
		float m02 = left.m02 * right.m00 + left.m12 * right.m01 + left.m22 * right.m02;
		float m10 = left.m00 * right.m10 + left.m10 * right.m11 + left.m20 * right.m12;
		float m11 = left.m01 * right.m10 + left.m11 * right.m11 + left.m21 * right.m12;
		float m12 = left.m02 * right.m10 + left.m12 * right.m11 + left.m22 * right.m12;
		float m20 = left.m00 * right.m20 + left.m10 * right.m21 + left.m20 * right.m22;
		float m21 = left.m01 * right.m20 + left.m11 * right.m21 + left.m21 * right.m22;
		float m22 = left.m02 * right.m20 + left.m12 * right.m21 + left.m22 * right.m22;

		dest.m00 = m00;
		dest.m01 = m01;
		dest.m02 = m02;

		dest.m10 = m10;
		dest.m11 = m11;
		dest.m12 = m12;

		dest.m20 = m20;
		dest.m21 = m21;
		dest.m22 = m22;

		return dest;
	}

	/**
	 * Calcule dest = left[3x3] * right
	 * 
	 * @param left
	 * @param right
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix3f matrixMult(Matrix4f left, Matrix3f right, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		float m00 = left.m00 * right.m00 + left.m10 * right.m01 + left.m20 * right.m02;
		float m01 = left.m01 * right.m00 + left.m11 * right.m01 + left.m21 * right.m02;
		float m02 = left.m02 * right.m00 + left.m12 * right.m01 + left.m22 * right.m02;
		float m10 = left.m00 * right.m10 + left.m10 * right.m11 + left.m20 * right.m12;
		float m11 = left.m01 * right.m10 + left.m11 * right.m11 + left.m21 * right.m12;
		float m12 = left.m02 * right.m10 + left.m12 * right.m11 + left.m22 * right.m12;
		float m20 = left.m00 * right.m20 + left.m10 * right.m21 + left.m20 * right.m22;
		float m21 = left.m01 * right.m20 + left.m11 * right.m21 + left.m21 * right.m22;
		float m22 = left.m02 * right.m20 + left.m12 * right.m21 + left.m22 * right.m22;

		dest.m00 = m00;
		dest.m01 = m01;
		dest.m02 = m02;

		dest.m10 = m10;
		dest.m11 = m11;
		dest.m12 = m12;

		dest.m20 = m20;
		dest.m21 = m21;
		dest.m22 = m22;

		return dest;
	}

	/**
	 * Calcule dest[3x3] = left * right[3x3]
	 * 
	 * @param left
	 * @param right
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix4f matrixMult(Matrix3f left, Matrix4f right, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		float m00 = left.m00 * right.m00 + left.m10 * right.m01 + left.m20 * right.m02;
		float m01 = left.m01 * right.m00 + left.m11 * right.m01 + left.m21 * right.m02;
		float m02 = left.m02 * right.m00 + left.m12 * right.m01 + left.m22 * right.m02;
		float m10 = left.m00 * right.m10 + left.m10 * right.m11 + left.m20 * right.m12;
		float m11 = left.m01 * right.m10 + left.m11 * right.m11 + left.m21 * right.m12;
		float m12 = left.m02 * right.m10 + left.m12 * right.m11 + left.m22 * right.m12;
		float m20 = left.m00 * right.m20 + left.m10 * right.m21 + left.m20 * right.m22;
		float m21 = left.m01 * right.m20 + left.m11 * right.m21 + left.m21 * right.m22;
		float m22 = left.m02 * right.m20 + left.m12 * right.m21 + left.m22 * right.m22;

		dest.m00 = m00;
		dest.m01 = m01;
		dest.m02 = m02;

		dest.m10 = m10;
		dest.m11 = m11;
		dest.m12 = m12;

		dest.m20 = m20;
		dest.m21 = m21;
		dest.m22 = m22;

		return dest;
	}

	/**
	 * Calcule dest[3x3] = left[3x3] * right
	 * 
	 * @param left
	 * @param right
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix4f matrixMult(Matrix4f left, Matrix3f right, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		float m00 = left.m00 * right.m00 + left.m10 * right.m01 + left.m20 * right.m02;
		float m01 = left.m01 * right.m00 + left.m11 * right.m01 + left.m21 * right.m02;
		float m02 = left.m02 * right.m00 + left.m12 * right.m01 + left.m22 * right.m02;
		float m10 = left.m00 * right.m10 + left.m10 * right.m11 + left.m20 * right.m12;
		float m11 = left.m01 * right.m10 + left.m11 * right.m11 + left.m21 * right.m12;
		float m12 = left.m02 * right.m10 + left.m12 * right.m11 + left.m22 * right.m12;
		float m20 = left.m00 * right.m20 + left.m10 * right.m21 + left.m20 * right.m22;
		float m21 = left.m01 * right.m20 + left.m11 * right.m21 + left.m21 * right.m22;
		float m22 = left.m02 * right.m20 + left.m12 * right.m21 + left.m22 * right.m22;

		dest.m00 = m00;
		dest.m01 = m01;
		dest.m02 = m02;

		dest.m10 = m10;
		dest.m11 = m11;
		dest.m12 = m12;

		dest.m20 = m20;
		dest.m21 = m21;
		dest.m22 = m22;

		return dest;
	}

	/**
	 * Calcule dest = left[3x3] * right[3x3]
	 * 
	 * @param left
	 * @param right
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix3f matrixMult(Matrix4f left, Matrix4f right, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		float m00 = left.m00 * right.m00 + left.m10 * right.m01 + left.m20 * right.m02;
		float m01 = left.m01 * right.m00 + left.m11 * right.m01 + left.m21 * right.m02;
		float m02 = left.m02 * right.m00 + left.m12 * right.m01 + left.m22 * right.m02;
		float m10 = left.m00 * right.m10 + left.m10 * right.m11 + left.m20 * right.m12;
		float m11 = left.m01 * right.m10 + left.m11 * right.m11 + left.m21 * right.m12;
		float m12 = left.m02 * right.m10 + left.m12 * right.m11 + left.m22 * right.m12;
		float m20 = left.m00 * right.m20 + left.m10 * right.m21 + left.m20 * right.m22;
		float m21 = left.m01 * right.m20 + left.m11 * right.m21 + left.m21 * right.m22;
		float m22 = left.m02 * right.m20 + left.m12 * right.m21 + left.m22 * right.m22;

		dest.m00 = m00;
		dest.m01 = m01;
		dest.m02 = m02;

		dest.m10 = m10;
		dest.m11 = m11;
		dest.m12 = m12;

		dest.m20 = m20;
		dest.m21 = m21;
		dest.m22 = m22;

		return dest;
	}

	/**
	 * Calcule dest[3x3] = left * right
	 * 
	 * @param left
	 * @param right
	 * @param dest  La matrice de destination ou null si une nouvelle matrice doit
	 *              �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix4f matrixMult(Matrix3f left, Matrix3f right, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		float m00 = left.m00 * right.m00 + left.m10 * right.m01 + left.m20 * right.m02;
		float m01 = left.m01 * right.m00 + left.m11 * right.m01 + left.m21 * right.m02;
		float m02 = left.m02 * right.m00 + left.m12 * right.m01 + left.m22 * right.m02;
		float m10 = left.m00 * right.m10 + left.m10 * right.m11 + left.m20 * right.m12;
		float m11 = left.m01 * right.m10 + left.m11 * right.m11 + left.m21 * right.m12;
		float m12 = left.m02 * right.m10 + left.m12 * right.m11 + left.m22 * right.m12;
		float m20 = left.m00 * right.m20 + left.m10 * right.m21 + left.m20 * right.m22;
		float m21 = left.m01 * right.m20 + left.m11 * right.m21 + left.m21 * right.m22;
		float m22 = left.m02 * right.m20 + left.m12 * right.m21 + left.m22 * right.m22;

		dest.m00 = m00;
		dest.m01 = m01;
		dest.m02 = m02;

		dest.m10 = m10;
		dest.m11 = m11;
		dest.m12 = m12;

		dest.m20 = m20;
		dest.m21 = m21;
		dest.m22 = m22;

		return dest;
	}

	/**
	 * Calcule:<br>
	 * dest = transpose(left) * right si transposeLeft vaut true <br>
	 * dest = left * transpose(right) si transposeLeft vaut false<br>
	 * 
	 * @param left
	 * @param right
	 * @param transposeLeft
	 * @param dest          La matrice de destination ou null si une nouvelle
	 *                      matrice doit �tre cr��e.
	 * @return La matrice de destination.
	 */
	public static Matrix3f matrixTransposeMult(Matrix3f left, Matrix3f right, boolean transposeLeft, Matrix3f dest) {
		if (dest == null) {
			dest = new Matrix3f();
		}

		if (transposeLeft) {
			float m00 = left.m00 * right.m00 + left.m01 * right.m01 + left.m02 * right.m02;
			float m01 = left.m10 * right.m00 + left.m11 * right.m01 + left.m12 * right.m02;
			float m02 = left.m20 * right.m00 + left.m21 * right.m01 + left.m22 * right.m02;
			float m10 = left.m00 * right.m10 + left.m01 * right.m11 + left.m02 * right.m12;
			float m11 = left.m10 * right.m10 + left.m11 * right.m11 + left.m12 * right.m12;
			float m12 = left.m20 * right.m10 + left.m21 * right.m11 + left.m22 * right.m12;
			float m20 = left.m00 * right.m20 + left.m01 * right.m21 + left.m02 * right.m22;
			float m21 = left.m10 * right.m20 + left.m11 * right.m21 + left.m12 * right.m22;
			float m22 = left.m20 * right.m20 + left.m21 * right.m21 + left.m22 * right.m22;

			dest.m00 = m00;
			dest.m01 = m01;
			dest.m02 = m02;

			dest.m10 = m10;
			dest.m11 = m11;
			dest.m12 = m12;

			dest.m20 = m20;
			dest.m21 = m21;
			dest.m22 = m22;
		} else {
			float m00 = left.m00 * right.m00 + left.m10 * right.m10 + left.m20 * right.m20;
			float m01 = left.m01 * right.m00 + left.m11 * right.m10 + left.m21 * right.m20;
			float m02 = left.m02 * right.m00 + left.m12 * right.m10 + left.m22 * right.m20;
			float m10 = left.m00 * right.m01 + left.m10 * right.m11 + left.m20 * right.m21;
			float m11 = left.m01 * right.m01 + left.m11 * right.m11 + left.m21 * right.m21;
			float m12 = left.m02 * right.m01 + left.m12 * right.m11 + left.m22 * right.m21;
			float m20 = left.m00 * right.m02 + left.m10 * right.m12 + left.m20 * right.m22;
			float m21 = left.m01 * right.m02 + left.m11 * right.m12 + left.m21 * right.m22;
			float m22 = left.m02 * right.m02 + left.m12 * right.m12 + left.m22 * right.m22;

			dest.m00 = m00;
			dest.m01 = m01;
			dest.m02 = m02;

			dest.m10 = m10;
			dest.m11 = m11;
			dest.m12 = m12;

			dest.m20 = m20;
			dest.m21 = m21;
			dest.m22 = m22;
		}

		return dest;
	}

	/**
	 * Calcule dest = transpose(rotation) * src;
	 * 
	 * @param rotation
	 * @param src
	 * @param dest
	 */
	public static void transformTranspose(Matrix3f rotation, Vector3f src, Vector3f dest) {
		if (dest == null)
			dest = new Vector3f();
		float x = rotation.m00 * src.x + rotation.m01 * src.y + rotation.m02 * src.z;
		float y = rotation.m10 * src.x + rotation.m11 * src.y + rotation.m12 * src.z;
		float z = rotation.m20 * src.x + rotation.m21 * src.y + rotation.m22 * src.z;
		dest.set(x, y, z);
	}

	/**
	 * Calcule dest = transpose(rotation) * src;
	 * 
	 * @param rotation
	 * @param src
	 * @param dest
	 */
	public static void transformTranspose(Matrix4f rotation, Vector3f src, Vector3f dest) {
		if (dest == null)
			dest = new Vector3f();
		float x = rotation.m00 * src.x + rotation.m01 * src.y + rotation.m02 * src.z;
		float y = rotation.m10 * src.x + rotation.m11 * src.y + rotation.m12 * src.z;
		float z = rotation.m20 * src.x + rotation.m21 * src.y + rotation.m22 * src.z;
		dest.set(x, y, z);
	}

	/**
	 * Computes v^T M v
	 * 
	 * @param mat
	 * @param v
	 * @return
	 */
	public static float sandwichDotProduct(Matrix3f mat, Vector3f v) {
		float x = v.x;
		float y = v.y;
		float z = v.z;

		return x * (mat.m00 * x + mat.m10 * y + mat.m20 * z) + y * (mat.m01 * x + mat.m11 * y + mat.m21 * z)
				+ z * (mat.m02 * x + mat.m12 * y + mat.m22 * z);
	}

	/**
	 * Computes left^T M right
	 * 
	 * @param left
	 * @param mat
	 * @param right
	 * @return
	 */
	public static float sandwichDotProduct(Vector3f left, Matrix3f mat, Vector3f right) {
		float x = right.x;
		float y = right.y;
		float z = right.z;

		return left.x * (mat.m00 * x + mat.m10 * y + mat.m20 * z) + left.y * (mat.m01 * x + mat.m11 * y + mat.m21 * z)
				+ left.z * (mat.m02 * x + mat.m12 * y + mat.m22 * z);
	}

	/**
	 * Computes -vx M vx with vx denoting the cross-product / skew-symetric matrix.
	 * 
	 * @param mat
	 * @param v
	 * @param dest
	 */
	public static void sandwichCrossProduct(Matrix3f mat, Vector3f v, Matrix3f dest) {
		float x = v.x;
		float y = v.y;
		float z = v.z;

		float m00 = -z * mat.m01 + y * mat.m02;
		float m10 = z * mat.m00 - x * mat.m02;
		float m20 = -y * mat.m00 + x * mat.m01;

		float m01 = -z * mat.m11 + y * mat.m12;
		float m11 = z * mat.m10 - x * mat.m12;
		float m21 = -y * mat.m10 + x * mat.m11;

		float m02 = -z * mat.m21 + y * mat.m22;
		float m12 = z * mat.m20 - x * mat.m22;
		float m22 = -y * mat.m20 + x * mat.m21;

		dest.m00 = z * m01 - y * m02;
		dest.m01 = -z * m00 + x * m02;
		dest.m02 = y * m00 - x * m01;

		dest.m10 = z * m11 - y * m12;
		dest.m11 = -z * m10 + x * m12;
		dest.m12 = y * m10 - x * m11;

		dest.m20 = z * m21 - y * m22;
		dest.m21 = -z * m20 + x * m22;
		dest.m22 = y * m20 - x * m21;
	}

	/**
	 * Builds a projection-view matrix for a directional light casting a shadow.
	 * This matrix maps points in world-space to light space, so that the frustum of
	 * the player is entierly contained within the shadow map.
	 * 
	 * @param lightDir       The direction of the light
	 * @param cameraPos      The position of the camera
	 * @param minToLightDist The minimum distance from the light to the camera, so
	 *                       that objects outside the camera view are still included
	 * @param minDepth       The minimum depth of the frustum of the shadow
	 * @param InvProjView    The perspective projection-view matrix of the player,
	 *                       inverted
	 * @param dest           The destination matrix or null
	 * @return dest or a new matrix
	 */
	public static Matrix4f createOrthographicShadowMatrix(Vector3f lightDir, Vector3f cameraPos, float minToLightDist,
			float minDepth, Matrix4f InvProjView, Matrix4f dest) {
		if (dest != null) {
			dest.setIdentity();
		} else {
			dest = new Matrix4f();
		}

		Vector3f min = new Vector3f(Float.POSITIVE_INFINITY);
		Vector3f max = new Vector3f(Float.NEGATIVE_INFINITY);
		Vector3f temp = new Vector3f();

		lightDir.negate();
		Matrix4f rotation = MatrixOps.computeOrthogonalComplement(lightDir, (Matrix4f) null);
		rotation.transpose();
		lightDir.negate();

		for (int i = 0; i < 8; i++) {// iterate over the corners of the NDC
			temp.x = (i & 1) != 0 ? -1.0f : 1.0f;
			temp.y = (i & 2) != 0 ? -1.0f : 1.0f;
			temp.z = (i & 4) != 0 ? -1.0f : 1.0f;

			float w = InvProjView.m03 * temp.x + InvProjView.m13 * temp.y + InvProjView.m23 * temp.z + InvProjView.m33;
			MatrixOps.vertexMult(InvProjView, temp, temp);// transform to world-space
			temp.scale(1.0f / w);

			MatrixOps.vectorMult(rotation, temp, temp);// transform to light-space
			VectorOps.min(min, temp, min);
			VectorOps.max(max, temp, max);
		}

		temp.set(cameraPos).translate(lightDir, -minToLightDist);
		MatrixOps.vectorMult(rotation, temp, temp);// transform to light-space
		VectorOps.min(min, temp, min);
		VectorOps.max(max, temp, max);
		
		//max.z = temp.z;

		temp.set(cameraPos).translate(lightDir, minDepth - minToLightDist);
		MatrixOps.vectorMult(rotation, temp, temp);// transform to light-space
		VectorOps.min(min, temp, min);
		VectorOps.max(max, temp, max);

		//min.z = Math.max(min.z, max.z - maxDepth);

		MatrixOps.createOrthographicProjectionMatrix(min, max, dest);
		Matrix4f.mul(dest, rotation, dest);
		return dest;
	}

	public static Matrix4f createOrthographicShadowMatrix(Vector3f lightDir, Vector3f cameraPos, float minToLightDist,
			float minDepth, Vector3f[] frustumCorners, Matrix4f dest) {
		if (dest != null) {
			dest.setIdentity();
		} else {
			dest = new Matrix4f();
		}

		Vector3f min = new Vector3f(Float.POSITIVE_INFINITY);
		Vector3f max = new Vector3f(Float.NEGATIVE_INFINITY);
		Vector3f temp = new Vector3f();

		lightDir.negate();
		Matrix4f rotation = MatrixOps.computeOrthogonalComplement(lightDir, (Matrix4f) null);
		rotation.transpose();
		lightDir.negate();

		for (int i = 0; i < 8; i++) {// iterate over the corners of the frustum
			MatrixOps.vectorMult(rotation, frustumCorners[i], temp);// transform to light-space
			VectorOps.min(min, temp, min);
			VectorOps.max(max, temp, max);
		}

		temp.set(cameraPos).translate(lightDir, -minToLightDist);
		MatrixOps.vectorMult(rotation, temp, temp);// transform to light-space
		VectorOps.min(min, temp, min);
		VectorOps.max(max, temp, max);
		
		//max.z = temp.z;

		temp.set(cameraPos).translate(lightDir, minDepth - minToLightDist);
		MatrixOps.vectorMult(rotation, temp, temp);// transform to light-space
		VectorOps.min(min, temp, min);
		VectorOps.max(max, temp, max);

		//min.z = Math.max(min.z, max.z - maxDepth);

		MatrixOps.createOrthographicProjectionMatrix(min, max, dest);
		Matrix4f.mul(dest, rotation, dest);
		return dest;
	}
}
