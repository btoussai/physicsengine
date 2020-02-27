package math;

import math.vector.Vector2f;
import math.vector.Vector3f;
import math.vector.Vector4f;

/**
 * Cette classe rassemble des fonctions pour calculer l'interpolation entre deux
 * valeurs.
 * 
 * @author Briac
 *
 */
public class Interpolation {

	/**
	 * Calcule l'interpolation lin�aire entre deux valeurs.
	 * 
	 * @param a
	 * @param b
	 * @param amount
	 * @return
	 */
	public static float linearInterpolation(float a, float b, float amount) {
		return a + (b - a) * amount;
	}

	/**
	 * Calcule l'interpolation lin�aire entre deux valeurs.
	 * 
	 * @param a
	 * @param b
	 * @param amount
	 * @return
	 */
	public static double linearInterpolation(double a, double b, double amount) {
		return a + (b - a) * amount;
	}

	/**
	 * Calcule l'interpolation lin�aire entre deux vecteurs.
	 * 
	 * @param a
	 * @param b
	 * @param dest   Le vecteur dans lequel stocker le r�sultat ou null si un
	 *               vecteur doit �tre cr��.
	 * @param amount
	 * @return Le r�sultat de l'interpolation.
	 */
	public static Vector4f linearInterpolation(Vector4f a, Vector4f b, Vector4f dest, float amount) {
		if (dest == null) {
			dest = new Vector4f();
		}
		dest.set(linearInterpolation(a.x, b.x, amount), linearInterpolation(a.y, b.y, amount),
				linearInterpolation(a.z, b.z, amount), linearInterpolation(a.w, b.w, amount));
		return dest;
	}

	/**
	 * Calcule l'interpolation lin�aire entre deux vecteurs.
	 * 
	 * @param a
	 * @param b
	 * @param dest   Le vecteur dans lequel stocker le r�sultat ou null si un
	 *               vecteur doit �tre cr��.
	 * @param amount
	 * @return Le r�sultat de l'interpolation.
	 */
	public static Vector3f linearInterpolation(Vector3f a, Vector3f b, Vector3f dest, float amount) {
		if (dest == null) {
			dest = new Vector3f();
		}
		dest.set(linearInterpolation(a.x, b.x, amount), linearInterpolation(a.y, b.y, amount),
				linearInterpolation(a.z, b.z, amount));
		return dest;
	}

	/**
	 * Calcule l'interpolation lin�aire entre deux vecteurs.
	 * 
	 * @param a
	 * @param b
	 * @param dest   Le vecteur dans lequel stocker le r�sultat ou null si un
	 *               vecteur doit �tre cr��.
	 * @param amount
	 * @return Le r�sultat de l'interpolation.
	 */
	public static Vector2f linearInterpolation(Vector2f a, Vector2f b, Vector2f dest, float amount) {
		if (dest == null) {
			dest = new Vector2f();
		}
		dest.set(linearInterpolation(a.x, b.x, amount), linearInterpolation(a.y, b.y, amount));
		return dest;
	}

	/**
	 * Calcule l'interpolation cubique entre 2 valeurs.
	 * 
	 * @param a
	 * @param b
	 * @param amount
	 * @return
	 */
	public static float cubicInterpolation(float a, float b, float amount) {
		amount = amount * amount * (3.0f - 2.0f * amount);
		return a + (b - a) * amount;
	}

	/**
	 * Calcule l'interpolation cubique entre 2 valeurs.
	 * 
	 * @param a
	 * @param b
	 * @param amount
	 * @return
	 */
	public static double cubicInterpolation(double a, double b, double amount) {
		amount = amount * amount * (3.0 - 2.0 * amount);
		return a + (b - a) * amount;
	}

	/**
	 * Calcule l'interpolation cubique entre deux vecteurs.
	 * 
	 * @param a
	 * @param b
	 * @param dest   Le vecteur dans lequel stocker le r�sultat ou null si un
	 *               vecteur doit �tre cr��.
	 * @param amount
	 * @return Le r�sultat de l'interpolation.
	 */
	public static Vector4f cubicInterpolation(Vector4f a, Vector4f b, Vector4f dest, float amount) {
		if (dest == null) {
			dest = new Vector4f();
		}
		dest.set(cubicInterpolation(a.x, b.x, amount), cubicInterpolation(a.y, b.y, amount),
				cubicInterpolation(a.z, b.z, amount), cubicInterpolation(a.w, b.w, amount));
		return dest;
	}

	/**
	 * Calcule l'interpolation cubique entre deux vecteurs.
	 * 
	 * @param a
	 * @param b
	 * @param dest   Le vecteur dans lequel stocker le r�sultat ou null si un
	 *               vecteur doit �tre cr��.
	 * @param amount
	 * @return Le r�sultat de l'interpolation.
	 */
	public static Vector3f cubicInterpolation(Vector3f a, Vector3f b, Vector3f dest, float amount) {
		if (dest == null) {
			dest = new Vector3f();
		}
		dest.set(cubicInterpolation(a.x, b.x, amount), cubicInterpolation(a.y, b.y, amount),
				cubicInterpolation(a.z, b.z, amount));
		return dest;
	}

	/**
	 * Calcule l'interpolation cubique entre deux vecteurs.
	 * 
	 * @param a
	 * @param b
	 * @param dest   Le vecteur dans lequel stocker le r�sultat ou null si un
	 *               vecteur doit �tre cr��.
	 * @param amount
	 * @return Le r�sultat de l'interpolation.
	 */
	public static Vector2f cubicInterpolation(Vector2f a, Vector2f b, Vector2f dest, float amount) {
		if (dest == null) {
			dest = new Vector2f();
		}
		dest.set(cubicInterpolation(a.x, b.x, amount), cubicInterpolation(a.y, b.y, amount));
		return dest;
	}

	/**
	 * Calcule l'interpolation entre deux angles en degr�s.
	 * 
	 * @param a      -l'angle a en degr�s
	 * @param b      -l'angle b en degr�s
	 * @param amount -le montant d'interpolation entre a et b compris entre 0.0f et
	 *               1.0f.
	 * @return l'angle r�sultat compris entre 0 et 360;
	 */
	public static float angularInterpolation(float a, float b, float amount) {

		a = a % 360f;
		b = b % 360f;
		if (a < 0)
			a += 360;
		if (b < 0)
			b += 360;

		if (Math.abs(a - b) < 180) {
			return linearInterpolation(a, b, amount);
		}

		if (a < b) {
			a += 360;
		} else {
			b += 360;
		}

		float angle = linearInterpolation(a, b, amount);
		if (angle > 360)
			angle -= 360;

		return angle;
	}

	/**
	 * Attribue un poids � chaque sommet du triangle en fonction de la position du
	 * point d'interpolation. La somme des poids vaut toujours 1.
	 * 
	 * @param P1
	 * @param P2
	 * @param P3
	 * @param coord
	 * @return un vecteur � trois composantes contenant les poids respectifs des
	 *         sommets.
	 */
	public static Vector3f interpolationTriangulaire(Vector2f P1, Vector2f P2, Vector2f P3, Vector2f coord) {

		Vector2f i1 = Maths.intersection(P2, P3, coord, P1);
		Vector2f i2 = Maths.intersection(P1, P3, coord, P2);
		Vector2f i3 = Maths.intersection(P1, P2, coord, P3);

		Vector2f P1I1 = new Vector2f(i1.x - P1.x, i1.y - P1.y);
		Vector2f P2I2 = new Vector2f(i2.x - P2.x, i2.y - P2.y);
		Vector2f P3I3 = new Vector2f(i3.x - P3.x, i3.y - P3.y);
		Vector2f P1Coord = new Vector2f(coord.x - P1.x, coord.y - P1.y);
		Vector2f P2Coord = new Vector2f(coord.x - P2.x, coord.y - P2.y);
		Vector2f P3Coord = new Vector2f(coord.x - P3.x, coord.y - P3.y);

		float influenceP1I1 = 1 - P1Coord.length() / P1I1.length();
		if (influenceP1I1 != influenceP1I1)
			influenceP1I1 = 1;
		float influenceP2I2 = 1 - P2Coord.length() / P2I2.length();
		if (influenceP2I2 != influenceP2I2)
			influenceP2I2 = 1;
		float influenceP3I3 = 1 - P3Coord.length() / P3I3.length();
		if (influenceP3I3 != influenceP3I3)
			influenceP3I3 = 1;

		float sum = influenceP1I1 + influenceP2I2 + influenceP3I3;
		return new Vector3f(influenceP1I1 / sum, influenceP2I2 / sum, influenceP3I3 / sum);
	}

}
