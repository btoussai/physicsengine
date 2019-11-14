package math;

import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

public class Maths {

	public static float min(float x, float y) {
		return (x < y) ? x : y;
	}
	
	public static float max(float x, float y) {
		return (x > y) ? x : y;
	}

	/**
	 * @param f
	 * @return 0 if f<0 and 1 if f >= 0
	 */
	public static int getSign(float f) {
		return 1 - (Float.floatToIntBits(f) >>> 31);
	}

	public static float mean(float valueA, float valueB) {
		return (valueA + valueB) / 2.0f;
	}

	public static float mean(float valueA, float valueB, float valueC) {
		return (valueA + valueB + valueC) / 3.0f;
	}

	public static float mean(float[] values) {
		return sum(values) / (float) values.length;
	}

	public static float sum(float[] values) {
		float sum = 0;
		for (int i = 0; i < values.length; i++) {
			sum += values[i];
		}
		return sum;
	}

	public static Vector3f mean(Vector3f v1, Vector3f v2) {
		return new Vector3f((v1.x + v2.x) / 2.0f, (v1.y + v2.y) / 2.0f, (v1.z + v2.z) / 2.0f);
	}

	public static Vector3f mean(Vector3f v1, Vector3f v2, Vector3f v3) {
		return new Vector3f((v1.x + v2.x + v3.x) / 3.0f, (v1.y + v2.y + v3.y) / 3.0f, (v1.z + v2.z + v3.z) / 3.0f);
	}

	public static float rayPlaneIntersection(Vector3f basePoint, Vector3f edge, Plane plane) {
		return -(Vector3f.dot(plane.normal, basePoint) + plane.equation.w) / Vector3f.dot(plane.normal, edge);
	}

	public static boolean checkPointInsideTriangle(Vector3f point, Vector3f pa, Vector3f pb, Vector3f pc) {

		Vector3f e10 = Vector3f.sub(pb, pa, null);
		Vector3f e20 = Vector3f.sub(pc, pa, null);
		float a = Vector3f.dot(e10, e10);
		float b = Vector3f.dot(e10, e20);
		float c = Vector3f.dot(e20, e20);
		float ac_bb = a * c - b * b;

		Vector3f vp = Vector3f.sub(point, pa, null);

		float d = Vector3f.dot(vp, e10);
		float e = Vector3f.dot(vp, e20);
		float x = d * c - e * b;
		float y = e * a - d * b;
		float z = x + y - ac_bb;

		return z < 0 && x >= 0 && y >= 0;
	}

	public static float distance2D(float a, float b) {
		return (float) Math.sqrt(a * a + b * b);
	}

	public static float distance3D(float a, float b, float c) {
		return (float) Math.sqrt(a * a + b * b + c * c);
	}

	public static float distanceSquared(Vector3f v0, Vector3f v1) {
		float a = v0.x - v1.x;
		float b = v0.y - v1.y;
		float c = v0.z - v1.z;
		return (a * a + b * b + c * c);
	}

	public static Vector2f intersection(Vector2f a, Vector2f b, Vector2f c, Vector2f d) {

		Vector2f u = new Vector2f(b.x - a.x, b.y - a.y);
		Vector2f v = new Vector2f(d.x - c.x, d.y - c.y);

		Vector3f droite1 = new Vector3f(u.y, -u.x, u.y * a.x - u.x * a.y);
		Vector3f droite2 = new Vector3f(v.y, -v.x, v.y * c.x - v.x * c.y);

		float det = droite1.x * droite2.y - droite1.y * droite2.x;

		Vector2f coords = new Vector2f(0, 0);
		coords.x = (droite1.z * droite2.y - droite2.z * droite1.y) / det;
		coords.y = (droite1.z * -droite2.x + droite2.z * droite1.x) / det;

		return coords;
	}

	public static Vector4f equationPlan(Vector3f point, Vector3f vecteurNormal) {
		Vector4f equation = new Vector4f();
		equation.x = vecteurNormal.x;
		equation.y = vecteurNormal.y;
		equation.z = vecteurNormal.z;
		equation.w = (vecteurNormal.x * point.x + vecteurNormal.y * point.y + vecteurNormal.z * point.z) * -1f;

		return equation;
	}

	public static Vector3f intersectionDroitePlan(Vector3f vecteurDirecteur, Vector3f point, Vector4f equationPlan) {

		float number = vecteurDirecteur.x * equationPlan.x + vecteurDirecteur.y * equationPlan.y
				+ vecteurDirecteur.z * equationPlan.z;
		if (number == 0) {
			return null;
		}

		float param = -(equationPlan.w + point.x * equationPlan.x + point.y * equationPlan.y + point.z * equationPlan.z)
				/ number;

		if (param < 0) {
			return null;
		}

		Vector3f intersection = new Vector3f(0, 0, 0);

		intersection.x = param * vecteurDirecteur.x + point.x;
		intersection.y = param * vecteurDirecteur.y + point.y;
		intersection.z = param * vecteurDirecteur.z + point.z;

		return intersection;
	}

	public static boolean isVertexInsideQuad(Vector3f vertex, Vector3f vertex1, Vector3f vertex2, Vector3f vertex3) {
		Vector3f alpha = Vector3f.sub(vertex, vertex1, null);
		Vector3f U = Vector3f.sub(vertex2, vertex1, null);
		float scalaireAlphaU = Vector3f.dot(alpha, U);
		float normU = U.length();
		if (scalaireAlphaU < 0)
			return false;
		if (scalaireAlphaU / normU > normU)
			return false;
		U = Vector3f.sub(vertex3, vertex1, null);
		scalaireAlphaU = Vector3f.dot(alpha, U);
		normU = U.length();
		if (scalaireAlphaU < 0)
			return false;
		if (scalaireAlphaU / normU > normU)
			return false;
		return true;
	}

	public static boolean isVertexInsideTriangle2D(Vector2f point, Vector2f P1, Vector2f P2, Vector2f P3) {

		Vector2f A = Vector2f.sub(P2, P1, null);
		Vector2f B = Vector2f.sub(P3, P1, null);
		Vector2f X = Vector2f.sub(point, P1, null);

		float a = A.y / A.x;

		float V = (X.y - a * X.x) / (-a * B.x + B.y);
		float U = (X.x - B.x * V) / A.x;

		System.out.println("Combinaison linéaire: X= A*" + U + " + B*" + V);
		System.out.println(A.x + "*" + U + " + " + B.x + "*" + V + " = " + (A.x * U + B.x * V) + " | " + X.x);
		System.out.println(A.y + "*" + U + " + " + B.y + "*" + V + " = " + (A.y * U + B.y * V) + " | " + X.y);

		if (U + V <= 1.0f && U >= 0.0f && V >= 0.0f) {
			return true;
		}

		return false;
	}

	public static boolean isVertexInsideTriangle3D(Vector3f point, Vector3f P1, Vector3f P2, Vector3f P3) {

		Vector3f A = Vector3f.sub(P2, P1, null);
		Vector3f B = Vector3f.sub(P3, P1, null);
		Vector3f X = Vector3f.sub(point, P1, null);

		float det = -A.x * B.y + A.y * B.x, U = -1, V = -1;
		if (det != 0) {
			U = X.x * -B.y / det + X.y * B.x / det;
			V = X.x * A.y / det - X.y * A.x / det;
		} else {
			det = -A.x * B.z + A.z * B.x;
			if (det != 0) {
				U = X.x * -B.z / det + X.z * B.x / det;
				V = X.x * A.z / det - X.z * A.x / det;
			} else {
				det = -A.y * B.z + A.z * B.y;
				if (det != 0) {
					U = X.y * -B.z / det + X.z * B.y / det;
					V = X.y * A.z / det - X.z * A.y / det;
				} else {
					// no solutions
				}
			}
		}
		if (U + V <= 1.0f && U >= 0.0f && V >= 0.0f) {
			return true;
		}

		return false;
	}

	public static float getLowestRoot(float a, float b, float c, float maxRange) {

		float delta = b * b - 4.0f * a * c;
		if (delta >= 0.0f) {
			delta = (float) Math.sqrt(delta);
			float x1 = (-b - delta) / (2.0f * a);
			float x2 = (-b + delta) / (2.0f * a);
			if (x1 > x2) {
				float temp = x2;
				x2 = x1;
				x1 = temp;
			}
			if (x1 >= 0.0f && x1 < maxRange) {
				return x1;
			}
			if (x2 >= 0.0f && x2 < maxRange) {
				return x2;
			}
		}
		return Float.NaN;
	}

	public static String getLowestRoot(float a, float b, float c) {

		float delta = b * b - 4.0f * a * c;
		if (delta >= 0.0f) {
			delta = (float) Math.sqrt(delta);
			float x1 = (-b - delta) / (2.0f * a);
			float x2 = (-b + delta) / (2.0f * a);
			if (x1 > x2) {
				float temp = x2;
				x2 = x1;
				x1 = temp;
			}
			return "Delta: " + delta + " x1: " + x1 + " x2: " + x2;
		}
		return " Delta²: " + delta;
	}

	public static double getLowestRoot(double a, double b, double c, double maxRange, double precision) {

		double delta = b * b - 4.0 * a * c;
		if (delta >= 0.0) {
			delta = Math.sqrt(delta);
			double x1 = (-b - delta) / (2.0 * a);
			double x2 = (-b + delta) / (2.0 * a);
			if (x1 > x2) {
				double temp = x2;
				x2 = x1;
				x1 = temp;
			}
			if (x1 >= -precision && x1 < maxRange) {
				if (x1 >= 0.0)
					return x1;
				return 0.0;
			}
			if (x2 >= -precision && x2 < maxRange) {
				if (x2 >= 0.0)
					return x2;
				return 0.0;
			}
		}
		return Double.NaN;
	}

}
