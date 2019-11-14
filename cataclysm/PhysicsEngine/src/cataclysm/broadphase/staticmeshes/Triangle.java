package cataclysm.broadphase.staticmeshes;

import org.lwjgl.util.vector.Vector3f;

/**
 * Représente un triangle dans un PhysicsMesh.
 * 
 * @author Briac
 *
 */
public class Triangle {

	public final Vector3f v1;
	public final Vector3f v2;
	public final Vector3f v3;
	public final Vector3f normal;
	private final float plane_offset;

	/**
	 * Le maillage auquel appartient ce triangle.
	 */
	public final StaticMesh mesh;

	public Triangle(StaticMesh mesh, Vector3f p1, Vector3f p2, Vector3f p3) {
		this.v1 = p1;
		this.v2 = p2;
		this.v3 = p3;

		normal = Vector3f.cross(Vector3f.sub(p2, p1, null), Vector3f.sub(p3, p1, null), null);
		float length = normal.length();
		if (length < 1E-6f) {
			throw new ArithmeticException("Error, degenerate triangle. Normal length is zero.");
		}
		normal.scale(1.0f / length);
		plane_offset = Vector3f.dot(normal, p1);

		this.mesh = mesh;
	}

	public void getMinMax(Vector3f min, Vector3f max) {
		getMinMaxX(min, max);
		getMinMaxY(min, max);
		getMinMaxZ(min, max);
	}

	private void getMinMaxX(Vector3f min, Vector3f max) {
		float a = v1.x;
		float b = v2.x;
		float c = v3.x;

		if (a > b) {
			float temp = a;
			a = b;
			b = temp;
		}

		if (b > c) {
			float temp = b;
			b = c;
			c = temp;
		}

		if (a > b) {
			float temp = a;
			a = b;
			b = temp;
		}

		min.x = Math.min(min.x, a);
		max.x = Math.max(max.x, c);
	}

	private void getMinMaxY(Vector3f min, Vector3f max) {
		float a = v1.y;
		float b = v2.y;
		float c = v3.y;

		if (a > b) {
			float temp = a;
			a = b;
			b = temp;
		}

		if (b > c) {
			float temp = b;
			b = c;
			c = temp;
		}

		if (a > b) {
			float temp = a;
			a = b;
			b = temp;
		}

		min.y = Math.min(min.y, a);
		max.y = Math.max(max.y, c);
	}

	private void getMinMaxZ(Vector3f min, Vector3f max) {
		float a = v1.z;
		float b = v2.z;
		float c = v3.z;

		if (a > b) {
			float temp = a;
			a = b;
			b = temp;
		}

		if (b > c) {
			float temp = b;
			b = c;
			c = temp;
		}

		if (a > b) {
			float temp = a;
			a = b;
			b = temp;
		}

		min.z = Math.min(min.z, a);
		max.z = Math.max(max.z, c);
	}

	/**
	 * Teste si un rayon intersecte ce triangle.
	 * 
	 * @param start            Le point de départ du rayon.
	 * @param dir              La direction du rayon, le vecteur doit être unitaire.
	 * @param maxLength        La distance maximale que le rayon est autorisé à
	 *                         parcourir.
	 * @param intersectionDest Le point d'intersection entre le rayon et le
	 *                         triangle.
	 * @param backfaceCulling  true pour rejeter le triangle si N.dir > 0.
	 * @return La distance au point d'intersection ou bien maxLength si le test
	 *         échoue.
	 */
	public float rayTest(Vector3f start, Vector3f dir, float maxLength, Vector3f intersectionDest,
			boolean backfaceCulling) {

		float dot = Vector3f.dot(dir, normal);
		if (backfaceCulling) {
			if (dot > 0) {
				return maxLength;
			}
		}

		if (Math.abs(dot) < 1E-6f) {
			return maxLength;
		}

		float d = (plane_offset - Vector3f.dot(start, normal)) / dot;
		if (d > maxLength || d < 0) {
			return maxLength;
		}

		intersectionDest.set(start.x + dir.x * d, start.y + dir.y * d, start.z + dir.z * d);
		if (checkPointInsideTriangle(intersectionDest, v1, v2, v3)) {
			return d;
		}
		return maxLength;
	}

	public boolean checkPointInsideTriangle(Vector3f point, Vector3f pa, Vector3f pb, Vector3f pc) {

		float e10x = pb.x - pa.x;
		float e10y = pb.y - pa.y;
		float e10z = pb.z - pa.z;

		float e20x = pc.x - pa.x;
		float e20y = pc.y - pa.y;
		float e20z = pc.z - pa.z;

		float a = e10x * e10x + e10y * e10y + e10z * e10z;
		float b = e10x * e20x + e10y * e20y + e10z * e20z;
		float c = e20x * e20x + e20y * e20y + e20z * e20z;
		float ac_bb = a * c - b * b;

		float vpx = point.x - pa.x;
		float vpy = point.y - pa.y;
		float vpz = point.z - pa.z;

		float d = vpx * e10x + vpy * e10y + vpz * e10z;
		float e = vpx * e20x + vpy * e20y + vpz * e20z;
		float x = d * c - e * b;
		float y = e * a - d * b;
		float z = x + y - ac_bb;

		return z < 0 && x >= 0 && y >= 0;
	}

}
