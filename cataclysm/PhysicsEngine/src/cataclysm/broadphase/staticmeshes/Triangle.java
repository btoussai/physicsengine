package cataclysm.broadphase.staticmeshes;

import math.vector.Vector3f;

/**
 * A triangle whose data is 'flattened'
 * 
 * @author Briac
 *
 */
public final class Triangle {

	private final float v0_x;
	private final float v0_y;
	private final float v0_z;
	private final float v1_x;
	private final float v1_y;
	private final float v1_z;
	private final float v2_x;
	private final float v2_y;
	private final float v2_z;
	private final float n_x;
	private final float n_y;
	private final float n_z;
	private final float plane_offset;

	/**
	 * Le maillage auquel appartient ce triangle.
	 */
	public final StaticMesh mesh;

	public Triangle(StaticMesh mesh, Vector3f p1, Vector3f p2, Vector3f p3) {
		v0_x = p1.x;
		v0_y = p1.y;
		v0_z = p1.z;
		
		v1_x = p2.x;
		v1_y = p2.y;
		v1_z = p2.z;
		
		v2_x = p3.x;
		v2_y = p3.y;
		v2_z = p3.z;
		
		float e0_x = v1_x - v0_x;
		float e0_y = v1_y - v0_y;
		float e0_z = v1_z - v0_z;

		float e2_x = v2_x - v0_x;
		float e2_y = v2_y - v0_y;
		float e2_z = v2_z - v0_z;

		// e2 ^ e0
		float n_x = e0_y * e2_z - e0_z * e2_y;
		float n_y = e2_x * e0_z - e2_z * e0_x;
		float n_z = e0_x * e2_y - e0_y * e2_x;

		float length = (float) Math.sqrt(n_x * n_x + n_y * n_y + n_z * n_z);
		if (length < 1E-6f) {
			throw new ArithmeticException("Error, degenerate triangle. Normal length is zero.");
		}
		float inv_l = 1.0f / length;
		this.n_x = n_x * inv_l;
		this.n_y = n_y * inv_l;
		this.n_z = n_z * inv_l;
		plane_offset = v0_x * this.n_x + v0_y * this.n_y + v0_z * this.n_z;

		this.mesh = mesh;
	}

	public void getMinMax(Vector3f min, Vector3f max) {
		getMinMaxX(min, max);
		getMinMaxY(min, max);
		getMinMaxZ(min, max);
	}

	private void getMinMaxX(Vector3f min, Vector3f max) {
		float a = v0_x;
		float b = v1_x;
		float c = v2_x;

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
		float a = v0_y;
		float b = v1_y;
		float c = v2_y;

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
		float a = v0_z;
		float b = v1_z;
		float c = v2_z;

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
	 * @param start            Le point de d�part du rayon.
	 * @param dir              La direction du rayon, le vecteur doit �tre unitaire.
	 * @param maxLength        La distance maximale que le rayon est autoris� �
	 *                         parcourir.
	 * @param intersectionDest Le point d'intersection entre le rayon et le
	 *                         triangle.
	 * @param backfaceCulling  true pour rejeter le triangle si N.dir > 0.
	 * @return La distance au point d'intersection ou bien maxLength si le test
	 *         �choue.
	 */
	public float rayTest(Vector3f start, Vector3f dir, float maxLength, Vector3f intersectionDest,
			boolean backfaceCulling) {

		float dot = dir.x * n_x + dir.y * n_y + dir.z * n_z;
		if (backfaceCulling) {
			if (dot > 0) {
				return maxLength;
			}
		}

		if (Math.abs(dot) < 1E-6f) {
			return maxLength;
		}

		float d = (plane_offset - (start.x * n_x + start.y * n_y + start.z * n_z)) / dot;
		if (d > maxLength || d < 0) {
			return maxLength;
		}

		intersectionDest.set(start.x + dir.x * d, start.y + dir.y * d, start.z + dir.z * d);
		if (checkPointInsideTriangle(intersectionDest)) {
			return d;
		}
		return maxLength;
	}

	public boolean checkPointInsideTriangle(Vector3f point) {

		float e10x = v1_x - v0_x;
		float e10y = v1_y - v0_y;
		float e10z = v1_z - v0_z;

		float e20x = v2_x - v0_x;
		float e20y = v2_y - v0_y;
		float e20z = v2_z - v0_z;

		float a = e10x * e10x + e10y * e10y + e10z * e10z;
		float b = e10x * e20x + e10y * e20y + e10z * e20z;
		float c = e20x * e20x + e20y * e20y + e20z * e20z;
		float ac_bb = a * c - b * b;

		float vpx = point.x - v0_x;
		float vpy = point.y - v0_y;
		float vpz = point.z - v0_z;

		float d = vpx * e10x + vpy * e10y + vpz * e10z;
		float e = vpx * e20x + vpy * e20y + vpz * e20z;
		float x = d * c - e * b;
		float y = e * a - d * b;
		float z = x + y - ac_bb;

		return z < 0 && x >= 0 && y >= 0;
	}

	public void getEdge0(Vector3f dest) {
		dest.x = v1_x - v0_x;
		dest.y = v1_y - v0_y;
		dest.z = v1_z - v0_z;
	}

	public void getEdge1(Vector3f dest) {
		dest.x = v2_x - v1_x;
		dest.y = v2_y - v1_y;
		dest.z = v2_z - v1_z;
	}

	public void getEdge2(Vector3f dest) {
		dest.x = v0_x - v2_x;
		dest.y = v0_y - v2_y;
		dest.z = v0_z - v2_z;
	}

	public Vector3f getNormal(Vector3f dest) {
		dest.x = n_x;
		dest.y = n_y;
		dest.z = n_z;
		return dest;
	}
	
	public Vector3f getV0(Vector3f dest) {
		dest.x = v0_x;
		dest.y = v0_y;
		dest.z = v0_z;
		return dest;
	}
	
	public Vector3f getV1(Vector3f dest) {
		dest.x = v1_x;
		dest.y = v1_y;
		dest.z = v1_z;
		return dest;
	}
	
	public Vector3f getV2(Vector3f dest) {
		dest.x = v2_x;
		dest.y = v2_y;
		dest.z = v2_z;
		return dest;
	}

	public float getNormal(int i) {
		switch (i) {
		case 0:
			return n_x;
		case 1:
			return n_y;
		case 2:
			return n_z;
		}
		throw new IllegalArgumentException();
	}

	public float getV0(int i) {
		switch (i) {
		case 0:
			return v0_x;
		case 1:
			return v0_y;
		case 2:
			return v0_z;
		}
		throw new IllegalArgumentException();
	}

	public float getV1(int i) {
		switch (i) {
		case 0:
			return v1_x;
		case 1:
			return v1_y;
		case 2:
			return v1_z;
		}
		throw new IllegalArgumentException();
	}

	public float getV2(int i) {
		switch (i) {
		case 0:
			return v2_x;
		case 1:
			return v2_y;
		case 2:
			return v2_z;
		}
		throw new IllegalArgumentException();
	}
	
	public float getPlaneOffset() {
		return plane_offset;
	}
}
