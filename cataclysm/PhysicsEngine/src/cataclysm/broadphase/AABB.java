package cataclysm.broadphase;

import math.vector.Vector3f;

/**
 * Repr�sente une bo�te align�e sur les axes xyz.
 * 
 * @author Briac
 *
 */
public class AABB {

	public float minX;
	public float minY;
	public float minZ;
	public float maxX;
	public float maxY;
	public float maxZ;
	private float surfaceArea = 0;

	/**
	 * Initialise l'AABB avec min = max = vec3(0, 0, 0);
	 */
	public AABB() {

	}

	/**
	 * Construit une AABB � partir d'un point central et d'une taille. <br>
	 * L'AABB mesurera 2*radius selon chaque direction.
	 * 
	 * @param center
	 * @param radius
	 */
	public AABB(Vector3f center, float radius) {
		set(center, radius);
	}
	
	public void set(Vector3f center, float radius) {
		minX = center.x - radius;
		minY = center.y - radius;
		minZ = center.z - radius;
		maxX = center.x + radius;
		maxY = center.y + radius;
		maxZ = center.z + radius;
	}

	/**
	 * Teste si cette AABB contient entièrement other.<br>
	 * Note: {@code this.contains(this)} vaut toujours true.
	 * 
	 * @param other
	 * @return true si other est à l'intérieur de cette AABB.
	 */
	public boolean contains(AABB other) {
		if (minX <= other.minX && maxX >= other.maxX) {
			if (minZ <= other.minZ && maxZ >= other.maxZ) {
				if (minY <= other.minY && maxY >= other.maxY) {
					return true;
				}
			}
		}
		return false;
	}

	/**
	 * Calcule l'union de deux AABB.
	 * 
	 * @param left
	 * @param right
	 * @return l'AABB la plus petite telle que left & right soient comprises �
	 *         l'int�rieur.
	 */
	public static AABB union(AABB left, AABB right) {
		AABB dest = new AABB();
		union(left, right, dest);
		return dest;
	}
	
	private static float min(float a, float b) {
		return a < b ? a : b;
	}
	private static float max(float a, float b) {
		return a > b ? a : b;
	}
	/**
	 * Calcule l'union de deux AABB et place le r�sultat dans dest.
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void union(AABB left, AABB right, AABB dest) {
		dest.minX = min(left.minX, right.minX);
		dest.minY = min(left.minY, right.maxY);
		dest.minZ = min(left.minZ, right.maxZ);
		dest.maxX = max(left.maxX, right.maxX);
		dest.maxY = max(left.maxY, right.maxY);
		dest.maxZ = max(left.maxZ, right.maxZ);
		dest.computeSurfaceArea();
	}

	public static boolean intersect(AABB left, AABB right) {
		if (left.maxX > right.minX && left.minX < right.maxX) {
			if (left.maxZ > right.minZ && left.minZ < right.maxZ) {
				if (left.maxY > right.minY && left.minY < right.maxY) {
					return true;
				}
			}
		}
		return false;
	}

	float getSurfaceArea() {
		return surfaceArea;
	}

	public float computeSurfaceArea() {
		float dx = maxX - minX;
		float dy = maxY - minY;
		float dz = maxZ - minZ;
		surfaceArea = 2.0f * (dx * dy + dy * dz + dz * dx);
		return surfaceArea;
	}

	@Override
	public String toString() {
		return "AABB[ (" + minX + ", " + minY + ", " + minZ + ") | (" + maxX + ", " + maxY + ", " + maxZ + ") ]";
	}

}
