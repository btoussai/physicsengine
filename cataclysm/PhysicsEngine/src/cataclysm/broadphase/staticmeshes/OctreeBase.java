package cataclysm.broadphase.staticmeshes;

import math.vector.Vector2f;
import math.vector.Vector3f;

/**
 * Repr�sente une cellule de base d'un octree.
 * 
 * @author Briac
 *
 */
class OctreeBase {

	final Coord coord;
	final OctreeCell rootCell;

	final Vector3f center;
	final Vector3f minCorner;
	final Vector3f maxCorner;

	OctreeBase(float size, Coord coord, int maxDepth) {
		this.coord = new Coord(coord.getX(), coord.getY(), coord.getZ());
		center = new Vector3f(coord.getX() * size, coord.getY() * size, coord.getZ() * size);
		rootCell = new OctreeCell(center, size, maxDepth);

		float half_size = 0.5f * size;
		minCorner = new Vector3f(center.x - half_size, center.y - half_size, center.z - half_size);
		maxCorner = new Vector3f(center.x + half_size, center.y + half_size, center.z + half_size);
	}

	boolean intersects(Vector3f min, Vector3f max) {
		if (max.x < minCorner.x || min.x > maxCorner.x) {
			if (max.y < minCorner.y || min.y > maxCorner.y) {
				if (max.z < minCorner.z || min.z > maxCorner.z) {
					return false;
				}
			}
		}

		return true;
	}

	/**
	 * D�termine les coordonn�es de la prochaine cellule dans la direction du rayon.
	 * 
	 * @param start            Le point de d�part du rayon
	 * @param dir              La direction du rayon, le vecteur doit �tre unitaire
	 * @param maxLength        La distance maximale que la rayon peut parcourir
	 * @param cellSize         La taille d'une cellule
	 * @param current          Les coordonn�es de la cellule actuelle
	 * @param dest             Les coordonn�es de la prochaine cellule
	 * @param intersectionTime Les distances d'intersection min/max du rayon et de
	 *                         la cellule.
	 * @return false si la prochaine cellule est trop loin pour �tre atteinte par le
	 *         rayon.
	 */
	static boolean nextCell(Vector3f start, Vector3f dir, float maxLength, float cellSize, Coord current, Coord dest,
			Vector2f intersectionTime) {
		
		if(intersectionTime.y > maxLength) {
			return false;
		}

		int intX = current.getX();
		int intY = current.getY();
		int intZ = current.getZ();

		float centerX = intX * cellSize;
		float centerY = intY * cellSize;
		float centerZ = intZ * cellSize;

		float half_size = 0.5f * cellSize;

		float x = start.x + dir.x * intersectionTime.y - centerX;
		float y = start.y + dir.y * intersectionTime.y - centerY;
		float z = start.z + dir.z * intersectionTime.y - centerZ;

		final float epsilon = 1E-2f;

		if (Math.abs(Math.abs(x) - half_size) < epsilon) {
			int sgnX = (int) Math.signum(x);
			int sgnDirX = (int) Math.signum(dir.x);
			intX += (sgnX + sgnDirX) / 2;
		}
		if (Math.abs(Math.abs(y) - half_size) < epsilon) {
			int sgnY = (int) Math.signum(y);
			int sgnDirY = (int) Math.signum(dir.y);
			intY += (sgnY + sgnDirY) / 2;
		}
		if (Math.abs(Math.abs(z) - half_size) < epsilon) {
			int sgnZ = (int) Math.signum(z);
			int sgnDirZ = (int) Math.signum(dir.z);
			intZ += (sgnZ + sgnDirZ) / 2;
		}

		dest.set(intX, intY, intZ);

		intersectionTime.set(intersectionTime.y, computeMaxIntersectionTime(start, dir, dest, cellSize));

		return true;
	}
	
	/**
	 * Calcule la distance d'intersection max du rayon et de la cellule.
	 * @param start
	 * @param dir
	 * @param cell
	 * @param cellSize
	 * @return
	 */
	private static float computeMaxIntersectionTime(Vector3f start, Vector3f dir, Coord cell, float cellSize) {
		float centerX = cell.getX() * cellSize;
		float centerY = cell.getY() * cellSize;
		float centerZ = cell.getZ() * cellSize;

		float half_size = 0.5f * cellSize;

		float tmax = Float.POSITIVE_INFINITY;
		float bmin, bmax;

		bmin = centerX - half_size;
		bmax = centerX + half_size;
		if (dir.x != 0.0) {
			float t1 = (bmin - start.x) / dir.x;
			float t2 = (bmax - start.x) / dir.x;

			tmax = Math.min(tmax, Math.max(t1, t2));
		}

		bmin = centerY - half_size;
		bmax = centerY + half_size;
		if (dir.y != 0.0) {
			float t1 = (bmin - start.y) / dir.y;
			float t2 = (bmax - start.y) / dir.y;

			tmax = Math.min(tmax, Math.max(t1, t2));
		}

		bmin = centerZ - half_size;
		bmax = centerZ + half_size;
		if (dir.z != 0.0) {
			float t1 = (bmin - start.z) / dir.z;
			float t2 = (bmax - start.z) / dir.z;

			tmax = Math.min(tmax, Math.max(t1, t2));
		}
		
		return tmax;
	}

	/**
	 * Calcule les distances d'intersection min/max du rayon et de la cellule.
	 * 
	 * @param dest
	 */
	static void computeIntersectionTime(Vector3f start, Vector3f dir, Coord cell, float cellSize, Vector2f dest) {
		float centerX = cell.getX() * cellSize;
		float centerY = cell.getY() * cellSize;
		float centerZ = cell.getZ() * cellSize;

		float half_size = 0.5f * cellSize;

		float tmin = Float.NEGATIVE_INFINITY, tmax = Float.POSITIVE_INFINITY;
		float bmin, bmax;

		bmin = centerX - half_size;
		bmax = centerX + half_size;
		if (dir.x != 0.0) {
			float t1 = (bmin - start.x) / dir.x;
			float t2 = (bmax - start.x) / dir.x;

			tmin = Math.max(tmin, Math.min(t1, t2));
			tmax = Math.min(tmax, Math.max(t1, t2));
		}

		bmin = centerY - half_size;
		bmax = centerY + half_size;
		if (dir.y != 0.0) {
			float t1 = (bmin - start.y) / dir.y;
			float t2 = (bmax - start.y) / dir.y;

			tmin = Math.max(tmin, Math.min(t1, t2));
			tmax = Math.min(tmax, Math.max(t1, t2));
		}

		bmin = centerZ - half_size;
		bmax = centerZ + half_size;
		if (dir.z != 0.0) {
			float t1 = (bmin - start.z) / dir.z;
			float t2 = (bmax - start.z) / dir.z;

			tmin = Math.max(tmin, Math.min(t1, t2));
			tmax = Math.min(tmax, Math.max(t1, t2));
		}

		dest.set(tmin, tmax);
	}

}
