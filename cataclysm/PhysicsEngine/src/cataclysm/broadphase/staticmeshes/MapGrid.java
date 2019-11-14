package cataclysm.broadphase.staticmeshes;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import org.lwjgl.util.vector.Vector2f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.broadphase.AABB;

/**
 * Représente la grille 3D dans laquel sont stockés les triangles des
 * physicsmesh.
 * 
 * @author Briac
 *
 */
class MapGrid {

	private final float GRID_CELL_SIZE;
	private final int maxOctreeDepth;
	private final float ONE_OVER_GRID_CELL_SIZE;
	private final Map<Coord, OctreeBase> grid = new HashMap<Coord, OctreeBase>();

	private final Vector3f min = new Vector3f();
	private final Vector3f max = new Vector3f();
	private final Vector3f axis = new Vector3f();
	private final Vector3f[] edges = new Vector3f[3];

	private final Coord iterator = new Coord(0, 0, 0);
	private final Coord start = new Coord(0, 0, 0);
	private final Coord end = new Coord(0, 0, 0);
	private final CoordRange range = new CoordRange(start, end);

	public MapGrid(float cellSize, int maxOctreeDepth) {
		this.GRID_CELL_SIZE = cellSize;
		this.maxOctreeDepth = maxOctreeDepth;
		this.ONE_OVER_GRID_CELL_SIZE = 1.0f / GRID_CELL_SIZE;

		edges[0] = new Vector3f();
		edges[1] = new Vector3f();
		edges[2] = new Vector3f();

		cleanUp();
	}

	/**
	 * Ajoute les triangles du mesh dans la grille de l'octree.
	 * 
	 * @param mesh
	 */
	void add(StaticMesh mesh) {

		getCoordRange(mesh);
		range.startIteration();
		while (range.next(iterator)) {
			OctreeBase base = grid.get(iterator);
			if (base == null) {
				base = new OctreeBase(GRID_CELL_SIZE, iterator, maxOctreeDepth);
				grid.put(base.coord, base);
			}

			for (Triangle triangle : mesh.triangles) {
				min.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
				max.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
				triangle.getMinMax(min, max);

				Vector3f.sub(triangle.v2, triangle.v1, edges[0]);
				Vector3f.sub(triangle.v3, triangle.v2, edges[1]);
				Vector3f.sub(triangle.v1, triangle.v3, edges[2]);
				base.rootCell.insertTriangle(triangle, min, max, edges, axis);

			}

		}

	}

	/**
	 * Supprime les triangles du mesh dans la grille de l'octree.
	 * 
	 * @param mesh
	 */
	void remove(StaticMesh mesh) {

		getCoordRange(mesh);
		range.startIteration();
		while (range.next(iterator)) {
			OctreeBase base = grid.get(iterator);
			if (base == null) {
				continue;
			}

			for (Triangle triangle : mesh.triangles) {
				min.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
				max.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
				triangle.getMinMax(min, max);

				Vector3f.sub(triangle.v2, triangle.v1, edges[0]);
				Vector3f.sub(triangle.v3, triangle.v2, edges[1]);
				Vector3f.sub(triangle.v1, triangle.v3, edges[2]);
				base.rootCell.removeTriangle(triangle, min, max, edges, axis);

			}

		}

	}

	/**
	 * 
	 * Calcule la distance entre start et le premier triangle touché par le rayon
	 * ray.
	 * 
	 * @param start           Le point de départ du rayon.
	 * @param dir             La direction du rayon, le vecteur doit être unitaire.
	 * @param maxLength       La distance maximale de recherche, supérieure à zéro.
	 *                        Ne peut pas être +INF.
	 * @param backfaceCulling Les triangles ne faisant pas face au rayon seront
	 *                        ignorés si true.
	 * @param normalDest      La normale du triangle touché sera stockée dedans.
	 * @return la distance du premier triangle touché ou maxLength si aucun triangle
	 *         n'a été trouvé.
	 */
	float rayTest(Vector3f start, Vector3f dir, float maxLength, boolean backfaceCulling, Vector3f normalDest) {
		
		Vector2f intersectionTime = new Vector2f();
		Coord iterator =  new Coord(0, 0, 0);
		getCoord(start, iterator);
		OctreeBase.computeIntersectionTime(start, dir, iterator, GRID_CELL_SIZE, intersectionTime);
		while (true) {
			//System.out.println(iterator);
			OctreeBase base = grid.get(iterator);
			if (base != null) {
				float length = base.rootCell.rayTest(start, dir, maxLength, intersectionTime.x, intersectionTime.y, backfaceCulling, normalDest);
				if (length < maxLength) {
					return length;
				}
			}
			
			if(!OctreeBase.nextCell(start, dir, maxLength, this.GRID_CELL_SIZE, iterator, iterator, intersectionTime)) {
				break;
			}
			
		}

		return maxLength;
	}

	/**
	 * Récupère l'ensemble des triangles en intersection avec l'AABB.
	 * 
	 * @param box
	 * @param dest
	 */
	void boxTest(AABB box, HashSet<Triangle> dest) {
		getCoordRange(box);
		range.startIteration();
		while (range.next(iterator)) {
			OctreeBase base = grid.get(iterator);
			if (base != null) {
				base.rootCell.boxTest(box, dest);
			}
		}
	}

	void cleanUp() {
		grid.clear();
	}

	/**
	 * Explore l'octree et ajoute une boite affichable dans la liste boxes à chaque
	 * cellule rencontrée.
	 * 
	 * @param boxes        La liste dans laquelle ranger les boites affichables.
	 * @param maxDepth     La profondeur maximale d'exploration.
	 * @param leavesOnly   N'ajoute que les cellules de profondeur maxDepth.
	 * @param nonVoidBoxes Ignore les cellules vides.
	 * @param position     La position en worldspace du début de l'exploration.
	 */
	void exploreOctree(List<OctreeCellRenderable> boxes, int maxDepth, boolean leavesOnly, boolean nonVoidBoxes,
			Vector3f position) {
		getCoord(position, iterator);
		OctreeBase base = grid.get(iterator);
		if (base != null) {
			base.rootCell.exploreHierarchy(boxes, maxDepth, leavesOnly, nonVoidBoxes);
		}
	}

	private void getCoordRange(StaticMesh mesh) {

		int minX = toGridCoord(mesh.min.x);
		int minY = toGridCoord(mesh.min.y);
		int minZ = toGridCoord(mesh.min.z);

		int maxX = toGridCoord(mesh.max.x);
		int maxY = toGridCoord(mesh.max.y);
		int maxZ = toGridCoord(mesh.max.z);

		start.set(minX, minY, minZ);
		end.set(maxX, maxY, maxZ);
		range.setFrom(start, end);
	}
	
	private void getCoordRange(AABB box) {

		int minX = toGridCoord(box.min.x);
		int minY = toGridCoord(box.min.y);
		int minZ = toGridCoord(box.min.z);

		int maxX = toGridCoord(box.max.x);
		int maxY = toGridCoord(box.max.y);
		int maxZ = toGridCoord(box.max.z);

		start.set(minX, minY, minZ);
		end.set(maxX, maxY, maxZ);
		range.setFrom(start, end);
	}

	/**
	 * Calcule la position dans la grille du point. Stocke le résultat dans dest
	 * 
	 * @param position
	 * @param dest
	 */
	private void getCoord(Vector3f position, Coord dest) {
		int x = toGridCoord(position.x);
		int y = toGridCoord(position.y);
		int z = toGridCoord(position.z);
		dest.set(x, y, z);
	}

	private int toGridCoord(float pos) {
		return (int) (pos * ONE_OVER_GRID_CELL_SIZE + 0.5f * Math.signum(pos));
	}

}
