package cataclysm.broadphase.staticmeshes;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import cataclysm.GeometryQuery;
import cataclysm.RayTest;
import cataclysm.broadphase.AABB;
import cataclysm.wrappers.Wrapper;
import math.vector.Vector2f;
import math.vector.Vector3f;

/**
 * Repr�sente la grille 3D dans laquel sont stock�s les triangles des
 * physicsmesh.
 * 
 * @author Briac
 *
 */
class MapGrid implements GeometryQuery{

	private final float GRID_CELL_SIZE;
	private final int maxOctreeDepth;
	private final float ONE_OVER_GRID_CELL_SIZE;
	private final Map<Coord, OctreeBase> grid = new HashMap<Coord, OctreeBase>();

	private final Vector3f min = new Vector3f();
	private final Vector3f max = new Vector3f();
	private final Vector3f axis = new Vector3f();
	private final Vector3f[] edges = new Vector3f[3];

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
		CoordRange range = new CoordRange();
		
		getCoordRange(mesh, range);
		range.startIteration();
		while (range.next()) {
			OctreeBase base = grid.get(range.getIterator());
			if (base == null) {
				base = new OctreeBase(GRID_CELL_SIZE, range.getIterator(), maxOctreeDepth);
				grid.put(base.coord, base);
			}

			for (Triangle triangle : mesh.triangles) {
				min.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
				max.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
				triangle.getMinMax(min, max);

				triangle.getEdge0(edges[0]);
				triangle.getEdge1(edges[1]);
				triangle.getEdge2(edges[2]);
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
		CoordRange range = new CoordRange();
		getCoordRange(mesh, range);
		range.startIteration();
		while (range.next()) {
			OctreeBase base = grid.get(range.getIterator());
			if (base == null) {
				continue;
			}

			for (Triangle triangle : mesh.triangles) {
				min.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
				max.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
				triangle.getMinMax(min, max);

				triangle.getEdge0(edges[0]);
				triangle.getEdge1(edges[1]);
				triangle.getEdge2(edges[2]);
				base.rootCell.removeTriangle(triangle, min, max, edges, axis);

			}

		}

	}

	/**
	 * Performs a ray test against static meshes.
	 * @param test
	 */
	@Override
	public void rayTest(RayTest test) {
		
		Vector2f intersectionTime = new Vector2f();
		Coord iterator =  new Coord(0, 0, 0);
		getCoord(test.getStart(), iterator);
		OctreeBase.computeIntersectionTime(test.getStart(), test.getDir(), iterator, GRID_CELL_SIZE, intersectionTime);
		while (true) {
			//System.out.println(iterator);
			OctreeBase base = grid.get(iterator);
			if (base != null) {
				float length = base.rootCell.rayTest(test.getStart(), test.getDir(), test.getMaxDistance(), intersectionTime.x, intersectionTime.y, test.isBackfaceCulling(), test.getHitNormal());
				if (length < test.getMaxDistance()) {
					test.setHitDistance(length);
					return;
				}
			}
			
			if(!OctreeBase.nextCell(test.getStart(), test.getDir(), test.getMaxDistance(), this.GRID_CELL_SIZE, iterator, iterator, intersectionTime)) {
				break;
			}
			
		}
	}
	
	@Override
	public void boxTriangleQuery(AABB box, Set<Triangle> set) {
		CoordRange range = new CoordRange();
		getCoordRange(box, range);
		range.startIteration();
		while (range.next()) {
			OctreeBase base = grid.get(range.getIterator());
			if (base != null) {
				base.rootCell.boxTest(box, set);
			}
		}
	}
	
	@Override
	public void boxWrapperQuery(AABB box, Set<Wrapper> set) {
		throw new IllegalStateException("Not applicable");	
	}

	void cleanUp() {
		grid.clear();
	}

	/**
	 * Explore l'octree et ajoute une boite affichable dans la liste boxes � chaque
	 * cellule rencontrée.
	 * 
	 * @param boxes        La liste dans laquelle ranger les boites affichables.
	 * @param maxDepth     La profondeur maximale d'exploration.
	 * @param leavesOnly   N'ajoute que les cellules de profondeur maxDepth.
	 * @param nonVoidBoxes Ignore les cellules vides.
	 * @param position     La position en worldspace du d�but de l'exploration.
	 */
	void exploreOctree(List<OctreeCellRenderable> boxes, int maxDepth, boolean leavesOnly, boolean nonVoidBoxes,
			Vector3f position) {
		Coord coord = new Coord(0, 0, 0);
		getCoord(position, coord);
		OctreeBase base = grid.get(coord);
		if (base != null) {
			base.rootCell.exploreHierarchy(boxes, maxDepth, leavesOnly, nonVoidBoxes);
		}
	}

	private void getCoordRange(StaticMesh mesh, CoordRange range) {

		int minX = toGridCoord(mesh.min.x);
		int minY = toGridCoord(mesh.min.y);
		int minZ = toGridCoord(mesh.min.z);

		int maxX = toGridCoord(mesh.max.x);
		int maxY = toGridCoord(mesh.max.y);
		int maxZ = toGridCoord(mesh.max.z);

		range.setStart(minX, minY, minZ);
		range.setStop(maxX, maxY, maxZ);
	}
	
	private void getCoordRange(AABB box, CoordRange range) {

		int minX = toGridCoord(box.minX);
		int minY = toGridCoord(box.minY);
		int minZ = toGridCoord(box.minZ);

		int maxX = toGridCoord(box.maxX);
		int maxY = toGridCoord(box.maxY);
		int maxZ = toGridCoord(box.maxZ);

		range.setStart(minX, minY, minZ);
		range.setStop(maxX, maxY, maxZ);
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
