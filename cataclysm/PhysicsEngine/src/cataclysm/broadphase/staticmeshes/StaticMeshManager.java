package cataclysm.broadphase.staticmeshes;

import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import cataclysm.DefaultParameters;
import cataclysm.GeometryQuery;
import cataclysm.PhysicsWorld;
import cataclysm.RayTest;
import cataclysm.broadphase.AABB;
import cataclysm.datastructures.BufferedManager;
import cataclysm.parallel.PhysicsWorkerPool;
import cataclysm.record.StaticMeshRepr;
import cataclysm.wrappers.Wrapper;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Repr�sente une structure de donn�es contenant tous les StaticMesh.
 * 
 * @author Briac
 *
 */
public final class StaticMeshManager extends BufferedManager<StaticMesh> implements GeometryQuery{

	private final MapGrid mapGrid;
	private final PhysicsWorld world;
	private final DefaultParameters params;

	private Consumer<StaticMesh> callbackOnAdd;
	private Consumer<StaticMesh> callbackOnRemove;

	public StaticMeshManager(PhysicsWorld world) {
		this.world = world;
		this.params = world.getParameters();
		this.mapGrid = new MapGrid(params.getGridCellSize(), params.getMaxOctreeDepth());
	}

	public StaticMeshManager(DefaultParameters params) {
		this.world = null;
		this.params = params;
		this.mapGrid = new MapGrid(params.getGridCellSize(), params.getMaxOctreeDepth());
	}

	/**
	 * Construit un nouveau maillage de collision statique.
	 * 
	 * @param data      Les donn�es g�om�triques du maillage.
	 * @param transform Une transformation � appliquer aux donn�es g�om�triques pour
	 *                  construire les triangles du maillage.
	 * @param keepData  Indique s'il faut conserver les donn�es g�om�triques. Ceci
	 *                  permet de dupliquer le maillage par la suite. Laisser �
	 *                  false si le maillage statique ne sera jamais dupliqu�.
	 * @return
	 */
	public StaticMesh newMesh(StaticMeshData data, Matrix4f transform, boolean keepData) {
		StaticMesh mesh = new StaticMesh(data, transform, params, keepData, nextID());
		addElement(mesh);
		return mesh;
	}

	/**
	 * Construit un nouveau maillage de collision statique � partir des donn�es
	 * g�om�triques initiales d'un maillage existant.
	 * 
	 * @param transform Une transformation � appliquer aux donn�es g�om�triques pour
	 *                  construire les triangles du maillage.
	 * @param other     Un maillage existant ayant conserv� ses donn�es
	 *                  g�om�triques.
	 * @param keepData  Indique s'il faut conserver les donn�es g�om�triques. Ceci
	 *                  permet de dupliquer le maillage par la suite. Laisser �
	 *                  false si le maillage statique ne sera jamais dupliqu�.
	 * @return
	 */
	public StaticMesh copyMesh(Matrix4f transform, StaticMesh other, boolean keepData) {
		StaticMesh mesh = new StaticMesh(transform, other, keepData, nextID());
		addElement(mesh);
		return mesh;
	}

	public StaticMesh newMesh(StaticMeshRepr repr) {
		StaticMesh mesh = new StaticMesh(repr, nextID());
		addElement(mesh);
		return mesh;
	}

	@Override
	protected void internalUpdate() {

	}

	@Override
	protected void processAddedAndRemovedElements(List<StaticMesh> added, List<StaticMesh> removed) {
		if (callbackOnRemove != null)
			removed.forEach(callbackOnRemove);
		if (callbackOnAdd != null)
			added.forEach(callbackOnAdd);

		if(removed.size() > 0) {
			long t1 = System.currentTimeMillis();
			for (StaticMesh mesh : removed) {
				
				mesh.getBodyContacts().forEach(c -> c.getWrapper().getMeshContacts().remove(c));
				
				mapGrid.remove(mesh);
			}
			long t2 = System.currentTimeMillis();
			System.out.println("Removed " + removed.size() + " meshes in " + (t2 - t1) + " ms");
		}


		if(added.size() > 0) {
			long t1 = System.currentTimeMillis();
			for (StaticMesh mesh : added) {
				mapGrid.add(mesh);
			}
			long t2 = System.currentTimeMillis();
			System.out.println("Added " + added.size() + " meshes in " + (t2 - t1) + " ms");
		}

		if (world != null && world.getActiveRecord() != null) {
			world.getUpdateStats().physicsRecorder.start();
			world.getActiveRecord().getCurrentFrame().fillMeshes(added, removed);
			world.getUpdateStats().physicsRecorder.pause();
		}
	}

	@Override
	public void cleanUp() {
		super.cleanUp();
		mapGrid.cleanUp();
	}

	/**
	 * Explore l'octree et ajoute une boite affichable dans la liste boxes � chaque
	 * cellule rencontr�e.
	 * 
	 * @param boxes        La liste dans laquelle ranger les boites affichables.
	 * @param maxDepth     La profondeur maximale d'exploration.
	 * @param leavesOnly   N'ajoute que les cellules de profondeur maxDepth.
	 * @param nonVoidBoxes Ignore les cellules vides.
	 * @param position     La position en worldspace du d�but de l'exploration.
	 */
	public void exploreHierarchy(List<OctreeCellRenderable> boxes, int maxDepth, boolean leavesOnly,
			boolean nonVoidBoxes, Vector3f position) {
		mapGrid.exploreOctree(boxes, maxDepth, leavesOnly, nonVoidBoxes, position);
	}

	public Consumer<StaticMesh> getCallbackOnAdd() {
		return callbackOnAdd;
	}

	public void setCallbackOnAdd(Consumer<StaticMesh> callbackOnAdd) {
		this.callbackOnAdd = callbackOnAdd;
	}

	public Consumer<StaticMesh> getCallbackOnRemove() {
		return callbackOnRemove;
	}

	public void setCallbackOnRemove(Consumer<StaticMesh> callbackOnRemove) {
		this.callbackOnRemove = callbackOnRemove;
	}

	@Override
	protected void processAddedAndRemovedElements(List<StaticMesh> added, List<StaticMesh> removed,
			PhysicsWorkerPool workers) {
		processAddedAndRemovedElements(added, removed);
	}

	@Override
	protected void internalUpdate(PhysicsWorkerPool workers) {
		internalUpdate();
	}

	@Override
	public void rayTest(RayTest test) {
		mapGrid.rayTest(test);
	}

	@Override
	public void boxTriangleQuery(AABB box, Set<Triangle> set) {
		mapGrid.boxTriangleQuery(box, set);
	}
	
	@Override
	public void boxWrapperQuery(AABB box, Set<Wrapper> set) {
		throw new IllegalStateException("Not applicable");	
	}

}
