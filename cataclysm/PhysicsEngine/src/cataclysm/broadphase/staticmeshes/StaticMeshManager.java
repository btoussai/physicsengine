package cataclysm.broadphase.staticmeshes;

import java.util.HashSet;
import java.util.List;

import cataclysm.DefaultParameters;
import cataclysm.PhysicsWorld;
import cataclysm.broadphase.AABB;
import cataclysm.datastructures.BufferedManager;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Repr�sente une structure de donn�es contenant tous les StaticMesh.
 * @author Briac
 *
 */
public final class StaticMeshManager extends BufferedManager<StaticMesh>{
	
	private final MapGrid mapGrid;
	private final PhysicsWorld world;
	private final DefaultParameters params;
	
	public StaticMeshManager(PhysicsWorld world){
		this.world = world;
		this.params = world.getParameters();
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
	public StaticMesh copyMesh(Matrix4f transform, StaticMesh other, boolean keepData)  {
		StaticMesh mesh = new StaticMesh(transform, other, keepData, nextID());
		addElement(mesh);
		return mesh;
	}
	
	/**
	 * Supprime un maillage statique
	 * @param mesh 
	 * @return true si le mesh a bien été supprimé.
	 */
	public boolean deleteMesh(StaticMesh mesh) {
		return removeElement(mesh);
	}
	
	/**
	 * R�cup�re l'ensemble des triangles en intersection avec l'AABB.
	 * 
	 * @param box
	 * @param dest
	 */
	public void boxTest(AABB box, HashSet<Triangle> dest) {
		mapGrid.boxTest(box, dest);
	}
	
	/**
	 * 
	 * Calcule la distance entre start et le premier triangle touch� par le rayon ray.
	 * 
	 * @param start Le point de d�part du rayon.
	 * @param dir La direction du rayon, le vecteur doit �tre unitaire.
	 * @param maxLength La distance maximale de recherche.
	 * @param backfaceCulling Les triangles ne faisant pas face au rayon seront ignor�s si true.
	 * @param normalDest La normale du triangle touch� sera stock�e dedans.
	 * @return la distance du premier triangle touch� ou maxLength si aucun triangle n'a �t� trouv�.
	 */
	public float rayTest(Vector3f start, Vector3f dir, float maxLength, boolean backfaceCulling, Vector3f normalDest) {
		return mapGrid.rayTest(start, dir, maxLength, backfaceCulling, normalDest);
	}

	@Override
	protected void internalUpdate() {
		
	}

	@Override
	protected void processAddedAndRemovedElements(List<StaticMesh> added, List<StaticMesh> removed) {
		for(StaticMesh mesh : added) {
			mapGrid.add(mesh);
		}
		for(StaticMesh mesh : removed) {
			mapGrid.remove(mesh);
		}
		
		if(world.getActiveRecord() != null) {
			world.getActiveRecord().getCurrentFrame().fillMeshes(added, removed);
		}
	}
	
	@Override
	public void cleanUp() {
		super.cleanUp();
		mapGrid.cleanUp();
	}
	
	
	/**
	 * Explore l'octree et ajoute une boite affichable dans la liste boxes � chaque cellule rencontr�e.
	 * @param boxes La liste dans laquelle ranger les boites affichables.
	 * @param maxDepth La profondeur maximale d'exploration.
	 * @param leavesOnly N'ajoute que les cellules de profondeur maxDepth.
	 * @param nonVoidBoxes Ignore les cellules vides.
	 * @param position La position en worldspace du d�but de l'exploration.
	 */
	public void exploreHierarchy(List<OctreeCellRenderable> boxes, int maxDepth, boolean leavesOnly, boolean nonVoidBoxes, Vector3f position) {
		mapGrid.exploreOctree(boxes, maxDepth, leavesOnly, nonVoidBoxes, position);
	}
	
	

}
