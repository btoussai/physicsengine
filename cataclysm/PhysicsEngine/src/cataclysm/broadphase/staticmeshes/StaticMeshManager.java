package cataclysm.broadphase.staticmeshes;

import java.util.HashSet;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.DefaultParameters;
import cataclysm.broadphase.AABB;
import cataclysm.datastructures.BufferedManager;

/**
 * Représente une structure de données contenant tous les StaticMesh.
 * @author Briac
 *
 */
public final class StaticMeshManager extends BufferedManager<StaticMesh>{
	
	private final MapGrid mapGrid;
	private final DefaultParameters params;
	
	public StaticMeshManager(DefaultParameters params){
		mapGrid = new MapGrid(params.getGridCellSize(), params.getMaxOctreeDepth());
		this.params = params;
	}
	
	/**
	 * Construit un nouveau maillage de collision statique.
	 * 
	 * @param data      Les données géométriques du maillage.
	 * @param transform Une transformation à appliquer aux données géométriques pour
	 *                  construire les triangles du maillage.
	 * @param keepData  Indique s'il faut conserver les données géométriques. Ceci
	 *                  permet de dupliquer le maillage par la suite. Laisser à
	 *                  false si le maillage statique ne sera jamais dupliqué.
	 * @return 
	 */
	public StaticMesh newMesh(StaticMeshData data, Matrix4f transform, boolean keepData) {
		StaticMesh mesh = new StaticMesh(data, transform, params, keepData, nextID());
		addElement(mesh);
		return mesh;
	}
	
	/**
	 * Construit un nouveau maillage de collision statique à partir des données
	 * géométriques initiales d'un maillage existant.
	 * 
	 * @param transform Une transformation à appliquer aux données géométriques pour
	 *                  construire les triangles du maillage.
	 * @param other     Un maillage existant ayant conservé ses données
	 *                  géométriques.
	 * @param keepData  Indique s'il faut conserver les données géométriques. Ceci
	 *                  permet de dupliquer le maillage par la suite. Laisser à
	 *                  false si le maillage statique ne sera jamais dupliqué.
	 * @return 
	 */
	public StaticMesh copyMesh(Matrix4f transform, StaticMesh other, boolean keepData)  {
		StaticMesh mesh = new StaticMesh(transform, other, keepData, nextID());
		addElement(mesh);
		return mesh;
	}
	
	/**
	 * Supprime le maillage statique possédant l'ID passé en paramètre.
	 * @param ID
	 * @return true si le mesh a bien été supprimé.
	 */
	public boolean deleteMesh(int ID) {
		return removeElement(ID);
	}
	
	/**
	 * Récupère l'ensemble des triangles en intersection avec l'AABB.
	 * 
	 * @param box
	 * @param dest
	 */
	public void boxTest(AABB box, HashSet<Triangle> dest) {
		mapGrid.boxTest(box, dest);
	}
	
	/**
	 * 
	 * Calcule la distance entre start et le premier triangle touché par le rayon ray.
	 * 
	 * @param start Le point de départ du rayon.
	 * @param dir La direction du rayon, le vecteur doit être unitaire.
	 * @param maxLength La distance maximale de recherche.
	 * @param backfaceCulling Les triangles ne faisant pas face au rayon seront ignorés si true.
	 * @param normalDest La normale du triangle touché sera stockée dedans.
	 * @return la distance du premier triangle touché ou maxLength si aucun triangle n'a été trouvé.
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
	}
	
	@Override
	public void cleanUp() {
		super.cleanUp();
		mapGrid.cleanUp();
	}
	
	
	/**
	 * Explore l'octree et ajoute une boite affichable dans la liste boxes à chaque cellule rencontrée.
	 * @param boxes La liste dans laquelle ranger les boites affichables.
	 * @param maxDepth La profondeur maximale d'exploration.
	 * @param leavesOnly N'ajoute que les cellules de profondeur maxDepth.
	 * @param nonVoidBoxes Ignore les cellules vides.
	 * @param position La position en worldspace du début de l'exploration.
	 */
	public void exploreHierarchy(List<OctreeCellRenderable> boxes, int maxDepth, boolean leavesOnly, boolean nonVoidBoxes, Vector3f position) {
		mapGrid.exploreOctree(boxes, maxDepth, leavesOnly, nonVoidBoxes, position);
	}
	
	

}
