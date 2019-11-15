package cataclysm.datastructures;

import java.util.ArrayList;
import java.util.List;

/**
 * Représente un manager d'objet avec des tampons permettant de garder la trace
 * des éléments ajoutés ou supprimés entre deux mises à jour du manager.
 * 
 * @author Briac
 *
 * @param <T>
 */
public abstract class BufferedManager<T extends Identifier> extends Manager<T> {

	/**
	 * La liste des éléments ayant été ajoutés à la liste actuelle.
	 */
	private List<T> added = new ArrayList<T>();

	/**
	 * La liste des éléments ayant été supprimés de la liste actuelle.
	 */
	private List<T> removed = new ArrayList<T>();

	/**
	 * Construit un nouveau manager avec des tampons.
	 */
	public BufferedManager() {

	}

	/**
	 * Applique un traitement particulier aux éléments devant être ajoutés et aux
	 * éléments devant être retirés. La fonction {@link #internalUpdate()} est
	 * appelée immédiatement après.
	 * 
	 * @param added   La liste des objets qui doivent être ajoutés. La liste est
	 *                vidée automatiquement au retour de la fonction. Les objets ne
	 *                sont ajoutés dans le manager qu'au retour de la fonction,
	 *                avant l'appel à {@link #internalUpdate()}
	 * 
	 * @param removed La liste des objets qui doivent être supprimés. La liste est
	 *                vidée automatiquement au retour de la fonction. Les IDs des
	 *                objets supprimés redeviennent disponibles au retour de la
	 *                fonction.
	 */
	protected abstract void processAddedAndRemovedElements(List<T> added, List<T> removed);

	@Override
	public void update() {
		processAddedAndRemovedElements(added, removed);
		removed.forEach(e -> {
			generator.freeID(e.getID());
			super.removeElement(e);
		});
		added.forEach(e -> super.addElement(e));
		
		added.clear();
		removed.clear();
		
		super.update();
	}

	@Override
	public void addElement(T element) {
		added.add(element);
	}

	@Override
	public boolean removeElement(T element) {
		if (element != null && contains(element)) {
			removed.add(element);
			return true;
		}
		return false;
	}
	
	@Override
	public boolean contains(T element) {
		return super.contains(element) &&  !removed.contains(element);
	}

	@Override
	public void cleanUp() {
		super.cleanUp();
		added.clear();
		removed.clear();
	}

}
