package cataclysm.datastructures;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Repr�sente un manager d'objets.
 * 
 * @author Briac
 *
 * @param <T>
 */
public abstract class Manager<T extends Identifier> implements Iterable<T> {

	/**
	 * Le générateur d'ID permettant d'instancier les objets gérés par le manager.
	 */
	protected IDGenerator generator = new IDGenerator();

	/**
	 * Les objets gérés par le manager.
	 */
	protected Map<Long, T> elements = new HashMap<Long, T>();

	/**
	 * Construit un nouveau manager d'objets.
	 */
	public Manager() {
	}

	/**
	 * Met à jour les éléments du manager.
	 */
	public void update() {
		internalUpdate();
	}

	/**
	 * La mise à jour customizable des éléments.
	 */
	protected abstract void internalUpdate();

	/**
	 * @return Le nombre d'objets dans le manager.
	 */
	public int size() {
		return elements.size();
	}

	/**
	 * @return Le prochain identifiant libre. Cet identifiant est compté comme
	 *         utilisé dès sa génération.
	 */
	public long nextID() {
		return generator.nextID();
	}

	/**
	 * Ajoute un élément à la liste des objets gérés par le manager. L'id de cet
	 * élément doit avoir été réservé impérativement par un appel à {@link #nextID()}
	 * 
	 * @param element L'élément à ajouter.
	 */
	public void addElement(T element) {
		elements.put(element.getID(), element);
	}

	/**
	 * Essaie de supprimer l'élément ayant l'ID indiqué en paramètre.
	 * 
	 * @param ID L'identifiant de l'objet à supprimer.
	 * @return true si l'élément a bien été supprimé.
	 */
	public boolean removeElement(long ID) {
		if (elements.remove(ID) != null) {
			generator.freeID(ID);
			return true;
		}
		return false;
	}
	
	/**
	 * Essaie de supprimer l'élément ayant l'ID indiqué en paramètre. 
	 * @param ID L'identifiant de l'objet à supprimer.
	 * @return L'élément supprimé ou null.
	 */
	public T removeAndGet(long ID) {
		return elements.remove(ID);
	}

	/**
	 * Teste la présence d'un élément comportant l'ID indiquée.
	 * 
	 * @param ID
	 * @return true s'il existe un element e tel que <br>
	 *         {@code e.getID()==ID}
	 */
	public boolean contains(long ID) {
		return elements.containsKey(ID);
	}

	/**
	 * Supprime tous les éléments dans le manager et réinitialise le générateur
	 * d'ID.
	 */
	public void cleanUp() {
		elements.clear();
		generator.reset();
	}

	@Override
	public Iterator<T> iterator() {
		return elements.values().iterator();
	}

}
