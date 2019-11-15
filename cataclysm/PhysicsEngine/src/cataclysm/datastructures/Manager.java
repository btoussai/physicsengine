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
	 * élément doit avoir été réservé impérativement par un appel à
	 * {@link #nextID()}
	 * 
	 * @param element L'élément à ajouter.
	 */
	public void addElement(T element) {
		elements.put(element.getID(), element);
	}

	/**
	 * Essaie de supprimer l'élément.
	 * 
	 * @param element L'objet à supprimer.
	 * @return true si l'élément a bien été supprimé.
	 */
	public boolean removeElement(T element) {
		if (elements.remove(element.getID()) != null) {
			generator.freeID(element.getID());
			return true;
		}
		return false;
	}

	/**
	 * Teste la présence d'un élément.
	 * 
	 * @param element
	 * @return true s'il existe un element e tel que <br>
	 *         {@code e.getID()==element.getID()}
	 */
	public boolean contains(T element) {
		return elements.containsKey(element.getID());
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
