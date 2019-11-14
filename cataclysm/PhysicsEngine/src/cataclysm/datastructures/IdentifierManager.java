package cataclysm.datastructures;

public abstract class IdentifierManager<T extends Identifier> {
	/**
	 * Le générateur d'ID permettant d'instancier les objets gérés par le manager.
	 */
	protected IDGenerator generator = new IDGenerator();

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
	public abstract int size();

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
	public abstract void addElement(T element);

	/**
	 * Essaie de supprimer l'élément ayant l'ID indiqué en paramètre.
	 * 
	 * @param ID L'identifiant de l'objet à supprimer.
	 * @return true si l'élément a bien été supprimé.
	 */
	public abstract boolean removeElement(long ID);
	
	/**
	 * Essaie de supprimer l'élément ayant l'ID indiqué en paramètre. 
	 * @param ID L'identifiant de l'objet à supprimer.
	 * @return L'élément supprimé ou null.
	 */
	public abstract T removeAndGet(long ID);

	/**
	 * Teste la présence d'un élément comportant l'ID indiquée.
	 * 
	 * @param ID
	 * @return true s'il existe un element e tel que <br>
	 *         {@code e.getID()==ID}
	 */
	public abstract boolean contains(long ID);

	/**
	 * Supprime tous les éléments dans le manager et réinitialise le générateur
	 * d'ID.
	 */
	public void cleanUp() {
		generator.reset();
	}
}
