package cataclysm.datastructures;

import java.util.ArrayList;
import java.util.List;

/**
 * Cette classe permet de générer les ID des entités de manière unique.
 * 
 * @author Briac
 *
 */
public final class IDGenerator {

	/**
	 * L'ID le plus grand ayant été généré.
	 */
	private long maxID = 0;

	/**
	 * La liste des ID réutilisables
	 */
	private List<Long> available = new ArrayList<Long>();

	/**
	 * Crée un générateur d'id. Les IDs générés sont
	 * uniques.
	 */
	public IDGenerator() {

	}

	/**
	 * @return Un nouvel ID unique.
	 */
	public long nextID() {
		if (available.isEmpty()) {
			maxID++;
			return maxID;
		} else {
			return available.remove(available.size() - 1);
		}
	}

	/**
	 * Libère un ID afin de le réutiliser lors de la prochaine génération d'ID.
	 * 
	 * @param ID
	 *            L'ID à libérer
	 *
	 * @throws IllegalArgumentException
	 *             Si ID < 0 ou ID > maxIDGenerated
	 */
	public void freeID(long ID) throws IllegalArgumentException {
		if (ID < 0 || ID > maxID) {
			throw new IllegalArgumentException("Invalid ID:" + ID + ", it may have already been freed.");
		}
		available.add(ID);
	}
	
	/**
	 * Réinitialise le générateur d'ID
	 */
	public void reset(){
		maxID = 0;
		available.clear();
	}

}
