package cataclysm;

/**
 * Représente une fonction appelée à chaque update du moteur physique. Permet de
 * contrôler un objet dans la simulation (ex: un moteur).
 * 
 * @author Briac
 *
 */
@FunctionalInterface
public interface Actor {

	/**
	 * Appelle la fonction de mise à jour.
	 * 
	 * @param simu La simulation.
	 * @return un boolean indiquant si l'acteur doit être conservé. Si false, l'acteur est supprimé.
	 */
	boolean update(PhysicsWorld simu);

}
