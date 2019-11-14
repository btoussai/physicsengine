package cataclysm;

/**
 * Repr�sente une fonction appel�e � chaque update du moteur physique. Permet de
 * contr�ler un objet dans la simulation (ex: un moteur).
 * 
 * @author Briac
 *
 */
@FunctionalInterface
public interface Actor {

	/**
	 * Appelle la fonction de mise � jour.
	 * 
	 * @param simu La simulation.
	 * @return un boolean indiquant si l'acteur doit �tre conserv�. Si false, l'acteur est supprim�.
	 */
	boolean update(PhysicsWorld simu);

}
