package cataclysm;

/**
 * Cette classe contient toutes les constantes utilisées dans les algorithmes de
 * la simulation. La plupart sont déclarées {@code static final} et ne devraient
 * pas être modifiées. Certaines sont cependant modifiables directement pour
 * permettre de moduler la précision dans la résolutions des contraintes, au
 * prix de la performance.
 * 
 * @author Briac
 *
 */
public class Epsilons {

	/**
	 * Contient les constantes relatives à la mise au repos des rigidbody.
	 * 
	 * @author Briac
	 *
	 */
	public static class Sleep {

		/**
		 * Indique si les rigidbody peuvent être considérés comme totalement immobiles
		 * lorsque leur déplacement devient négligeable. Un rigidbody en sommeil ne
		 * subira plus aucune force extérieure. Un changement de la vitesse réveillera
		 * le rigidbody.
		 */
		public static final boolean SLEEPING_ALLOWED = true;

		/**
		 * La vitesse de rotation minimale pour laquelle on considère la rotation
		 * négligeable.
		 */
		public static final float MIN_ROTATION_SPEED = 2.0f * (float) Math.PI / 20.0f;// 10 seconds per rotation

		/**
		 * La vitesse de translation minimale pour laquelle on considère le déplacement
		 * négligeable.
		 */
		public static final float MIN_TRANSLATION_SPEED = 0.1f;

		/**
		 * Si la vitesse de translation et de rotation sont considérées comme
		 * négligeables pendant un nombre de frame consécutives égales à cette valeur,
		 * le rigidbody est mis en sommeil.
		 */
		public static final int FRAMES_SPENT_AT_REST = 5;
	}

	/**
	 * Si true, les impulsions précédentes sont réutilisées comme valeur initiale
	 * pour la résolution des contraintes.
	 */
	public static boolean WARM_START = false;

	public static ContactSolver solver = ContactSolver.SEQUENTIAL_IMPULSE;

	public static ContactType contactType = ContactType.SIMPLE;

	public enum ContactType {
		SIMPLE, ARRAY_BASED;
	}

	public enum ContactSolver {
		SEQUENTIAL_IMPULSE, PARALLEL_IMPULSE_2, PARALLEL_IMPULSE_3, PARALLEL_IMPULSE_4, PARALLEL_IMPULSE_5,
		PARALLEL_IMPULSE_6, PARALLEL_IMPULSE_7, PARALLEL_IMPULSE_8;
	}

	// ############### SUPER STATIC CONSTS BELOW, SHOULDN'T CHANGE ###############

	/**
	 * Le nombre maximal de contacts entre deux enveloppes convexes.
	 */
	public static final int MAX_CONTACTS = 4;

	/**
	 * La valeur des impulsions cumulées en dessous de laquelle on considère qu'il y
	 * a eu convergence.
	 */
	public static final float MIN_IMPULSE = 1E-5f;

	/**
	 * La vitesse minimale à partir de laquelle on considère qu'il n'y a pas de
	 * vitesse tangente. Le vecteur tangent est mis à 0.
	 */
	public static final float MIN_TANGENT_SPEED = 1E-3f;

	/**
	 * La fraction de la p�n�tration corrig�e � chaque frame.
	 */
	public static final float PENETRATION_RECOVERY = 0.2f;

	/**
	 * Repr�sente la p�n�tration maximale autoris�e. Ceci permet d'augmenter la
	 * stabilit�.
	 */
	public static final float ALLOWED_PENETRATION = 0.01f;

	/**
	 * La longueur minimale d'un vecteur lors d'une tentative de normalisation. Un
	 * vecteur de module inf�rieur est consid�r� comme valant z�ro.
	 */
	public static final float MIN_LENGTH = 1E-4f;

	/**
	 * MIN_LENGTH, au carr�.
	 */
	public static final float MIN_LENGTH_2 = MIN_LENGTH * MIN_LENGTH;

	/**
	 * La vitesse en dessous de laqulle les chocs ont une �lasticit� nulle. Cela
	 * permet de stabiliser les objets presque � l'arr�t.
	 */
	public static final float VELOCITY_ELASTICITY_LIMIT = 0.5f;

	/**
	 * L'angle considéré comme minimal
	 */
	private static final float MIN_ANGLE = (float) Math.toRadians(1.0);// 1 degree

	/**
	 * Si la valeur absolue du produit scalaire entre deux vecteurs unitaires est
	 * inf�rieure � cette limite, les vecteurs sont consid�r�s perpendiculaires.
	 */
	public static final float ORTHOGONAL_LIMIT = (float) Math.sin(MIN_ANGLE);

	/**
	 * ORTHOGONAL_LIMIT, au carré.
	 */
	public static final float ORTHOGONAL_LIMIT_2 = ORTHOGONAL_LIMIT * ORTHOGONAL_LIMIT;

	/**
	 * Si la valeur absolue du produit scalaire entre deux vecteurs unitaires est
	 * sup�rieure � cette limite, les vecteurs sont consid�r�s parall�les.
	 */
	public static final float PARALLEL_LIMIT = (float) Math.cos(MIN_ANGLE);

	/**
	 * PARALLEL_LIMIT, au carré.
	 */
	public static final float PARALLEL_LIMIT_2 = PARALLEL_LIMIT * PARALLEL_LIMIT;

}
