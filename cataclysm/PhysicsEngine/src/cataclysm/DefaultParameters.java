package cataclysm;

import cataclysm.broadphase.AABB;
import cataclysm.contact_creation.ContactProperties;
import cataclysm.integrators.ExternalForceIntegrator;
import cataclysm.integrators.VerticalGravityIntegrator;
import cataclysm.wrappers.Wrapper;

/**
 * Repr�sente les param�tres par d�faut utilis�s dans la simulation.
 * 
 * @author Briac
 *
 */
public class DefaultParameters {

	/**
	 * Le pas d'intégration.
	 */
	private float timeStep;
	

	/**
	 * Le nombre maximal d'itérations utilisées lors de la résolution des
	 * contraintes et des contacts pour la vitesse.
	 */
	private int maxIterationVelocity;

	/**
	 * Le nombre maximal d'itérations utilisées lors de la résolution des
	 * contraintes et des contacts pour la position.
	 */
	private int maxIterationsPosition;

	/**
	 * La taille d'une cellule de base dans l'octree.
	 */
	private final float gridCellSize;

	/**
	 * Le nombre de subdivisions utilisées dans l'octree.
	 */
	private final int maxOctreeDepth;

	/**
	 * Les forces externes appliquées aux objets.
	 */
	private final ExternalForceIntegrator forceIntegrator;

	/**
	 * Les fonctions appelées par défaut aux différentes étapes de la résolution des
	 * contacts.
	 */
	private final CataclysmCallbacks callbacks;

	/**
	 * Permet de d�terminer si deux rigidbody doivent ou non enter en collision.
	 */
	private final CollisionFilter collisionFilter;

	/**
	 * Indique s'il faut int�grer le terme gyroscopique dans les �quations de la
	 * rotation. <br>
	 * Si false, on utilise la formule suivante: <br>
	 * Ib * dw/dt = 0 <br>
	 * Si true, on utilise la formule compl�te: <br>
	 * Ib * dw/dt + w x Ib * w = 0 <br>
	 * <br>
	 * Avec Ib le tenseur d'inertie de l'objet et w sa vitesse angulaire.
	 */
	private boolean gyroscopic = false;

	/**
	 * true si la gravité est activée.
	 */
	private boolean gravity = true;
	
	/**
	 * Une marge ajoutée à la taille de la {@link WrapperBox} des {@link Wrapper}.
	 */
	private final float padding;
	
	/**
	 * La friction et l'élasticité par défaut pour un objet.
	 */
	private final ContactProperties contactProperties;

	public DefaultParameters() {
		this.timeStep = 1.0f / 60.0f;
		this.maxIterationsPosition = 2;
		this.maxIterationVelocity = 8;
		this.padding = (timeStep * 10) * 2.0f;
		this.gridCellSize = 32.0f;
		this.maxOctreeDepth = 5;
		this.forceIntegrator = new VerticalGravityIntegrator();
		this.callbacks = new CataclysmCallbacks();
		this.collisionFilter = new DefaultCollisionFilter();
		this.contactProperties = new ContactProperties(0.2f, 0.3f);
	}

	public DefaultParameters(float timeStep, int maxIterationsPosition, int maxIterationVelocity, float gridCellSize, int maxOctreeDepth,
			ExternalForceIntegrator forceIntegrator, CataclysmCallbacks callbacks, CollisionFilter filter, ContactProperties contactProperties) {
		this.maxIterationsPosition = maxIterationsPosition;
		this.maxIterationVelocity = maxIterationVelocity;
		this.timeStep = timeStep;
		this.padding = (timeStep * 10) * 2;
		this.gridCellSize = gridCellSize;
		this.maxOctreeDepth = maxOctreeDepth;
		this.forceIntegrator = forceIntegrator;
		this.callbacks = callbacks;
		this.collisionFilter = filter;
		this.contactProperties = contactProperties;
	}

	public boolean isGravity() {
		return gravity;
	}

	public void setGravity(boolean gravity) {
		this.gravity = gravity;
	}
	
	public void setGravityStrength(float gravityStrength) {
		this.forceIntegrator.setGravityStrength(gravityStrength);
	}

	public float getTimeStep() {
		return timeStep;
	}

	public void setTimeStep(float timeStep) {
		this.timeStep = timeStep;
	}

	public int getMaxIterationVelocity() {
		return maxIterationVelocity;
	}

	public void setMaxIterationVelocity(int maxIterationVelocity) {
		this.maxIterationVelocity = maxIterationVelocity;
	}

	public int getMaxIterationsPosition() {
		return maxIterationsPosition;
	}

	public void setMaxIterationsPosition(int maxIterationsPosition) {
		this.maxIterationsPosition = maxIterationsPosition;
	}
	
	public void setStepParams(float timeStep, int maxIterationsPosition, int maxIterationVelocity) {
		this.timeStep = timeStep;
		this.maxIterationsPosition = maxIterationsPosition;
		this.maxIterationVelocity = maxIterationVelocity;
	}

	/**
	 * Une marge ajoutée à la taille de la {@link AABB} des {@link Wrapper}.
	 * 
	 * @return
	 */
	public float getPadding() {
		return padding;
	}

	public float getGridCellSize() {
		return gridCellSize;
	}

	public int getMaxOctreeDepth() {
		return maxOctreeDepth;
	}

	public boolean useGyroscopicIntegration() {
		return gyroscopic;
	}

	public void setGyroscopicIntegration(boolean gyroscopic) {
		this.gyroscopic = gyroscopic;
	}

	public ExternalForceIntegrator getForceIntegrator() {
		return forceIntegrator;
	}

	public CataclysmCallbacks getCallbacks() {
		return callbacks;
	}

	public CollisionFilter getCollisionFilter() {
		return collisionFilter;
	}

	public ContactProperties getContactProperties() {
		return contactProperties;
	}

}
