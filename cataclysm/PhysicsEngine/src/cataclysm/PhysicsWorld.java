package cataclysm;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import cataclysm.broadphase.staticmeshes.StaticMesh;
import cataclysm.broadphase.staticmeshes.StaticMeshData;
import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.record.PhysicsPlayer;
import cataclysm.record.PhysicsRecorder;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;
import cataclysm.wrappers.WrapperBuilder;
import math.MatrixOps;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Repr�sente l'ensemble des objets sur lesquels de la physique est appliquée.
 * 
 * @author Briac
 *
 */
public class PhysicsWorld {

	/**
	 * Les paramètres de la simulation.
	 */
	private final DefaultParameters params;

	/**
	 * L'ensemble des corps mobiles.
	 */
	private final RigidBodyManager bodies;

	/**
	 * L'ensemble des maillages de collision statiques.
	 */
	private final StaticMeshManager meshes;

	/**
	 * La liste des contraintes.
	 */
	private final List<AbstractConstraint> constraints = new ArrayList<AbstractConstraint>();

	/**
	 * Le moteur physique permettant de simuler les contacts et les contraintes
	 * entre les corps.
	 */
	private final PhysicsEngine engine;

	/**
	 * Des statistiques sur la simulation.
	 */
	private final PhysicsStats stats = new PhysicsStats();

	/**
	 * Des fonctions de mise à jour définies par l'utilisateur.
	 */
	private final List<Actor> actors = new ArrayList<Actor>();

	/**
	 * A recorder currently recording the state of the world
	 */
	private PhysicsRecorder activeRecord;

	/**
	 * A list of the records currently being replayed
	 */
	private List<PhysicsPlayer> recordPlayers;

	/**
	 * Instancie un nouveau monde pour simuler de la physique.
	 * 
	 * @param params Les paramètres par défaut.
	 */
	public PhysicsWorld(DefaultParameters params) {
		this.params = params;
		meshes = new StaticMeshManager(this);
		bodies = new RigidBodyManager(this, meshes, stats);
		engine = new PhysicsEngine(this);
	}

	/**
	 * This method must be called only once to initialize the simulation.
	 */
	public void start() {
		if (activeRecord != null) {
			// record frame 0
			activeRecord.newFrame();
		}

		if (recordPlayers != null) {
			for (PhysicsPlayer player : recordPlayers) {
				player.step(meshes, bodies);
			}
		}

		meshes.update();
		bodies.update();
		if (activeRecord != null) {
			activeRecord.endOfFrame();
		}
	}

	/**
	 * Met à jour la simulation.
	 * 
	 * @param frameCount Le nombre de pas de temps à simuler.
	 */
	public void update(int frameCount) {
		if (frameCount < 1) {
			throw new IllegalArgumentException("Invalid number of frames to simulate: " + frameCount);
		}

		for (int i = 0; i < frameCount; i++) {
			stats.globalUpdate.start();
			actors.removeIf(actor -> !actor.update(this));
			stats.step(params.getTimeStep());
			if (activeRecord != null) {
				activeRecord.newFrame();
			}
			engine.update(bodies, meshes, constraints, stats);
			if (activeRecord != null) {
				activeRecord.endOfFrame();
			}

			stats.globalUpdate.stop();
		}

		System.out.println(stats);
	}

	/**
	 * 
	 * Calcule la distance entre start et le premier triangle touch� par le rayon
	 * ray.
	 * 
	 * @param start           Le point de d�part du rayon.
	 * @param dir             La direction du rayon, le vecteur doit �tre unitaire.
	 * @param maxLength       La distance maximale de recherche.
	 * @param backfaceCulling Les triangles ne faisant pas face au rayon seront
	 *                        ignor�s si true.
	 * @param normalDest      La normale du triangle touch� sera stock�e dedans.
	 * @return la distance du premier triangle touch� ou maxLength si aucun triangle
	 *         n'a �t� trouv�.
	 */
	public float rayTest(Vector3f start, Vector3f dir, float maxLength, boolean backfaceCulling, Vector3f normalDest) {
		return meshes.rayTest(start, dir, maxLength, backfaceCulling, normalDest);
	}

	/**
	 * @return Le temps �coul� dans la simulation.
	 */
	public double getElapsedTime() {
		return stats.getElapsedTime();
	}

	/**
	 * @return Le nombre de frames �coul�es dans la simulation.
	 */
	public long getElapsedFrames() {
		return stats.getFrameCount();
	}

	public PhysicsStats getUpdateStats() {
		return stats;
	}

	/**
	 * Ajoute un objet dans la simulation � la position sp�cifi�e. <br>
	 * Pour modifier �galement la rotation initiale de l'objet, il faut utiliser
	 * {@link #newBody(Matrix4f, WrapperBuilder...)}.
	 * 
	 * 
	 * @param position La position de l'objet.
	 * @param builders Les enveloppes de cet objet.
	 * @return L'objet nouvellement cr��.
	 */
	public RigidBody newBody(Vector3f position, WrapperBuilder... builders) {
		return bodies.newBody(MatrixOps.createTransformationMatrix(position, 0, 0, 0, 1, null), builders);
	}

	/**
	 * Ajoute un objet dans la simulation.
	 * 
	 * @param transform La position et la rotation de l'objet.
	 * @param builders  Les enveloppes de cet objet.
	 * @return L'objet nouvellement créé.
	 */
	public RigidBody newBody(Matrix4f transform, WrapperBuilder... builders) {
		return bodies.newBody(transform, builders);
	}

	/**
	 * Supprime un rigidbody. Toutes les contraintes ayant un point d'ancrage sur ce
	 * corps seront également supprimées à la prochaine update.
	 * 
	 * @param ID
	 * @return true si le corps a effectivement été supprimé.
	 */
	public boolean deleteBody(long ID) {
		return bodies.removeElement(ID);
	}

	/**
	 * Teste la présence d'un rigidbody dans la simulation.
	 * 
	 * @param ID
	 * @return
	 */
	public boolean containsBody(long ID) {
		return bodies.contains(ID);
	}

	/**
	 * Construit un nouveau maillage de collision statique. <br>
	 * Attention, le maillage ne sera plac� dans la simulation qu'apr�s un appel �
	 * {@link #start()} ou {@link #update(int)}
	 * 
	 * @param data      Les donn�es g�om�triques du maillage.
	 * @param transform Une transformation � appliquer aux donn�es g�om�triques pour
	 *                  construire les triangles du maillage.
	 * @param keepData  Indique s'il faut conserver les donn�es g�om�triques. Ceci
	 *                  permet de dupliquer le maillage par la suite. Laisser �
	 *                  false si le maillage statique ne sera jamais dupliqu�.
	 * @return
	 */
	public StaticMesh newMesh(StaticMeshData data, Matrix4f transform, boolean keepData) {
		return meshes.newMesh(data, transform, keepData);
	}

	/**
	 * Construit un nouveau maillage de collision statique � partir des donn�es
	 * g�om�triques initiales d'un maillage existant.
	 * 
	 * @param transform Une transformation � appliquer aux donn�es g�om�triques pour
	 *                  construire les triangles du maillage.
	 * @param other     Un maillage existant ayant conserv� ses donn�es
	 *                  g�om�triques.
	 * @param keepData  Indique s'il faut conserver les donn�es g�om�triques. Ceci
	 *                  permet de dupliquer le maillage par la suite. Laisser �
	 *                  false si le maillage statique ne sera jamais dupliqu�.
	 * @return
	 */
	public StaticMesh copyMesh(Matrix4f transform, StaticMesh other, boolean keepData) {
		return meshes.copyMesh(transform, other, keepData);
	}

	/**
	 * Supprime un staticmesh.
	 * 
	 * @param ID
	 * @return true si le mesh a effectivement été supprimé.
	 */
	public boolean deleteMesh(long ID) {
		return meshes.removeElement(ID);
	}

	/**
	 * Teste la présence d'un staticmesh dans la simulation.
	 * 
	 * @param ID
	 * @return
	 */
	public boolean containsMesh(long ID) {
		return meshes.contains(ID);
	}

	/**
	 * Ajoute une nouvelle contrainte entre deux objets dans la simulation.
	 * 
	 * @param constraint
	 */
	public void addConstraint(AbstractConstraint constraint) {
		constraints.add(constraint);
		bodies.addConstraint(constraint);
	}

	/**
	 * Supprime une contrainte
	 * 
	 * @param constraint
	 */
	public void deleteConstraint(AbstractConstraint constraint) {
		if (constraints.remove(constraint)) {
			bodies.removeConstraint(constraint);
		}
	}

	/**
	 * Ajoute une fonction appellée à chaque update de la simulation.
	 * 
	 * @param actor
	 */
	public void addActor(Actor actor) {
		actors.add(actor);
	}

	/**
	 * Supprime l'ensemble des objets de la simulation.
	 */
	public void cleanUp() {
		if (activeRecord != null) {
			stopRecording();
		}

		if (recordPlayers != null) {
			for (PhysicsPlayer player : recordPlayers) {
				player.close();
			}
		}

		bodies.cleanUp();
		meshes.cleanUp();
		constraints.clear();
	}

	/**
	 * @return Un it�rable sur les corps rigide de la simulation. Il ne faut en
	 *         aucun cas s'en servir pour ajouter ou supprimer un rigidbody dans la
	 *         simulation.
	 */
	public Iterable<RigidBody> getBodies() {
		return bodies;
	}

	/**
	 * @return Un it�rable sur les maillage statiques de la simulation. Il ne faut
	 *         en aucun cas s'en servir pour ajouter ou supprimer un staticmesh dans
	 *         la simulation.
	 */
	public Iterable<StaticMesh> getMeshes() {
		return meshes;
	}

	/**
	 * @return Un it�rable sur les contraintes de la simulation. Il ne faut en aucun
	 *         cas s'en servir pour ajouter ou supprimer des contraintes dans la
	 *         simulation.
	 */
	public Iterable<AbstractConstraint> getConstraints() {
		return constraints;
	}

	/**
	 * @return Les param�tres par défaut de la simulation.
	 */
	public DefaultParameters getParameters() {
		return params;
	}

	/**
	 * @return la nombre de nanosecondes écoulées lors de la mise à jour de la
	 *         simulation.
	 */
	public double getLastUpdateDeltaNanosec() {
		return stats.globalUpdate.getDeltaNanos();
	}

	/**
	 * This method should be called before {@link #start()} in order to record
	 * everything from the start.
	 * 
	 * @param path
	 * @throws IOException
	 */
	public void startRecording(String path) throws IOException {
		if (activeRecord != null) {
			throw new IllegalStateException("Cannot start a new record while the world is already being recorded");
		}
		activeRecord = new PhysicsRecorder(path, getElapsedTime());
	}

	public PhysicsRecorder getActiveRecord() {
		return activeRecord;
	}

	public void stopRecording() {
		if (activeRecord == null) {
			throw new IllegalStateException("The world is not currently being recorded");
		}
		activeRecord.close(this);
		activeRecord = null;
	}

	public void addPlaybackRecord(PhysicsPlayer player) {
		if (recordPlayers == null) {
			recordPlayers = new ArrayList<PhysicsPlayer>();
		}
		recordPlayers.add(player);
	}

	public List<PhysicsPlayer> getRecordPlayers() {
		return recordPlayers;
	}

}
