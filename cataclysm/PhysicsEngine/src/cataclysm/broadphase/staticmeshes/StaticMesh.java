package cataclysm.broadphase.staticmeshes;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.DefaultParameters;
import cataclysm.contact_creation.ContactProperties;
import cataclysm.datastructures.Identifier;

/**
 * Repr�sente un maillage statique de collision pour les d�cors.
 * 
 * @author Briac
 *
 */
public class StaticMesh extends Identifier {

	/**
	 * Les don�es g�om�triques du maillage. Permet d'effectuer des copies
	 * ult�rieures du maillage � d'autres position dans le monde 3D.
	 */
	private final StaticMeshData data;

	/**
	 * Les triangles constituant le maillage.
	 */
	final Triangle[] triangles;

	/**
	 * La matrice de transformation appliqu�es aux donn�es originelles pour
	 * construire les triangles.
	 */
	private final Matrix4f transform;

	/**
	 * Les coordonn�es min des triangles du maillage.
	 */
	final Vector3f min = new Vector3f();

	/**
	 * Les coordonn�es max des triangles du maillage.
	 */
	final Vector3f max = new Vector3f();

	/**
	 * La friction et l'�lasticit� du sol.
	 */
	private final ContactProperties contactProperties;

	/**
	 * Construit un nouveau maillage de collision statique.
	 * 
	 * @param data      Les donn�es g�om�triques du maillage.
	 * @param transform Une transformation � appliquer aux donn�es g�om�triques pour
	 *                  construire les triangles du maillage.
	 * @param keepData  Indique s'il faut conserver les donn�es g�om�triques. Ceci
	 *                  permet de dupliquer le maillage par la suite et de
	 *                  l'afficher. Laisser � false si le maillage statique ne sera
	 *                  jamais dupliqu�.
	 * @param ID        Un identifinant unique.
	 */
	StaticMesh(StaticMeshData data, Matrix4f transform, DefaultParameters params, boolean keepData, long ID) {
		super(ID);
		triangles = data.buildTriangles(this, transform, min, max);
		if (keepData) {
			this.data = data;
			this.transform = new Matrix4f(transform);
		} else {
			this.data = null;
			this.transform = null;
		}

		contactProperties = new ContactProperties(params.getContactProperties());
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
	 * @param ID        Un identifiant unique.
	 */
	StaticMesh(Matrix4f transform, StaticMesh other, boolean keepData, long ID) {
		super(ID);
		if (other.data == null) {
			throw new NullPointerException(
					"Erreur, le static mesh � recopier n'a pas conserv� ses donn�es g�om�triques.");
		}
		triangles = other.data.buildTriangles(this, transform, min, max);
		if (keepData) {
			this.data = other.data;
			this.transform = new Matrix4f(transform);
		} else {
			this.data = null;
			this.transform = null;
		}
		this.contactProperties = new ContactProperties(other.contactProperties);
	}

	public Matrix4f getTransform() {
		return transform;
	}

	public ContactProperties getContactProperties() {
		return contactProperties;
	}

}