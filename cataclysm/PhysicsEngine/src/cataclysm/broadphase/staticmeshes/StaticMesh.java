package cataclysm.broadphase.staticmeshes;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.HashSet;

import cataclysm.DefaultParameters;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.contact_creation.ContactProperties;
import cataclysm.datastructures.Identifier;
import cataclysm.record.StaticMeshRepr;
import cataclysm.record.TriangleRepr;
import cataclysm.wrappers.ConvexHullWrapperData.ModelData;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Defines a static collision mesh in the simulation. As the name implies, these
 * collision meshes are undeformable and cannot be moved.
 * 
 * @author Briac Toussaint
 *
 */
public class StaticMesh extends Identifier {

	/**
	 * Les don�es g�om�triques du maillage. Permet d'effectuer des copies
	 * ult�rieures du maillage � d'autres position dans le monde 3D.
	 */
	private final StaticMeshData data;
	
	/**
	 * A list of the contacts between this static mesh and rigid bodies.
	 */
	protected final Set<AbstractSingleBodyContact> bodyContacts = new HashSet<AbstractSingleBodyContact>(1);

	/**
	 * Les triangles constituant le maillage.
	 */
	final Triangle[] triangles;

	/**
	 * La matrice de transformation appliqu�es aux donn�es originelles pour
	 * construire les triangles.
	 */
	private final Matrix4f transform = new Matrix4f();

	/**
	 * Les coordonnées min des triangles du maillage.
	 */
	final Vector3f min = new Vector3f();

	/**
	 * Les coordonnéSes max des triangles du maillage.
	 */
	final Vector3f max = new Vector3f();

	/**
	 * La friction et l'élasticitée du sol.
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
			this.transform.load(transform);
		} else {
			this.data = null;
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
			this.transform.load(transform);
		} else {
			this.data = null;
		}
		this.contactProperties = new ContactProperties(other.contactProperties);
	}

	StaticMesh(StaticMeshRepr m, long ID) {
		super(ID);
		this.min.set(m.min);
		this.max.set(m.max);
		this.contactProperties = new ContactProperties(m.contactProperties);
		this.triangles = new Triangle[m.triangles.getElementCount()];
		for (int i = 0; i < m.triangles.getElementCount(); i++) {
			TriangleRepr t = m.triangles.get(i);
			triangles[i] = new Triangle(this, new Vector3f(t.v1), new Vector3f(t.v2), new Vector3f(t.v3));
		}

		this.data = null;
	}

	public Matrix4f getTransform() {
		return transform;
	}

	public ContactProperties getContactProperties() {
		return contactProperties;
	}

	public void fill(StaticMeshRepr m) {
		m.ID = this.getID();
		m.min.set(min);
		m.max.set(max);
		m.contactProperties.set(contactProperties);
		m.triangles.rewind();
		for (Triangle t : triangles) {
			TriangleRepr tRepr = m.triangles.getNext();
			t.getV0(tRepr.v1);
			t.getV1(tRepr.v2);
			t.getV2(tRepr.v3);
		}
	}
	
	public ModelData asModel() {
		ModelData data = new ModelData();
		
		List<Integer> indices = new ArrayList<>();
		List<Vector3f> vertices = new ArrayList<>();
		asModel(indices, vertices);

		data.indices = new int[indices.size()];
		data.vertices = new float[vertices.size() * 3];
		
		for(int i=0; i<indices.size(); i++) {
			data.indices[i] = indices.get(i);
		}
		for(int i=0; i<vertices.size(); i++) {
			data.vertices[3*i+0] = vertices.get(i).x;
			data.vertices[3*i+1] = vertices.get(i).y;
			data.vertices[3*i+2] = vertices.get(i).z;
		}
		
		return data;
	}

	public void asModel(List<Integer> indices, List<Vector3f> vertices) {
		indices.clear();
		vertices.clear();
		int faceIndex = 0;
		for (Triangle t : triangles) {
			vertices.add(t.getV0(new Vector3f()));
			vertices.add(t.getV1(new Vector3f()));
			vertices.add(t.getV2(new Vector3f()));

			indices.add(3 * faceIndex + 0);
			indices.add(3 * faceIndex + 1);
			indices.add(3 * faceIndex + 2);
			faceIndex++;
		}

	}

	public Set<AbstractSingleBodyContact> getBodyContacts() {
		return bodyContacts;
	}
}
