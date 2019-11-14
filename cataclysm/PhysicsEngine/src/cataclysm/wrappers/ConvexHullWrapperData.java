package cataclysm.wrappers;

import org.lwjgl.util.vector.Vector3f;

/**
 * Représente les données nécessaires à une enveloppe convexe pour les
 * collisions.
 * 
 * @author Briac
 *
 */
public class ConvexHullWrapperData {

	/**
	 * L'ensemble des faces de l'enveloppe convexe.
	 */
	protected final ConvexHullWrapperFace[] faces;

	/**
	 * L'ensemble des arrêtes de l'enveloppe convexe. Les arrêtes sont stockées avec
	 * leur jumelle juste à côté.
	 */
	protected final ConvexHullWrapperHalfEdge[] edges;

	/**
	 * L'ensemble des sommets de l'enveloppe convexe.
	 */
	protected final Vector3f[] vertices;

	/**
	 * L'ensemble des normales des faces.
	 */
	protected final Vector3f[] faceNormals;

	/**
	 * L'ensemble des centres géométriques des faces.
	 */
	protected final Vector3f[] faceCentroids;

	/**
	 * L'ensemble des offsets dans les équations des plans des faces.
	 */
	protected final float[] planeOffsets;

	/**
	 * Le rayon maximal de l'enveloppe.
	 */
	protected float maxRadius;
	
	/**
	 * Le facteur d'échelle entre l'enveloppe et son modèle.
	 */
	protected float scale = 1;

	// les positions en wrapper-space des sommets et des normales.
	private final Vector3f[] backupVertices;
	private final Vector3f[] backupFaceNormals;
	private final Vector3f[] backupFaceCentroids;

	public ConvexHullWrapperData(ConvexHullWrapperFace[] faces, ConvexHullWrapperHalfEdge[] edges, Vector3f[] vertices,
			Vector3f[] faceNormals, Vector3f[] faceCentroids) {
		this.faces = faces;
		for (ConvexHullWrapperFace face : faces) {
			face.data = this;
		}

		this.edges = edges;
		for (ConvexHullWrapperHalfEdge edge : edges) {
			edge.data = this;
		}

		this.vertices = vertices;
		this.faceNormals = faceNormals;
		this.faceCentroids = faceCentroids;

		this.backupVertices = vertices;
		this.backupFaceNormals = faceNormals;
		this.backupFaceCentroids = faceCentroids;

		this.planeOffsets = new float[faces.length];
		for (int i = 0; i < faces.length; i++) {
			planeOffsets[i] = Vector3f.dot(faceCentroids[i], faceNormals[i]);
		}

		float maxRadius = 0;
		for (Vector3f vec : vertices) {
			maxRadius = Math.max(maxRadius, vec.lengthSquared());
		}
		this.maxRadius = (float) Math.sqrt(maxRadius);
	}

	ConvexHullWrapperData(ConvexHullWrapperData data) {
		this.faces = new ConvexHullWrapperFace[data.faces.length];
		for (int i = 0; i < faces.length; i++) {
			faces[i] = new ConvexHullWrapperFace(data.faces[i], this);
		}

		this.edges = new ConvexHullWrapperHalfEdge[data.edges.length];
		for (int i = 0; i < edges.length; i++) {
			edges[i] = new ConvexHullWrapperHalfEdge(data.edges[i], this);
		}

		this.vertices = new Vector3f[data.backupVertices.length];
		this.backupVertices = new Vector3f[data.backupVertices.length];
		for (int i = 0; i < vertices.length; i++) {
			vertices[i] = new Vector3f();
			backupVertices[i] = new Vector3f(data.backupVertices[i]);
		}

		faceNormals = new Vector3f[faces.length];
		backupFaceNormals = new Vector3f[faces.length];
		faceCentroids = new Vector3f[faces.length];
		backupFaceCentroids = new Vector3f[faces.length];
		planeOffsets = new float[faces.length];
		for (int i = 0; i < faces.length; i++) {
			faceNormals[i] = new Vector3f();
			backupFaceNormals[i] = new Vector3f(data.backupFaceNormals[i]);

			faceCentroids[i] = new Vector3f();
			backupFaceCentroids[i] = new Vector3f(data.backupFaceCentroids[i]);
		}

		this.scale = data.scale;
		this.maxRadius = data.maxRadius;
	}

	/**
	 * Applique une transformation wrapper-space -> world-space sur les sommets et
	 * les normales de l'enveloppe convexe.
	 * 
	 * @param wrapperToWorld
	 */
	void tranform(Transform wrapperToWorld) {
		for (int i = 0; i < faces.length; i++) {
			wrapperToWorld.transformVector(backupFaceNormals[i], faceNormals[i]);
		}

		for (int i = 0; i < vertices.length; i++) {
			wrapperToWorld.transformVertex(backupVertices[i], vertices[i]);
		}

		for (int i = 0; i < faces.length; i++) {
			wrapperToWorld.transformVertex(backupFaceCentroids[i], faceCentroids[i]);
			planeOffsets[i] = Vector3f.dot(faceCentroids[i], faceNormals[i]);
		}

	}

	/**
	 * Applique un changement d'échelle sur les sommets en wrapper-space.
	 * 
	 * @param scaleFactor
	 * @param origin      Le point servant d'origine pour appliquer le changement
	 *                    d'échelle.
	 */
	void scale(float scaleFactor, Vector3f origin) {

		// On applique le changement d'échelle aux sommets.
		for (int i = 0; i < backupVertices.length; i++) {
			Vector3f v = backupVertices[i];
			v.x = origin.x + (v.x - origin.x) * scaleFactor;
			v.y = origin.y + (v.y - origin.y) * scaleFactor;
			v.z = origin.z + (v.z - origin.z) * scaleFactor;
		}

		// Puis aux centres des faces
		for (int i = 0; i < backupFaceCentroids.length; i++) {
			Vector3f v = backupFaceCentroids[i];
			v.x = origin.x + (v.x - origin.x) * scaleFactor;
			v.y = origin.y + (v.y - origin.y) * scaleFactor;
			v.z = origin.z + (v.z - origin.z) * scaleFactor;
		}
		
		this.maxRadius *= scaleFactor;
		this.scale *= scaleFactor;
	}

	void translate(float x, float y, float z) {
		// On applique le changement d'échelle aux sommets.
		for (int i = 0; i < backupVertices.length; i++) {
			backupVertices[i].translate(x, y, z);
		}

		// Puis aux centres des faces
		for (int i = 0; i < backupFaceCentroids.length; i++) {
			backupFaceCentroids[i].translate(x, y, z);
		}
	}

	/**
	 * @return Le facteur d'echelle entre le modèle et les sommets de l'enveloppe.
	 */
	public float getScale() {
		return scale;
	}

}
