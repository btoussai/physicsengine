package cataclysm.broadphase.staticmeshes;

import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Repr�sente les donn�es g�om�triques d'un staticmesh.
 * @author Briac
 *
 */
public class StaticMeshData {
	
	private int[] indices;
	private Vector3f[] vertices;
	
	/**
	 * Permet de construire un {@link StaticMesh}.
	 * Les donn�es g�om�triques doivent �tre celles d'un maillage triangulaire.
	 * 
	 * @param indices La liste d'indices permettant de construire les triangles du maillage.
	 * @param vertices Les sommets constituant le maillage.
	 */
	public StaticMeshData(int[] indices, Vector3f[] vertices) {
		this.indices = indices;
		this.vertices = vertices;
	}
	
	/**
	 * Construit les triangles du maillage.
	 * @param max Un vecteur dans lequel stocker les coordonn�es max du maillage.
	 * @param min Un vecteur dans lequel stocker les coordonn�es min du maillage.
	 * @param transformation Une matrice de transformation appliquée aux sommets.
	 * @return
	 */
	Triangle[] buildTriangles(StaticMesh mesh, Matrix4f transform, Vector3f min, Vector3f max) {
		min.set(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
		max.set(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
		
		Vector3f[] copy =  new Vector3f[vertices.length];
		for(int i=0; i<vertices.length; i++) {
			Vector3f v = transformVertex(vertices[i], transform);
			min.x = Math.min(min.x, v.x);
			min.y = Math.min(min.y, v.y);
			min.z = Math.min(min.z, v.z);
			
			max.x = Math.max(max.x, v.x);
			max.y = Math.max(max.y, v.y);
			max.z = Math.max(max.z, v.z);
			copy[i] = v;
		}
		
		Triangle[] triangles = new Triangle[indices.length / 3];
		for(int i=0; i<triangles.length; i++) {
			triangles[i] = new Triangle(mesh, copy[indices[3*i]], copy[indices[3*i+1]], copy[indices[3*i+2]]);
		}
		return triangles;
	}
	
	private Vector3f transformVertex(Vector3f vertex, Matrix4f transform) {
		float x = transform.m00 * vertex.x + transform.m10 * vertex.y + transform.m20 * vertex.z + transform.m30;
		float y = transform.m01 * vertex.x + transform.m11 * vertex.y + transform.m21 * vertex.z + transform.m31;
		float z = transform.m02 * vertex.x + transform.m12 * vertex.y + transform.m22 * vertex.z + transform.m32;
		return new Vector3f(x, y, z);
	}
	

}
