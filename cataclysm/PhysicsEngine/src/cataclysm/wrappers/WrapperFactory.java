package cataclysm.wrappers;

import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.quickHull.ConvexHull;
import cataclysm.quickHull.QuickHull;

/**
 * Cette classe permet d'instancier des enveloppes pour un objet.
 * 
 * @see Wrapper
 * @see RigidBody
 * 
 * @author Briac
 *
 */
public class WrapperFactory {
	
	private static Matrix4f identity = new Matrix4f();
	
	public static WrapperBuilder newSphere(float radius) {		
		return new SphereBuilder(identity, radius);
	}
	
	public static WrapperBuilder newSphere(Matrix4f transform, float radius) {
		return new SphereBuilder(transform, radius);
	}
	
	public static WrapperBuilder newCapsule(float radius, float halfLength) {
		return new CapsuleBuilder(identity, radius, halfLength);
	}
	
	public static WrapperBuilder newCapsule(Matrix4f transform, float radius, float halfLength) {
		return new CapsuleBuilder(transform, radius, halfLength);
	}
	
	public static WrapperBuilder newBox(float sizeX, float sizeY, float sizeZ) {
		return newBox(sizeX, sizeY, sizeZ, null, null);
	}
	
	public static WrapperBuilder newBox(float sizeX, float sizeY, float sizeZ, List<Integer> indices, List<Vector3f> vertices) {

		sizeX = 0.5f * sizeX;
		sizeY = 0.5f * sizeY;
		sizeZ = 0.5f * sizeZ;

		Vector3f v000 = new Vector3f(-sizeX, -sizeY, -sizeZ);
		Vector3f v001 = new Vector3f(-sizeX, -sizeY, sizeZ);
		Vector3f v010 = new Vector3f(-sizeX, sizeY, -sizeZ);
		Vector3f v011 = new Vector3f(-sizeX, sizeY, sizeZ);

		Vector3f v100 = new Vector3f(sizeX, -sizeY, -sizeZ);
		Vector3f v101 = new Vector3f(sizeX, -sizeY, sizeZ);
		Vector3f v110 = new Vector3f(sizeX, sizeY, -sizeZ);
		Vector3f v111 = new Vector3f(sizeX, sizeY, sizeZ);

		Vector3f[] verticesTab = new Vector3f[] { v000, v001, v010, v011, v100, v101, v110, v111 };
		
		return newHull(verticesTab, indices, vertices);
	}
	
	public static WrapperBuilder newCone(float radius, float height, int sides) {
		return newCone(radius, height, sides, null, null);
	}
	
	public static WrapperBuilder newCone(float radius, float height, int sides, List<Integer> indices, List<Vector3f> vertices) {
		if (sides < 3) {
			throw new IllegalArgumentException("Le nombre de côtés d'un cône doit être >= 3, valeur:" + sides);
		}
		Vector3f[] verticesTab = new Vector3f[sides + 1];

		for (int i = 0; i < sides; i++) {
			float angle = 2.0f * (float) Math.PI * i / sides;
			float c = (float)Math.cos(angle);
			float s = (float)Math.sin(angle);
			verticesTab[i] = new Vector3f(radius * c, 0, radius * s);
		}
		verticesTab[sides] = new Vector3f(0, height, 0);

		return newHull(verticesTab, indices, vertices);
	}
	
	public static WrapperBuilder newCylinder(float radius, float height, int sides) {
		return newCylinder(radius, height, sides, null, null);
	}
	
	public static WrapperBuilder newCylinder(float radius, float height, int sides, List<Integer> indices, List<Vector3f> vertices) {
		if (sides < 3) {
			throw new IllegalArgumentException("Le nombre de côtés d'un cylindre doit être >= 3, valeur:" + sides);
		}
		Vector3f[] verticesTab = new Vector3f[2*sides];

		for (int i = 0; i < sides; i++) {
			float angle = 2.0f * (float) Math.PI * i / sides;
			float c = (float)Math.cos(angle);
			float s = (float)Math.sin(angle);
			verticesTab[2*i] = new Vector3f(radius * c, 0.5f*height, radius * s);
			verticesTab[2*i+1] = new Vector3f(radius * c, -0.5f*height, radius * s);
		}

		return newHull(verticesTab, indices, vertices);
	}
	
	public static WrapperBuilder newHull(Vector3f[] verticesTab) {
		return newHull(verticesTab, null, null);
	}
	
	public static WrapperBuilder newHull(Vector3f[] verticesTab, List<Integer> indices, List<Vector3f> vertices) {
		ConvexHull hull = QuickHull.buildConvexHull(verticesTab);
		ConvexHullWrapperData data = hull.convertToWrapperData();
		
		if(indices != null && vertices != null) {
			indices.clear();
			vertices.clear();
			hull.getFaces(indices, vertices);
		}
		
		return new ConvexHullWrapperBuilder(identity, data);
	}

}
