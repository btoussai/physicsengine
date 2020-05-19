package cataclysm.wrappers;

import math.vector.Vector3f;

public class ConvexHullWrapperData {
	
	public final int faceCount;
	public final int edgeCount;
	public final int vertexCount;
	public final short[] intData;
	public final float[] floatData;
	public final float maxRadius;
	
	public ConvexHullWrapperData(int faceCount, int edgeCount, int vertexCount, short[] intData,
			float[] floatData, float maxRadius) {
		this.faceCount = faceCount;
		this.edgeCount = edgeCount;
		this.vertexCount = vertexCount;
		this.intData = intData;
		this.floatData = floatData;
		this.maxRadius = maxRadius;
	}

	public void scale(float scaleFactor, Vector3f vector3f) {
		throw new IllegalStateException("Not implemented !");
	}
	
	

}
