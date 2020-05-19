package cataclysm.contact_creation;

import math.vector.Vector3f;


/**
 * Tells which part of an object collides in a {@link ContactZone}.
 * @author Briac
 *
 */
class ContactFeature {
	
	enum FeatureType {
		//Convex hull features
		HullFace, HullEdge, 
		//triangle features and other wrapper type
		TriangleFace, Edge, Vertex, 
		
		None;
	}
	
	private FeatureType type = FeatureType.None;
	private int hullFeatureIndex;
	private Vector3f v1;
	private Vector3f v2;
	private Vector3f v3;
	
	public ContactFeature() {
		
	}
	
	public void clean() {
		switch(type) {
		case HullFace:
		case HullEdge:
			hullFeatureIndex = -1;
		case TriangleFace:
			v3 = null;
		case Edge:
			v2 = null;
		case Vertex:
			v1 = null;
			break;
		case None:
			break;
		default:
			throw new IllegalStateException();
		}
		type = FeatureType.None;
	}
	
	public void setFrom(ContactFeature other){
		clean();
		switch(other.type) {
		case HullFace:
		case HullEdge:
			this.hullFeatureIndex = other.hullFeatureIndex;
			break;
		case TriangleFace:
			this.v3 = other.v3;
		case Edge:
			this.v2 = other.v2;
		case Vertex:
			this.v1 = other.v1;
			break;
		case None:
			break;
		default:
			throw new IllegalStateException();
		}
		this.type = other.type;
	}
	
	public void setFromHullFace(int face){
		clean();
		this.hullFeatureIndex = face;
		this.type = FeatureType.HullFace;
	}
	
	public void setFromHullEdge(int edge){
		clean();
		this.hullFeatureIndex = edge;
		this.type = FeatureType.HullEdge;
	}
	
	public void setFrom(Vector3f v1){
		clean();
		this.v1 = v1;
		this.type = FeatureType.Vertex;
	}
	
	public void setFrom(Vector3f v1, Vector3f v2){
		clean();
		this.v1 = v1;
		this.v2 = v2;
		this.type = FeatureType.Edge;
	}
	
	public void setFrom(Vector3f v1, Vector3f v2, Vector3f v3){
		clean();
		this.v1 = v1;
		this.v2 = v2;
		this.v3 = v3;
		this.type = FeatureType.TriangleFace;
	}
	
	public FeatureType getType() {
		return type;
	}

	public int getHullFeatureIndex() {
		return hullFeatureIndex;
	}

	public Vector3f getV1() {
		return v1;
	}

	public Vector3f getV2() {
		return v2;
	}

	public Vector3f getV3() {
		return v3;
	}

}
