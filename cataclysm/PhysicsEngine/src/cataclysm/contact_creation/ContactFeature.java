package cataclysm.contact_creation;

import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;
import math.vector.Vector3f;


/**
 * Indique quelle partie d'un objet est en collision dans un {@link Contact}.
 * @author Briac
 *
 */
class ContactFeature {
	
	/**
	 * Indique le type de la partie en collision.
	 * 
	 * @author Briac
	 *
	 */
	enum FeatureType {
		Face, HalfEdge, Vertex, Segment, Triangle, None;
	}
	
	private FeatureType type = FeatureType.None;
	private ConvexHullWrapperFace face;
	private ConvexHullWrapperHalfEdge halfedge;
	private Vector3f v1;
	private Vector3f v2;
	private Vector3f v3;
	
	public ContactFeature() {
		
	}
	
	public void clean() {
		switch(type) {
		case Face:
			face = null;
			break;
		case HalfEdge:
			halfedge = null;
			break;
		case Triangle:
			v3 = null;
		case Segment:
			v2 = null;
		case Vertex:
			v1 = null;
			break;
		default:
			break;
		}
		type = FeatureType.None;
	}
	
	public void setFrom(ContactFeature other){
		clean();
		this.type = other.type;
		switch(other.type) {
		case Face:
			this.face = other.face;
			break;
		case HalfEdge:
			this.halfedge = other.halfedge;
			break;
		case Triangle:
			this.v3 = other.v3;
		case Segment:
			this.v2 = other.v2;
		case Vertex:
			this.v1 = other.v1;
			break;
		case None:
		default:
			break;
		
		}
	}
	
	public void setFrom(ConvexHullWrapperFace face){
		clean();
		this.face = face;
		this.type = FeatureType.Face;
	}
	
	public void setFrom(ConvexHullWrapperHalfEdge halfedge){
		clean();
		this.halfedge = halfedge;
		this.type = FeatureType.HalfEdge;
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
		this.type = FeatureType.Segment;
	}
	
	public void setFrom(Vector3f v1, Vector3f v2, Vector3f v3){
		clean();
		this.v1 = v1;
		this.v2 = v2;
		this.v3 = v3;
		this.type = FeatureType.Triangle;
	}
	
	public FeatureType getType() {
		return type;
	}

	public ConvexHullWrapperFace getFace() {
		return face;
	}

	public ConvexHullWrapperHalfEdge getHalfedge() {
		return halfedge;
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
