package cataclysm.wrappers;

import cataclysm.record.ReadWriteObject;
import cataclysm.record.RecordFile;
import math.vector.Vector3f;

/**
 * Repr�sente une arr�te de l'enveloppe convexe.
 * 
 * @author Briac
 *
 */
public class ConvexHullWrapperHalfEdge implements ReadWriteObject{

	ConvexHullWrapperData data;

	int tail = -1;
	int index = -1;
	int next = -1;
	int prev = -1;
	int twin = -1;
	int face = -1;

	public ConvexHullWrapperHalfEdge() {
		
	}

	public ConvexHullWrapperHalfEdge(ConvexHullWrapperHalfEdge edge, ConvexHullWrapperData data) {
		this.data = data;
		this.tail = edge.tail;
		this.index = edge.index;
		this.next = edge.next;
		this.prev = edge.prev;
		this.twin = edge.twin;
		this.face = edge.face;
	}

	public Vector3f getTail() {
		return data.vertices[tail];
	}

	public ConvexHullWrapperHalfEdge getNext() {
		return data.edges[next];
	}

	public ConvexHullWrapperHalfEdge getPrev() {
		return data.edges[prev];
	}

	public ConvexHullWrapperHalfEdge getTwin() {
		return data.edges[twin];
	}

	public Vector3f getHead() {
		return getNext().getTail();
	}

	public Vector3f getFaceNormal() {
		return data.faces[face].getNormal();
	}

	public Vector3f getAdjacentFaceNormal() {
		return getTwin().getFaceNormal();
	}

	public void setTail(int tail) {
		this.tail = tail;
	}

	public void setIndex(int index) {
		this.index = index;
	}

	public void setNext(int next) {
		this.next = next;
	}

	public void setPrev(int prev) {
		this.prev = prev;
	}

	public void setTwin(int twin) {
		this.twin = twin;
	}

	public void setFace(int face) {
		this.face = face;
	}

	public int getTwinIndex() {
		return twin;
	}

	public int getFaceIndex() {
		return face;
	}

	public int getPrevIndex() {
		return prev;
	}

	public int getNextIndex() {
		return next;
	}

	public int getIndex() {
		return index;
	}

	public int getTailIndex() {
		return tail;
	}
	
	public ConvexHullWrapperHalfEdge(RecordFile f, ConvexHullWrapperData data){
		read(f);
		this.data = data;
	}

	@Override
	public void read(RecordFile f) {
		tail = f.readInt();
		index = f.readInt();
		next = f.readInt();
		prev = f.readInt();
		twin = f.readInt();
		face = f.readInt();
	}

	@Override
	public void write(RecordFile f) {
		f.writeInt(tail);
		f.writeInt(index);
		f.writeInt(next);
		f.writeInt(prev);
		f.writeInt(twin);
		f.writeInt(face);
	}

}


