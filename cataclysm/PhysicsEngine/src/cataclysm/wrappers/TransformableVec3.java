package cataclysm.wrappers;

import cataclysm.record.ReadWriteObject;
import cataclysm.record.RecordFile;
import math.vector.Vector3f;

/**
 * Defines the association of two {@link Vector3f} as the representation of the same vector quantity in two reference frames.
 * 
 * @author Briac Toussaint
 *
 */
public final class TransformableVec3 implements ReadWriteObject {

	private final Vector3f inputSpaceCoord;
	private final Vector3f outputSpaceCoord;

	public TransformableVec3(Vector3f inputSpaceCoord) {
		this.inputSpaceCoord = new Vector3f(inputSpaceCoord);
		this.outputSpaceCoord = new Vector3f();
	}

	public TransformableVec3(TransformableVec3 vec) {
		this.inputSpaceCoord = new Vector3f(vec.inputSpaceCoord);
		this.outputSpaceCoord = new Vector3f(vec.outputSpaceCoord);
	}

	public TransformableVec3(float x, float y, float z) {
		this.inputSpaceCoord = new Vector3f(x, y, z);
		this.outputSpaceCoord = new Vector3f();
	}

	public TransformableVec3() {
		this.inputSpaceCoord = new Vector3f();
		this.outputSpaceCoord = new Vector3f();
	}

	public Vector3f getInputSpaceCoord() {
		return inputSpaceCoord;
	}

	public Vector3f getOutputSpaceCoord() {
		return outputSpaceCoord;
	}

	public void transformAsVertex(Transform transform) {
		transform.transformVertex(inputSpaceCoord, outputSpaceCoord);
	}

	public void transformAsVector(Transform transform) {
		transform.transformVector(inputSpaceCoord, outputSpaceCoord);
	}

	@Override
	public String toString() {
		return "InputSpace: " + inputSpaceCoord + "    OutputSpace: " + outputSpaceCoord;
	}

	@Override
	public void read(RecordFile f) {
		f.readVector3f(inputSpaceCoord);
		f.readVector3f(outputSpaceCoord);
	}

	@Override
	public void write(RecordFile f) {
		f.writeVector3f(inputSpaceCoord);
		f.writeVector3f(outputSpaceCoord);
	}

	public void set(TransformableVec3 vec) {
		this.inputSpaceCoord.set(vec.inputSpaceCoord);
		this.outputSpaceCoord.set(vec.outputSpaceCoord);
	}

	@Override
	public int size() {
		return 3 * 4 + 3 * 4;
	}

}
