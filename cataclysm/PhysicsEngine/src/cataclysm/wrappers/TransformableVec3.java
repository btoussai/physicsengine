package cataclysm.wrappers;

import org.lwjgl.util.vector.Vector3f;

/**
 * Représente un ensemble de deux {@link Vector3f} représentant un même vecteur dans deux repères.
 * 
 * @author Briac
 *
 */
public class TransformableVec3 {

	private final Vector3f inputSpaceCoord;
	private final Vector3f outputSpaceCoord;

	public TransformableVec3(Vector3f inputSpaceCoord) {
		this.inputSpaceCoord = new Vector3f(inputSpaceCoord);
		this.outputSpaceCoord = new Vector3f();
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
		return "ModelSpace: " + inputSpaceCoord + "    WorldSpace: " + outputSpaceCoord;
	}

}
