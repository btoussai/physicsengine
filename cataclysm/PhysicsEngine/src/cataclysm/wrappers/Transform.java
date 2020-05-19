package cataclysm.wrappers;

import cataclysm.record.ReadWriteObject;
import cataclysm.record.RecordFile;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Représente une transformation d'un repère vers un autre.
 * 
 * @author Briac
 *
 */
public final class Transform implements ReadWriteObject {

	private final Matrix3f rotation = new Matrix3f();
	private final Vector3f translation = new Vector3f();

	public Transform() {

	}

	public Transform(Matrix3f rotation, Vector3f translation) {
		Matrix3f.load(rotation, this.rotation);
		this.translation.set(translation);
	}

	public Transform(Matrix4f transform) {
		loadFrom(transform);
	}

	public Transform(Transform transform) {
		this.translation.set(transform.translation);
		this.rotation.load(transform.rotation);
	}

	public Transform translate(Vector3f translation) {
		this.translation.x += translation.x;
		this.translation.y += translation.y;
		this.translation.z += translation.z;
		return this;
	}

	public Transform translate(float x, float y, float z) {
		this.translation.x += x;
		this.translation.y += y;
		this.translation.z += z;
		return this;
	}

	public Transform rotateLeft(Matrix3f rotation) {
		Matrix3f.mul(rotation, this.rotation, this.rotation);
		return this;
	}

	public Transform rotateLeft(Matrix4f rotation) {
		MatrixOps.matrixMult(rotation, this.rotation, this.rotation);
		return this;
	}

	public Transform rotateRight(Matrix3f rotation) {
		Matrix3f.mul(this.rotation, rotation, this.rotation);
		return this;
	}

	public Transform rotateRight(Matrix4f rotation) {
		MatrixOps.matrixMult(this.rotation, rotation, this.rotation);
		return this;
	}

	public void transformVertex(Vector3f vertex, Vector3f dest) {
		float x = rotation.m00 * vertex.x + rotation.m10 * vertex.y + rotation.m20 * vertex.z + translation.x;
		float y = rotation.m01 * vertex.x + rotation.m11 * vertex.y + rotation.m21 * vertex.z + translation.y;
		float z = rotation.m02 * vertex.x + rotation.m12 * vertex.y + rotation.m22 * vertex.z + translation.z;
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	public void transformVertex(float[] src, int srcIndex, float[] dest, int destIndex) {
		float sx = src[srcIndex + 0];
		float sy = src[srcIndex + 1];
		float sz = src[srcIndex + 2];
		float x = rotation.m00 * sx + rotation.m10 * sy + rotation.m20 * sz + translation.x;
		float y = rotation.m01 * sx + rotation.m11 * sy + rotation.m21 * sz + translation.y;
		float z = rotation.m02 * sx + rotation.m12 * sy + rotation.m22 * sz + translation.z;
		dest[destIndex + 0] = x;
		dest[destIndex + 1] = y;
		dest[destIndex + 2] = z;
	}

	public void invertTransformVertex(Vector3f vertex, Vector3f dest) {
		float x = vertex.x - translation.x;
		float y = vertex.y - translation.y;
		float z = vertex.z - translation.z;

		dest.x = rotation.m00 * x + rotation.m01 * y + rotation.m02 * z;
		dest.y = rotation.m10 * x + rotation.m11 * y + rotation.m12 * z;
		dest.z = rotation.m20 * x + rotation.m21 * y + rotation.m22 * z;
	}

	public void transformVector(Vector3f vector, Vector3f dest) {
		float x = rotation.m00 * vector.x + rotation.m10 * vector.y + rotation.m20 * vector.z;
		float y = rotation.m01 * vector.x + rotation.m11 * vector.y + rotation.m21 * vector.z;
		float z = rotation.m02 * vector.x + rotation.m12 * vector.y + rotation.m22 * vector.z;
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	public void transformVector(float[] src, int srcIndex, float[] dest, int destIndex) {
		float sx = src[srcIndex + 0];
		float sy = src[srcIndex + 1];
		float sz = src[srcIndex + 2];
		float x = rotation.m00 * sx + rotation.m10 * sy + rotation.m20 * sz;
		float y = rotation.m01 * sx + rotation.m11 * sy + rotation.m21 * sz;
		float z = rotation.m02 * sx + rotation.m12 * sy + rotation.m22 * sz;
		dest[destIndex + 0] = x;
		dest[destIndex + 1] = y;
		dest[destIndex + 2] = z;
	}

	public void invertTransformVector(Vector3f vector, Vector3f dest) {
		float x = rotation.m00 * vector.x + rotation.m01 * vector.y + rotation.m02 * vector.z;
		float y = rotation.m10 * vector.x + rotation.m11 * vector.y + rotation.m12 * vector.z;
		float z = rotation.m20 * vector.x + rotation.m21 * vector.y + rotation.m22 * vector.z;
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	public Transform invert(Transform dest) {
		return invert(this, dest);
	}

	public static Transform invert(Transform src, Transform dest) {
		if (dest == null) {
			dest = new Transform();
		}

		Matrix3f.transpose(src.rotation, dest.rotation);
		Matrix3f.transform(dest.rotation, src.translation, dest.translation);
		dest.translation.negate();

		return dest;
	}

	public static Transform compose(Transform left, Transform right, Transform dest) {
		if (dest == null) {
			dest = new Transform();
		}
		// In case dest == left
		float x = left.translation.x;
		float y = left.translation.y;
		float z = left.translation.z;
		Matrix3f.transform(left.rotation, right.translation, dest.translation);
		dest.translation.translate(x, y, z);
		Matrix3f.mul(left.rotation, right.rotation, dest.rotation);

		return dest;
	}

	public static Matrix4f compose(Transform left, Matrix4f right, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		Matrix3f rotation = left.rotation;
		MatrixOps.matrixMult(rotation, right, dest);

		float x = rotation.m00 * right.m30 + rotation.m10 * right.m31 + rotation.m20 * right.m32 + left.translation.x;
		float y = rotation.m01 * right.m30 + rotation.m11 * right.m31 + rotation.m21 * right.m32 + left.translation.y;
		float z = rotation.m02 * right.m30 + rotation.m21 * right.m31 + rotation.m22 * right.m32 + left.translation.z;

		dest.m30 = x;
		dest.m31 = y;
		dest.m32 = z;
		dest.m33 = 1;

		return dest;
	}

	public void loadTo(Matrix4f dest) {
		dest.m00 = rotation.m00;
		dest.m01 = rotation.m01;
		dest.m02 = rotation.m02;
		dest.m03 = 0;

		dest.m10 = rotation.m10;
		dest.m11 = rotation.m11;
		dest.m12 = rotation.m12;
		dest.m13 = 0;

		dest.m20 = rotation.m20;
		dest.m21 = rotation.m21;
		dest.m22 = rotation.m22;
		dest.m23 = 0;

		dest.m30 = translation.x;
		dest.m31 = translation.y;
		dest.m32 = translation.z;
		dest.m33 = 1.0f;
	}

	public void loadFrom(Matrix4f src) {
		MatrixOps.loadMatrix(src, rotation, false);
		this.translation.set(src.m30, src.m31, src.m32);
	}

	public void loadFrom(Transform transform) {
		this.translation.set(transform.translation);
		this.rotation.load(transform.rotation);
	}

	public Matrix3f getRotation() {
		return rotation;
	}

	public Vector3f getTranslation() {
		return translation;
	}

	@Override
	public String toString() {
		return "Transform: Rotation =\n" + this.rotation + "Translation = " + this.translation + "\n";
	}

	public Transform(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		f.readMatrix3f(rotation);
		f.readVector3f(translation);
	}

	@Override
	public void write(RecordFile f) {
		f.writeMatrix3f(rotation);
		f.writeVector3f(translation);
	}

	@Override
	public int size() {
		return 3 * 4 + 9 * 4;
	}

}
