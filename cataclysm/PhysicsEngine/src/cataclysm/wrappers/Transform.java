package cataclysm.wrappers;

import cataclysm.record.ReadWriteObject;
import cataclysm.record.RecordFile;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Defines a euclidean transform from one reference frame to another.
 * 
 * @author Briac
 *
 */
public final class Transform implements ReadWriteObject {

	private final Matrix3f r = new Matrix3f();
	private final Vector3f t = new Vector3f();

	public Transform() {

	}

	public Transform(Matrix3f rotation, Vector3f translation) {
		Matrix3f.load(rotation, this.r);
		this.t.set(translation);
	}

	public Transform(Matrix4f transform) {
		loadFrom(transform);
	}

	public Transform(Transform transform) {
		this.t.set(transform.t);
		this.r.load(transform.r);
	}

	public Transform translate(Vector3f translation) {
		this.t.x += translation.x;
		this.t.y += translation.y;
		this.t.z += translation.z;
		return this;
	}

	public Transform translate(float x, float y, float z) {
		this.t.x += x;
		this.t.y += y;
		this.t.z += z;
		return this;
	}

	public Transform rotateLeft(Matrix3f rotation) {
		Matrix3f.mul(rotation, this.r, this.r);
		return this;
	}

	public Transform rotateLeft(Matrix4f rotation) {
		MatrixOps.matrixMult(rotation, this.r, this.r);
		return this;
	}

	public Transform rotateRight(Matrix3f rotation) {
		Matrix3f.mul(this.r, rotation, this.r);
		return this;
	}

	public Transform rotateRight(Matrix4f rotation) {
		MatrixOps.matrixMult(this.r, rotation, this.r);
		return this;
	}

	public void transformVertex(Vector3f vertex, Vector3f dest) {
		float x = r.m00 * vertex.x + r.m10 * vertex.y + r.m20 * vertex.z + t.x;
		float y = r.m01 * vertex.x + r.m11 * vertex.y + r.m21 * vertex.z + t.y;
		float z = r.m02 * vertex.x + r.m12 * vertex.y + r.m22 * vertex.z + t.z;
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	public void transformVertex(float[] src, int srcIndex, float[] dest, int destIndex) {
		float sx = src[srcIndex + 0];
		float sy = src[srcIndex + 1];
		float sz = src[srcIndex + 2];
		float x = r.m00 * sx + r.m10 * sy + r.m20 * sz + t.x;
		float y = r.m01 * sx + r.m11 * sy + r.m21 * sz + t.y;
		float z = r.m02 * sx + r.m12 * sy + r.m22 * sz + t.z;
		dest[destIndex + 0] = x;
		dest[destIndex + 1] = y;
		dest[destIndex + 2] = z;
	}

	public void invertTransformVertex(Vector3f vertex, Vector3f dest) {
		float x = vertex.x - t.x;
		float y = vertex.y - t.y;
		float z = vertex.z - t.z;

		dest.x = r.m00 * x + r.m01 * y + r.m02 * z;
		dest.y = r.m10 * x + r.m11 * y + r.m12 * z;
		dest.z = r.m20 * x + r.m21 * y + r.m22 * z;
	}

	public void transformVector(Vector3f vector, Vector3f dest) {
		float x = r.m00 * vector.x + r.m10 * vector.y + r.m20 * vector.z;
		float y = r.m01 * vector.x + r.m11 * vector.y + r.m21 * vector.z;
		float z = r.m02 * vector.x + r.m12 * vector.y + r.m22 * vector.z;
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}

	public void transformVector(float[] src, int srcIndex, float[] dest, int destIndex) {
		float sx = src[srcIndex + 0];
		float sy = src[srcIndex + 1];
		float sz = src[srcIndex + 2];
		float x = r.m00 * sx + r.m10 * sy + r.m20 * sz;
		float y = r.m01 * sx + r.m11 * sy + r.m21 * sz;
		float z = r.m02 * sx + r.m12 * sy + r.m22 * sz;
		dest[destIndex + 0] = x;
		dest[destIndex + 1] = y;
		dest[destIndex + 2] = z;
	}

	public void invertTransformVector(Vector3f vector, Vector3f dest) {
		float x = r.m00 * vector.x + r.m01 * vector.y + r.m02 * vector.z;
		float y = r.m10 * vector.x + r.m11 * vector.y + r.m12 * vector.z;
		float z = r.m20 * vector.x + r.m21 * vector.y + r.m22 * vector.z;
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

		Matrix3f.transpose(src.r, dest.r);
		Matrix3f.transform(dest.r, src.t, dest.t);
		dest.t.negate();

		return dest;
	}

	public static Transform compose(Transform left, Transform right, Transform dest) {
		if (dest == null) {
			dest = new Transform();
		}
		// In case dest == left
		float x = left.t.x;
		float y = left.t.y;
		float z = left.t.z;
		Matrix3f.transform(left.r, right.t, dest.t);
		dest.t.translate(x, y, z);
		Matrix3f.mul(left.r, right.r, dest.r);

		return dest;
	}

	public static Matrix4f compose(Transform left, Matrix4f right, Matrix4f dest) {
		if (dest == null) {
			dest = new Matrix4f();
		}

		Matrix3f rotation = left.r;
		MatrixOps.matrixMult(rotation, right, dest);

		float x = rotation.m00 * right.m30 + rotation.m10 * right.m31 + rotation.m20 * right.m32 + left.t.x;
		float y = rotation.m01 * right.m30 + rotation.m11 * right.m31 + rotation.m21 * right.m32 + left.t.y;
		float z = rotation.m02 * right.m30 + rotation.m21 * right.m31 + rotation.m22 * right.m32 + left.t.z;

		dest.m30 = x;
		dest.m31 = y;
		dest.m32 = z;
		dest.m33 = 1;

		return dest;
	}

	public void loadTo(Matrix4f dest) {
		dest.m00 = r.m00;
		dest.m01 = r.m01;
		dest.m02 = r.m02;
		dest.m03 = 0;

		dest.m10 = r.m10;
		dest.m11 = r.m11;
		dest.m12 = r.m12;
		dest.m13 = 0;

		dest.m20 = r.m20;
		dest.m21 = r.m21;
		dest.m22 = r.m22;
		dest.m23 = 0;

		dest.m30 = t.x;
		dest.m31 = t.y;
		dest.m32 = t.z;
		dest.m33 = 1.0f;
	}

	public void loadFrom(Matrix4f src) {
		MatrixOps.loadMatrix(src, r, false);
		this.t.set(src.m30, src.m31, src.m32);
	}

	public void loadFrom(Transform transform) {
		this.t.set(transform.t);
		this.r.load(transform.r);
	}

	public Matrix3f getRotation() {
		return r;
	}

	public Vector3f getTranslation() {
		return t;
	}

	@Override
	public String toString() {
		return "Transform: Rotation =\n" + this.r + "Translation = " + this.t + "\n";
	}

	public Transform(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		f.readMatrix3f(r);
		f.readVector3f(t);
	}

	@Override
	public void write(RecordFile f) {
		f.writeMatrix3f(r);
		f.writeVector3f(t);
	}

	@Override
	public int size() {
		return 3 * 4 + 9 * 4;
	}

}
