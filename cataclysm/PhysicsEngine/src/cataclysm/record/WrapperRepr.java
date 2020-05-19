package cataclysm.record;

import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapperData;
import cataclysm.wrappers.MassProperties;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.SphereWrapper;
import cataclysm.wrappers.Transform;
import cataclysm.wrappers.TransformableVec3;
import cataclysm.wrappers.Wrapper;

public final class WrapperRepr implements ReadWriteObject {

	public final Transform wrapperToBody = new Transform();
	public final TransformableVec3 centroid = new TransformableVec3();
	public float maxRadius;
	public final MassProperties massProperties = new MassProperties(0, 0, false, 0);

	public int type;// 0 1 2 -> Sphere, Capsule, ConvexHull
	// sphere
	public float sphereRadius;

	// Capsule
	public float capsuleRadius;
	public float halfLength;

	// ConvexHull
	public ConvexHullWrapperData data;
	// ConvexHull with flat layout
	public int faceCount;
	public int edgeCount;
	public int vertexCount;
	public short[] intData;
	public float[] floatData;

	public WrapperRepr() {
	}

	public WrapperRepr(RecordFile f) {
		read(f);
	}

	public Wrapper build(RigidBody b, long ID) {
		if (type == 0) {
			return new SphereWrapper(b, this, ID);
		} else if (type == 1) {
			return new CapsuleWrapper(b, this, ID);
		} else if (type == 2) {
			return new ConvexHullWrapper(b, this, ID);
		} else {
			throw new RecordFileDecodeError("Error while decoding record file !");
		}

	}

	@Override
	public void read(RecordFile f) {
		wrapperToBody.read(f);
		centroid.read(f);
		maxRadius = f.readFloat();
		massProperties.read(f);

		type = f.readInt();
		if (type == 0) {
			sphereRadius = f.readFloat();
		} else if (type == 1) {
			capsuleRadius = f.readFloat();
			halfLength = f.readFloat();
		} else if (type == 2) {
			vertexCount = f.readInt();
			edgeCount = f.readInt();
			faceCount = f.readInt();
			intData = f.readShortArray();
			floatData = f.readFloatArray();
		} else {
			throw new RecordFileDecodeError("Error while decoding record file !");
		}

	}

	@Override
	public void write(RecordFile f) {
		wrapperToBody.write(f);
		centroid.write(f);
		f.writeFloat(maxRadius);
		massProperties.write(f);

		f.writeInt(type);
		if (type == 0) {
			f.writeFloat(sphereRadius);
		} else if (type == 1) {
			f.writeFloat(capsuleRadius);
			f.writeFloat(halfLength);
		} else if (type == 3) {
			f.writeInt(vertexCount);
			f.writeInt(edgeCount);
			f.writeInt(faceCount);

			f.writeShortArray(intData);
			f.writeFloatArray(floatData);
		} else {
			throw new RecordFileDecodeError("Error while encoding record file !");
		}
	}

	@Override
	public int size() {
		int typeSize = 4;
		if (type == 0) {
			typeSize += 4;
		} else if (type == 1) {
			typeSize += 8;
		} else if (type == 2) {
			typeSize += 12 + (intData.length + 1 + floatData.length + 1) * 4;
		}
		return wrapperToBody.size() + centroid.size() + 4 + massProperties.size() + typeSize;
	}

}
