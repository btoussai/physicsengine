package cataclysm.record;

import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Represents the state of a rigidbody
 * 
 * @author Briac
 *
 */
public final class RigidBodyState implements ReadWriteObject {
	public long ID;
	public final Vector3f position = new Vector3f();
	public final Matrix3f rotation = new Matrix3f();
	public final Vector3f velocity = new Vector3f();
	public final Vector3f angularVelocity = new Vector3f();
	
	public RigidBodyState() {
	}
	
	public RigidBodyState(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		ID = f.readLong();
		f.readVector3f(position);
		f.readMatrix3f(rotation);
		f.readVector3f(velocity);
		f.readVector3f(angularVelocity);
	}

	@Override
	public void write(RecordFile f) {
		f.writeLong(ID);
		f.writeVector3f(position);
		f.writeMatrix3f(rotation);
		f.writeVector3f(velocity);
		f.writeVector3f(angularVelocity);
	}

	@Override
	public int size() {
		return 8 + 3*4 + 9*4 + 3*4 + 3*4;
	}
}
