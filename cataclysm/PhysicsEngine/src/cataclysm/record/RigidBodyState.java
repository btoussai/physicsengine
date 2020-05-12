package cataclysm.record;

import cataclysm.wrappers.Transform;
import math.vector.Vector3f;

/**
 * Represents the state of a rigidbody
 * 
 * @author Briac
 *
 */
public final class RigidBodyState implements ReadWriteObject {
	public long ID;
	public final Transform bodyToWorld = new Transform();
	public final Transform barycentricToWorld = new Transform();
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
		bodyToWorld.read(f);
		barycentricToWorld.read(f);
		f.readVector3f(velocity);
		f.readVector3f(angularVelocity);
	}

	@Override
	public void write(RecordFile f) {
		f.writeLong(ID);
		bodyToWorld.write(f);
		barycentricToWorld.write(f);
		f.writeVector3f(velocity);
		f.writeVector3f(angularVelocity);
	}

	@Override
	public int size() {
		return 8 + bodyToWorld.size() + barycentricToWorld.size() + 3*4 + 3*4;
	}
}
