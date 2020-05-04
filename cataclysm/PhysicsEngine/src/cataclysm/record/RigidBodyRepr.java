package cataclysm.record;

import cataclysm.contact_creation.ContactProperties;
import cataclysm.wrappers.Transform;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Represents a rigid body in a compact form
 * 
 * @author Briac
 *
 */
public final class RigidBodyRepr implements ReadWriteObject {

	public long ID;
	public final Transform bodyToWorld = new Transform();
	public final Transform barycentricToWorld = new Transform();
	public final Vector3f velocity = new Vector3f();
	public final Vector3f angularVelocity = new Vector3f();
	public final ContactProperties contactProperties = new ContactProperties(0, 0);
	public float inv_mass;
	public final Vector3f inertia = new Vector3f();
	public final Matrix3f inv_Iws = new Matrix3f();
	public final ReadWriteList<WrapperRepr> wrappers = new ReadWriteList<WrapperRepr>(WrapperRepr::new,
			WrapperRepr::new);

	// public final ReadWriteList<AnchorPoint> anchorPoints; //don't save anchor
	// points for now

	public int mask = 0xFFFFFFFF;
	public int category = 0xFFFFFFFF;
	public boolean sleeping = false;
	public int sleepCounter = 0;
	public byte flags = 0;

	public RigidBodyRepr() {

	}

	public RigidBodyRepr(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		ID = f.readLong();
		bodyToWorld.read(f);
		barycentricToWorld.read(f);
		f.readVector3f(velocity);
		f.readVector3f(angularVelocity);
		contactProperties.read(f);
		inv_mass = f.readFloat();
		f.readVector3f(inertia);
		f.readMatrix3f(inv_Iws);
		wrappers.read(f);

		mask = f.readInt();
		category = f.readInt();
		sleeping = f.readBool();
		sleepCounter = f.readInt();
		flags = f.readByte();
	}

	@Override
	public void write(RecordFile f) {
		f.writeLong(ID);
		bodyToWorld.write(f);
		barycentricToWorld.write(f);
		f.writeVector3f(velocity);
		f.writeVector3f(angularVelocity);
		contactProperties.write(f);
		f.writeFloat(inv_mass);
		f.writeVector3f(inertia);
		f.writeMatrix3f(inv_Iws);
		wrappers.write(f);

		f.writeInt(mask);
		f.writeInt(category);
		f.writeBool(sleeping);
		f.writeInt(sleepCounter);
		f.writeByte(flags);
	}

	@Override
	public int size() {
		return 8 + bodyToWorld.size() + barycentricToWorld.size() + 3 * 4 + 3 * 4 + contactProperties.size() + 4 + 3 * 4
				+ 9 * 4 + wrappers.size() + 4 + 4 + 1 + 4 + 1;
	}

}
